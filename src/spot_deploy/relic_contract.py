"""Explicit, hash-bound contract for the unchanged released ReLIC controller."""

from typing import Literal

import numpy as np
from pydantic import Field, model_validator

from .contracts import (
    ARM, JOINTS, ContractError, Hash, Manifest, Positive, StrictModel, Vector19,
    artifact_path, sha256,
)
from .relic_policy import ACTION_IDS, ACTION_JOINTS, CHECKPOINT_SHA256, DEFAULT_Q, OBS_JOINTS


class ReLICObservations(StrictModel):
    contract: Literal["relic84-v1"]

    @property
    def input_size(self):
        return 84


class Preparation(StrictModel):
    kind: Literal["four-foot-static-support", "direct"]
    model_file: str
    model_sha256: Hash
    duration_s: float = Field(ge=0, le=30)
    foot_radius_m: float = Field(gt=0, le=.1)
    friction_coefficient: float = Field(gt=0, le=2)
    handover_pose_error_rad: Positive
    handover_joint_speed_rad_s: Positive
    handover_torque_step_max: Vector19

    @model_validator(mode="after")
    def limits(self):
        if (self.kind == "direct") != (self.duration_s == 0):
            raise ValueError("direct startup requires zero duration; interpolation requires positive duration")
        if any(v <= 0 for v in self.handover_torque_step_max):
            raise ValueError("handover torque limits must be positive")
        return self


class ReLICSettings(StrictModel):
    height_source: Literal["initial", "fixed"]
    body_height_m: float | None = Field(ge=.25, le=.8)
    initial_height_min_m: float = Field(ge=.25, le=.8)
    initial_height_max_m: float = Field(ge=.25, le=.8)
    preparation: Preparation

    @model_validator(mode="after")
    def height(self):
        if (self.height_source == "fixed") != (self.body_height_m is not None):
            raise ValueError("fixed height needs a value; initial height requires null")
        if self.initial_height_min_m >= self.initial_height_max_m:
            raise ValueError("initial height bounds must be ordered")
        return self


class ReLICManifest(Manifest):
    schema_version: Literal[2]
    adapter: Literal["relic84"]
    observations: ReLICObservations
    relic: ReLICSettings

    @model_validator(mode="after")
    def released_contract(self):
        self.validate_policy_reference()
        if self.policy_hz != 50 or self.stream_hz != 200:
            raise ValueError("this ReLIC rollout uses 50 Hz policy and 200 Hz commands")
        if tuple(self.actions.joint_order) != ACTION_JOINTS:
            raise ValueError("ReLIC action order is joint-type-major")
        if (not np.allclose(self.actions.default_positions, DEFAULT_Q[ACTION_IDS], rtol=0, atol=1e-7)
                or not np.allclose(self.actions.scale, .2, rtol=0, atol=1e-8)
                or self.actions.clip_min is not None or self.actions.clip_max is not None
                or self.actions.previous_action != "raw"):
            raise ValueError("ReLIC defaults, scaling, unclipped raw actions must match release")
        if (self.gains.kp[:12] != [60.] * 12 or self.gains.kd[:12] != [1.5] * 12
                or self.gains.kp[12:] != [120., 120., 120., 100., 100., 100., 16.]
                or self.gains.kd[12:] != [2., 2., 2., 2., 2., 2., .32]
                or self.gains.feedforward != [0.] * 19):
            raise ValueError("ReLIC policy phase requires unchanged released gains and zero feedforward")
        return self

    def validate_policy_reference(self):
        if self.policy_sha256 != CHECKPOINT_SHA256:
            raise ValueError("ReLIC adapter requires the exact released checkpoint")
        if self.input_name != "obs" or self.output_name != "actions":
            raise ValueError("ReLIC tensor names must be obs/actions")


def load_manifest(path):
    import json

    data = json.loads(path.read_text())
    if data.get("schema_version") == 3:
        from .exploy_policy import ExployReLICManifest

        cls = ExployReLICManifest
    else:
        cls = ReLICManifest if data.get("schema_version") == 2 else Manifest
    return cls.model_validate(data)


def make_policy(path, manifest, manifest_path, **kwargs):
    if getattr(manifest, "adapter", None) == "relic-exploy":
        from .exploy_policy import ExployReLICPolicy

        return ExployReLICPolicy(path, manifest, manifest_path, **kwargs)
    if isinstance(manifest, ReLICManifest):
        return ReLICDeploymentPolicy(path, manifest, manifest_path, **kwargs)
    from .policy import OnnxPolicy

    return OnnxPolicy(path, manifest)


class ReLICDeploymentPolicy:
    """One inference contract used in replay, simulation and guarded physical control.

    evaluate() is stateless with respect to action history. The rollout owns
    acknowledged history; predict() retains previous-output semantics for replay.
    """

    def __init__(self, path, manifest, manifest_path, *, device="cpu", profile_prefix=None):
        self.manifest = manifest
        self.runner = self.create_runner(path, manifest_path, device, profile_prefix)
        self.support_model = artifact_path(manifest_path, manifest.relic.preparation.model_file)
        if sha256(self.support_model) != manifest.relic.preparation.model_sha256:
            raise ContractError("ReLIC preparation-model hash mismatch")
        from .static_support import StaticSupport

        prep = manifest.relic.preparation
        self.support = StaticSupport(self.support_model, prep.foot_radius_m,
                                     prep.friction_coefficient)
        self.reset()

    def create_runner(self, path, manifest_path, device, profile_prefix):
        from .relic_policy import ReLICPolicy

        return ReLICPolicy(path, self.manifest.relic.body_height_m or .55,
                           device=device, profile_prefix=profile_prefix)

    def warmup(self):
        """Discard a network output without changing history or the latched height."""
        self.runner.session.run(["actions"], {"obs": np.zeros((1, 84), dtype=np.float32)})

    def reset(self):
        self.previous_action = np.zeros(12, dtype=np.float32)
        self.body_height = self.manifest.relic.body_height_m

    def initialize_height(self, measurement):
        spec = self.manifest.relic
        h = measurement["height_m"]
        if not np.isfinite(h) or not spec.initial_height_min_m <= h <= spec.initial_height_max_m:
            raise ContractError("initial standing height outside ReLIC contract")
        if measurement.get("foot_contacts") != [1] * 4:
            raise ContractError("ReLIC preparation requires four reported foot contacts")
        self.body_height = h if spec.height_source == "initial" else spec.body_height_m

    def evaluate(self, state, previous):
        if self.body_height is None:
            raise ContractError("initial ReLIC body height has not been latched")
        from .relic_policy import finite, joint_targets, observation
        from time import monotonic

        start = monotonic()
        arm = np.asarray(self.manifest.arm_stowed_positions, dtype=np.float32)
        obs = observation(state, arm, previous, self.body_height)
        action = finite(self.runner.session.run(["actions"], {"obs": obs[None]})[0][0], 12)
        target = joint_targets(action, arm).astype(float)
        target[12:] = self.manifest.arm_stowed_positions
        return target, monotonic() - start, obs[None], action

    def predict(self, state):
        result = self.evaluate(state, self.previous_action)
        self.previous_action = result[3].copy()
        return result


def policy_metadata(manifest):
    return dict(adapter="relic84", observation_joint_order=list(OBS_JOINTS),
                command_joint_order=list(JOINTS), held_arm_joint_order=list(ARM),
                previous_actions="zero initially; last acknowledged policy output in rollout",
                velocity_command=[0, 0, 0], mode="four-foot", policy_hz=50,
                stream_hz=manifest.stream_hz, checkpoint_sha256=CHECKPOINT_SHA256)
