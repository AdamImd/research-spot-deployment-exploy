"""Versioned contracts. No hardware limits have implicit/default values."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

LEGS = tuple(f"{leg}_{joint}" for leg in ("fl", "fr", "hl", "hr") for joint in ("hx", "hy", "kn"))
ARM = ("arm0_sh0", "arm0_sh1", "arm0_el0", "arm0_el1", "arm0_wr0", "arm0_wr1", "arm0_f1x")
JOINTS = LEGS + ARM
TERM_SIZES = {
    "body_linear_velocity": 3,
    "body_angular_velocity": 3,
    "projected_gravity": 3,
    "joint_positions": 12,
    "joint_velocities": 12,
    "previous_action": 12,
    "velocity_command": 3,
}
Hash = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
Positive = Annotated[float, Field(gt=0, allow_inf_nan=False)]
Vector19 = Annotated[list[float], Field(min_length=19, max_length=19)]
Vector12 = Annotated[list[float], Field(min_length=12, max_length=12)]
Vector7 = Annotated[list[float], Field(min_length=7, max_length=7)]


class ContractError(ValueError):
    """An artifact or runtime value violates the registered contract."""


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, validate_assignment=True)


class Term(StrictModel):
    name: Literal[
        "body_linear_velocity",
        "body_angular_velocity",
        "projected_gravity",
        "joint_positions",
        "joint_velocities",
        "previous_action",
        "velocity_command",
    ]
    scale: list[float]
    offset: list[float]

    @model_validator(mode="after")
    def dimensions(self):
        if len(self.scale) != TERM_SIZES[self.name] or len(self.offset) != len(self.scale):
            raise ValueError("observation term transform has incorrect dimensions")
        return self


class Observations(StrictModel):
    terms: list[Term]
    joint_order: list[str]
    history_length: int = Field(ge=1, le=64)
    history_initialization: Literal["zeros", "repeat_first"]
    # Transform each term first, concatenate, normalize, clip, then stack oldest first.
    mean: list[float]
    std: list[float]
    # Required even when disabled: null means the training pipeline does not clip.
    clip: Positive | None

    @property
    def frame_size(self) -> int:
        return sum(TERM_SIZES[t.name] for t in self.terms)

    @property
    def input_size(self) -> int:
        return self.frame_size * self.history_length

    @model_validator(mode="after")
    def check(self):
        permutation(self.joint_order, LEGS)
        names = [t.name for t in self.terms]
        required = set(TERM_SIZES) - {"velocity_command"}
        if not required.issubset(names) or len(names) != len(set(names)):
            raise ValueError("missing or duplicate observation terms")
        if len(self.mean) != self.frame_size or len(self.std) != self.frame_size:
            raise ValueError("normalization must describe one complete observation frame")
        if any(x <= 0 for x in self.std):
            raise ValueError("normalization std must be positive")
        return self


class Actions(StrictModel):
    joint_order: list[str]
    default_positions: Vector12
    scale: Vector12
    clip_min: Vector12 | None
    clip_max: Vector12 | None
    previous_action: Literal["raw", "clipped"]

    @model_validator(mode="after")
    def check(self):
        permutation(self.joint_order, LEGS)
        if (self.clip_min is None) != (self.clip_max is None):
            raise ValueError("action clipping requires both bounds or two explicit nulls")
        if self.clip_min is not None and any(
            lo >= hi for lo, hi in zip(self.clip_min, self.clip_max)
        ):
            raise ValueError("action clip bounds must be ordered")
        if any(x == 0 for x in self.scale):
            raise ValueError("zero action scale is not a policy contract")
        return self


class Gains(StrictModel):
    kp: Vector19
    kd: Vector19
    feedforward: Vector19

    @model_validator(mode="after")
    def positive(self):
        if any(x <= 0 for x in self.kp) or any(x < 0 for x in self.kd):
            raise ValueError("standing position gains require kp > 0 and kd >= 0")
        return self


class Manifest(StrictModel):
    schema_version: Literal[1]
    purpose: Literal["fixture", "candidate"]
    task: Literal["standing"]
    morphology: Literal["spot-arm-stowed"]
    policy_file: str
    policy_sha256: Hash
    training_config_file: str
    training_config_sha256: Hash
    training_commit: str = Field(pattern=r"^[0-9a-f]{40}$")
    input_name: str = Field(min_length=1)
    output_name: str = Field(min_length=1)
    policy_hz: Positive
    stream_hz: float = Field(ge=100, le=333)
    observations: Observations
    actions: Actions
    gains: Gains
    arm_stowed_positions: Vector7
    # Qualification must include this exact robot model and inertial configuration.
    robot_model_sha256: Hash
    payload_config_sha256: Hash
    payload_description: str = Field(min_length=1)

    @model_validator(mode="after")
    def timings(self):
        if self.policy_hz > self.stream_hz:
            raise ValueError("policy rate cannot exceed stream rate")
        return self


class Envelope(StrictModel):
    schema_version: Literal[1]
    scope: Literal["hardware", "simulation"] = "hardware"
    joint_order: list[str]
    position_min: Vector19
    position_max: Vector19
    velocity_max: Vector19
    load_max: Vector19
    actuator_limit_profile: Literal["spot-sdk-5.0.1"] | None = None
    # Omission preserves legacy min(per-joint) behavior. An explicit global
    # backstop does not replace the host's tighter per-joint speed checks.
    sdk_velocity_safety_limit: Positive | None = None
    tracking_error_max: Vector19
    command_rate_max: Vector19
    max_roll_rad: Positive
    max_pitch_rad: Positive
    max_linear_speed: Positive
    max_angular_speed: Positive
    max_arm_pose_error: Positive
    max_state_age_s: Positive
    max_state_gap_s: Positive
    max_future_skew_s: Positive
    max_inference_s: Positive
    max_policy_age_s: Positive
    max_command_gap_s: Positive
    max_command_ack_s: Positive
    max_full_state_age_s: Positive
    command_ttl_s: Positive
    shutdown_timeout_s: Positive
    transition_s: float = Field(ge=0, allow_inf_nan=False)
    max_duration_s: float = Field(gt=0, le=60)
    min_battery_percent: float = Field(gt=0, le=100)

    @model_validator(mode="after")
    def check(self):
        if tuple(self.joint_order) != JOINTS:
            raise ValueError("envelope vectors must use canonical SDK joint order")
        if any(lo >= hi for lo, hi in zip(self.position_min, self.position_max)):
            raise ValueError("joint position bounds must be ordered")
        for field in (
            self.velocity_max,
            self.load_max,
            self.tracking_error_max,
            self.command_rate_max,
        ):
            if any(x <= 0 for x in field):
                raise ValueError("all per-joint safety limits must be positive")
        if not self.max_command_gap_s < self.command_ttl_s <= self.max_policy_age_s:
            raise ValueError("require command gap < command TTL <= maximum policy age")
        if self.max_command_ack_s >= self.command_ttl_s:
            raise ValueError("command acknowledgement budget must be less than command TTL")
        if self.transition_s >= self.max_duration_s:
            raise ValueError("transition must finish within the bounded standing trial")
        if (self.sdk_velocity_safety_limit is not None
                and self.sdk_velocity_safety_limit < max(self.velocity_max)):
            raise ValueError("SDK velocity backstop must cover host per-joint speed limits")
        if self.actuator_limit_profile:
            from .actuator_limits import KNEE_INDICES, KNEE_TABLE, MAX_LOADS

            if any(x > limit for x, limit in zip(self.load_max, MAX_LOADS)):
                raise ValueError("load envelope exceeds manufacturer maximum")
            if any(self.position_min[i] < KNEE_TABLE[0, 0]
                   or self.position_max[i] > KNEE_TABLE[-1, 0] for i in KNEE_INDICES):
                raise ValueError("knee envelope outside manufacturer torque table")
        return self


class HardwareEstopRequirement(StrictModel):
    profile_sha256: Hash
    status_file: str = Field(pattern=r"^/[^\x00]+$")
    max_status_age_s: float = Field(gt=0, le=0.3)


class RobotConfig(StrictModel):
    schema_version: Literal[1]
    endpoint: str
    expected_serial: str = Field(min_length=1)
    # Reviewed wire contract; live qualification and entitlement are separate gates.
    expected_firmware: str = Field(pattern=r"^(?:5\.0\.1|5\.1\.\d+)$")
    expected_host: str
    wired_interface: str
    rpc_timeout_s: float = Field(gt=0, le=10)
    joint_control_feature: Literal["joint_level_control"]
    estop_authority: Literal["sdk_endpoint", "tablet"] = "sdk_endpoint"
    hardware_estop: HardwareEstopRequirement | None = None

    @field_validator("joint_control_feature", mode="before")
    @classmethod
    def normalize_legacy_feature_name(cls, value):
        # Earlier local profiles used the wrong feature code. Preserve their readability.
        return "joint_level_control" if value == "joint_control" else value

    @model_validator(mode="after")
    def check(self):
        if self.estop_authority == "tablet" and self.hardware_estop is not None:
            raise ValueError("tablet authority cannot also require a local E-stop bridge")
        for value in (self.endpoint, self.expected_host, self.wired_interface):
            if not re.fullmatch(r"[A-Za-z0-9_.-]+", value) or value.startswith("-"):
                raise ValueError(
                    "endpoint, host and interface must be plain names or IPv4 addresses"
                )
        return self


class WatchRobotConfig(RobotConfig):
    """Named read-only profile with the same reviewed firmware wire contract."""


class State(StrictModel):
    robot_time_s: float = Field(ge=0)
    received_monotonic_s: float = Field(ge=0)
    positions: Vector19
    velocities: Vector19
    loads: Vector19
    odom_quaternion_wxyz: Annotated[list[float], Field(min_length=4, max_length=4)]
    linear_velocity_odom: Annotated[list[float], Field(min_length=3, max_length=3)]
    angular_velocity_odom: Annotated[list[float], Field(min_length=3, max_length=3)]
    last_command_key: int = Field(ge=0)
    last_command_received_robot_s: float = Field(ge=0)

    @model_validator(mode="after")
    def orientation(self):
        norm = math.sqrt(sum(x * x for x in self.odom_quaternion_wxyz))
        if abs(norm - 1) > 1e-3:
            raise ValueError("invalid unit quaternion")
        return self


def permutation(actual, expected):
    if len(actual) != len(expected) or set(actual) != set(expected):
        raise ValueError("joint names must be a complete unique permutation")


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def load(path: Path, model):
    return model.model_validate_json(path.read_text())


def artifact_path(manifest_path: Path, relative: str) -> Path:
    base = manifest_path.resolve().parent
    result = (base / relative).resolve()
    if Path(relative).is_absolute() or not result.is_relative_to(base):
        raise ContractError("artifact paths must remain inside the manifest directory")
    return result


def check_artifacts(path: Path, manifest: Manifest) -> Path:
    for relative, expected in (
        (manifest.policy_file, manifest.policy_sha256),
        (manifest.training_config_file, manifest.training_config_sha256),
    ):
        if sha256(artifact_path(path, relative)) != expected:
            raise ContractError("artifact hash mismatch")
    return artifact_path(path, manifest.policy_file)


def canonical_hash(value: dict) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()
