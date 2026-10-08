import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.contracts import ContractError, Envelope
from spot_deploy.relic_contract import ReLICManifest
from spot_deploy.relic_control import stand
from spot_deploy.relic_policy import ACTION_IDS, ACTION_JOINTS, CHECKPOINT_SHA256, DEFAULT_Q
from spot_deploy.relic_rollout import ReLICRollout
from test_control_loop import FakeControl, FakeReader, Log

BASE = Path(__file__).parents[1] / "fixtures/relic-simulation"


@pytest.fixture
def relic_manifest(manifest):
    d = manifest.model_dump()
    settings = json.loads((BASE / "settings.json").read_text())
    settings["preparation"].update(
        model_file="support.urdf", model_sha256="0" * 64, duration_s=0.04
    )
    d.update(
        schema_version=2,
        adapter="relic84",
        policy_sha256=CHECKPOINT_SHA256,
        observations={"contract": "relic84-v1"},
        relic=settings,
        arm_stowed_positions=DEFAULT_Q[12:].astype(float).tolist(),
        gains=dict(
            kp=[60.0] * 12 + [120.0, 120.0, 120.0, 100.0, 100.0, 100.0, 16.0],
            kd=[1.5] * 12 + [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 0.32],
            feedforward=[0.0] * 19,
        ),
        actions=dict(
            joint_order=list(ACTION_JOINTS),
            scale=[0.2] * 12,
            default_positions=DEFAULT_Q[ACTION_IDS].tolist(),
            clip_min=None,
            clip_max=None,
            previous_action="raw",
        ),
    )
    return ReLICManifest.model_validate(d)


@pytest.fixture
def relic_envelope():
    d = json.loads((BASE / "envelope.json").read_text())
    d.update(
        transition_s=0.04,
        max_command_gap_s=0.04,
        max_command_ack_s=0.06,
        command_ttl_s=0.08,
        max_policy_age_s=0.1,
        max_state_age_s=0.1,
    )
    return Envelope.model_validate(d)


class FakeReLIC:
    def __init__(self, manifest, mode=None):
        self.manifest, self.mode, self.calls = manifest, mode, []
        self.support = SimpleNamespace(compute=lambda state: (np.ones(19), {}))
        self.body_height = 0.52

    def reset(self):
        self.calls.clear()

    def initialize_height(self, height):
        assert height["foot_contacts"] == [1] * 4
        self.body_height = height["height_m"]

    def evaluate(self, state, previous):
        self.calls.append(previous.copy())
        if self.mode == "stall":
            time.sleep(0.3)
        action = np.ones(12, dtype=np.float32) * len(self.calls) * 0.01
        target = DEFAULT_Q.astype(float).copy()
        target[ACTION_IDS] += 0.2 * action
        if self.mode == "nan":
            target[0] = np.nan
        obs = np.zeros((1, 84))
        obs[0, -12:] = previous
        return target, 0.001, obs, action


def sample(state, t, key=0):
    return state.model_copy(
        update=dict(
            positions=DEFAULT_Q.astype(float).tolist(),
            robot_time_s=1000 + t,
            received_monotonic_s=10 + t,
            last_command_key=key,
        )
    )


def core_fixture(manifest, envelope, state):
    policy = FakeReLIC(manifest)
    core = ReLICRollout(policy, envelope, Log())
    first = sample(state, 0, 19)
    core.initialize(first, {"height_m": 0.52, "foot_contacts": [1] * 4}, 10, 1000)
    command = core.command(first, 10, 1000)
    return core, policy, command


def test_history_waits_for_activation_and_ack(relic_manifest, relic_envelope, state):
    core, policy, command = core_fixture(relic_manifest, relic_envelope, state)
    assert command.key == 20 and not policy.calls
    core.activate(10.005)
    for i in range(1, 10):
        t = 0.005 * i
        command = core.command(sample(state, t, command.key), 10 + t, 1000 + t)
    assert len(policy.calls) == 1
    np.testing.assert_array_equal(policy.calls[0], 0)
    assert command.feedforward == (0.0,) * 19
    first_policy_key = command.key
    # Lack of policy receipt acknowledgement does not advance inference/history.
    for i in range(10, 15):
        t = 0.005 * i
        command = core.command(sample(state, t, first_policy_key - 1), 10 + t, 1000 + t)
    assert len(policy.calls) == 1
    t = 0.075
    core.command(sample(state, t, command.key), 10 + t, 1000 + t)
    assert len(policy.calls) == 2
    np.testing.assert_allclose(policy.calls[1], 0.01)


@pytest.mark.parametrize("fault", ["pose", "velocity", "torque", "unknown_ack", "stale"])
def test_core_handover_and_state_gates(relic_manifest, relic_envelope, state, fault):
    core, _, command = core_fixture(relic_manifest, relic_envelope, state)
    core.activate(10)
    for i in range(1, 8):
        t = 0.005 * i
        command = core.command(sample(state, t, command.key), 10 + t, 1000 + t)
    s = sample(state, 0.04, command.key)
    if fault == "pose":
        s.positions[0] += 0.1
    if fault == "velocity":
        s.velocities[0] = 0.6
    if fault == "torque":
        core.spec.handover_torque_step_max = [0.001] * 19
    if fault == "unknown_ack":
        s.last_command_key += 100
    if fault == "stale":
        s.received_monotonic_s -= 1
    with pytest.raises(ContractError):
        core.command(s, 10.04, 1000.04)


@pytest.mark.parametrize("mutation", ["gain", "scale", "clip", "checkpoint", "joint_order"])
def test_changed_release_contract_rejected(relic_manifest, mutation):
    d = relic_manifest.model_dump()
    if mutation == "gain":
        d["gains"]["kp"][0] = 59
    if mutation == "scale":
        d["actions"]["scale"][0] = 0.3
    if mutation == "clip":
        d["actions"].update(clip_min=[-1.0] * 12, clip_max=[1.0] * 12)
    if mutation == "checkpoint":
        d["policy_sha256"] = "0" * 64
    if mutation == "joint_order":
        d["actions"]["joint_order"] = list(reversed(ACTION_JOINTS))
    with pytest.raises(ValidationError):
        ReLICManifest.model_validate(d)


@pytest.mark.parametrize("mode", [None, "nan", "stall", "lease", "estop", "disconnect", "ack"])
def test_live_orchestration_faults_close_without_resuming(
    relic_manifest, relic_envelope, state, mode
):
    state.positions = DEFAULT_Q.astype(float).tolist()
    reader = FakeReader(state)
    reader.read_body_height = lambda: {"height_m": 0.52, "foot_contacts": [1] * 4}
    control = FakeControl(reader, mode)
    policy = FakeReLIC(relic_manifest, mode)
    log = Log()
    if mode is None:
        result = stand(reader, control, policy, relic_envelope, 0.15, log, lambda c, *a: c)
        assert result["shutdown_confirmed"] and result["policy_samples"] >= 2
        assert any(any(x != 0 for x in c.feedforward) for c in control.commands)
        np.testing.assert_array_equal(policy.calls[0], 0)
    else:
        with pytest.raises(ContractError):
            stand(reader, control, policy, relic_envelope, 0.5, log, lambda c, *a: c)
    assert control.calls[-1] == "close"
    assert not control.worker or not control.worker.is_alive()
    before = len(control.commands)
    time.sleep(0.01)
    assert len(control.commands) == before


def test_simulation_envelope_cannot_promote(relic_manifest, relic_envelope):
    from spot_deploy.readiness import require_live

    relic_manifest.purpose = "candidate"
    with pytest.raises(ContractError, match="simulation envelopes"):
        require_live(
            relic_manifest,
            relic_envelope,
            None,
            None,
            None,
            None,
            None,
            "Adam",
            "Other operator",
            True,
            0.5,
        )


def test_direct_start_has_no_interpolation_and_keeps_ack_history(relic_manifest, relic_envelope, state):
    from spot_deploy.relic_contract import Preparation
    relic_manifest.relic.preparation = Preparation.model_validate(
        relic_manifest.relic.preparation.model_dump() | {"kind": "direct", "duration_s": 0})
    relic_envelope.transition_s = 0
    policy = FakeReLIC(relic_manifest)
    core = ReLICRollout(policy, relic_envelope, Log())
    first = sample(state, 0, 7)
    # A captured stance need not equal the nominal training reset pose.
    first.positions[0] += .05
    core.initialize(first, {"height_m": .52, "foot_contacts": [1]*4}, 10, 1000)
    held = core.command(first, 10, 1000)
    np.testing.assert_array_equal(held.positions, first.positions)
    assert not policy.calls and np.isfinite(held.feedforward).all()
    core.activate(10.005)
    first.robot_time_s += .005
    first.received_monotonic_s += .005
    first.last_command_key = held.key
    command = core.command(first, 10.005, 1000.005)
    assert core.phase == "policy" and len(policy.calls) == 1
    np.testing.assert_array_equal(policy.calls[0], 0)
    assert command.feedforward == (0.,)*19
    first.robot_time_s += .02
    first.received_monotonic_s += .02
    first.last_command_key = command.key
    core.command(first, 10.025, 1000.025)
    np.testing.assert_allclose(policy.calls[1], .01)


@pytest.mark.parametrize("kind,duration", [("direct", .04), ("four-foot-static-support", 0)])
def test_startup_kind_and_duration_must_match(relic_manifest, kind, duration):
    d = relic_manifest.model_dump()
    d["relic"]["preparation"].update(kind=kind, duration_s=duration)
    with pytest.raises(ValidationError, match="direct startup"):
        ReLICManifest.model_validate(d)


def test_generic_controller_rejects_zero_transition(manifest, envelope):
    from spot_deploy.safety import Guard
    envelope.transition_s = 0
    with pytest.raises(ContractError, match="zero transition"):
        Guard(manifest, envelope)
