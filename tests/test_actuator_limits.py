import ast
import json
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.actuator_limits import KNEE_INDICES, KNEE_TABLE, MAX_LOADS, check_torque, torque_limits
from spot_deploy.contracts import ContractError, Envelope
from spot_deploy.safety import Command, Guard
from spot_deploy.sdk_control import command_proto

ROOT = Path(__file__).resolve().parents[1]


def pose(knee=-1.443626):
    q = np.zeros(19)
    q[list(KNEE_INDICES)] = knee
    return q


def manufacturer_envelope(envelope):
    data = envelope.model_dump() | dict(
        actuator_limit_profile="spot-sdk-5.0.1", load_max=list(MAX_LOADS),
        position_min=[-4.] * 19, position_max=[4.] * 19, tracking_error_max=[4.] * 19,
        velocity_max=[8.] * 12 + [.5] * 7, sdk_velocity_safety_limit=8.)
    for i in KNEE_INDICES:
        data["position_min"][i] = KNEE_TABLE[0, 0]
        data["position_max"][i] = KNEE_TABLE[-1, 0]
    return Envelope.model_validate(data)


def test_knee_table_matches_bundled_source_and_interpolation():
    source = ROOT / "simulation/source/relic/relic/assets/spot/constants.py"
    node = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.AnnAssign)
                and getattr(n.target, "id", "") == "JOINT_PARAMETER_LOOKUP_TABLE")
    expected = np.array(ast.literal_eval(node.value))[:, [0, 2]]
    np.testing.assert_array_equal(expected, KNEE_TABLE)
    assert torque_limits(pose(-2.7929))[2] == pytest.approx(37.165077)
    assert torque_limits(pose(-1.443626))[2] == pytest.approx(113.236015)
    assert torque_limits(pose(-.2471))[2] == pytest.approx(30.6025)
    midpoint = KNEE_TABLE[:2, 0].mean()
    assert torque_limits(pose(midpoint))[2] == pytest.approx(KNEE_TABLE[:2, 1].mean())


@pytest.mark.parametrize("q", [-2.793, -.247, float("nan")])
def test_knee_table_never_extrapolates(q):
    with pytest.raises(ValueError):
        torque_limits(pose(q))


def test_coupled_arm_checks_motor_space_not_independent_joint_box():
    torque = np.zeros(19)
    torque[13:15] = [160., 80.]
    check_torque(pose(), torque)
    torque[14] = -80.  # All individual joint caps pass; coupled motor needs 240 N m.
    with pytest.raises(ValueError, match="coupled"):
        check_torque(pose(), torque)


def test_measured_and_requested_loads_use_current_knee_pose(manifest, envelope, state):
    env = manufacturer_envelope(envelope)
    q = pose(-2.78)
    loads = np.zeros(19)
    loads[2] = 42.
    sample = state.model_copy(update={"positions": q.tolist(), "loads": loads.tolist()})
    with pytest.raises(ContractError, match="measured load: manufacturer"):
        Guard(manifest, env).check_state(sample, 100., 1000.)
    manifest.gains.kp = [60.] * 19
    sample.loads = [0.] * 19
    target = q.copy()
    target[2] += .7
    with pytest.raises(ContractError, match="predicted.*manufacturer"):
        Guard(manifest, env).command(target, sample, 100., 1000., 100., .001)
    # The same 42 N m is feasible near the strongest knee configuration.
    q = pose()
    sample.positions = q.tolist()
    target = q.copy()
    target[2] += .7
    Guard(manifest, env).command(target, sample, 100., 1000., 100., .001)


def test_feedforward_coupling_and_tighter_application_caps(manifest, envelope, state):
    env = manufacturer_envelope(envelope)
    sample = state.model_copy(update={"positions": pose().tolist()})
    ff = np.zeros(19)
    ff[13:15] = [160., -80.]
    with pytest.raises(ContractError, match="feedforward.*coupled"):
        Guard(manifest, env).command(sample.positions, sample, 100., 1000., 100., .001, ff)
    env.load_max = [20.] * 12 + list(MAX_LOADS[12:])
    sample.loads = [21.] + [0.] * 18
    with pytest.raises(ContractError, match="measured joint velocity or load"):
        Guard(manifest, env).check_state(sample, 100., 1000.)


def test_explicit_sdk_backstop_retains_host_arm_limit(manifest, envelope, state):
    env = manufacturer_envelope(envelope)
    proto = command_proto(Command(tuple(pose()), 1000.05, 1), manifest, env)
    assert proto.joint_command.velocity_safety_limit.value == 8.
    dq = np.zeros(19)
    dq[12] = .6
    sample = state.model_copy(update={"positions": pose().tolist(), "velocities": dq.tolist()})
    with pytest.raises(ContractError, match="velocity"):
        Guard(manifest, env).check_state(sample, 100., 1000.)


@pytest.mark.parametrize("change", [
    {"load_max": [200.] * 19}, {"actuator_limit_profile": "unknown"},
    {"position_min": [-4.] * 19}, {"sdk_velocity_safety_limit": .5},
])
def test_invalid_manufacturer_configuration_is_rejected(envelope, change):
    data = manufacturer_envelope(envelope).model_dump() | change
    with pytest.raises(ValidationError):
        Envelope.model_validate(data)


def test_selected_configuration_uses_manufacturer_profile():
    data = json.loads((ROOT / "configs/spot-manufacturer-standing-envelope.json").read_text())
    env = Envelope.model_validate(data)
    assert env.scope == "hardware"
    assert env.actuator_limit_profile == "spot-sdk-5.0.1"
    assert env.load_max == list(MAX_LOADS)
    assert env.max_duration_s == 10 and env.transition_s == 0
