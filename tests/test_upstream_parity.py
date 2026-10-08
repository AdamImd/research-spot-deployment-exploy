"""Offline regression vectors for the pinned reference's policy/SDK boundary.

The separate tools/verify_upstream.py runs the actual reference modules and ONNX.
These tests require neither the external source, the model, nor a network.
"""

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.contracts import ContractError, LEGS, Manifest
from spot_deploy.policy import ObservationBuilder
from spot_deploy.safety import Command
from spot_deploy.sdk_control import command_proto
from spot_deploy.sdk_read import decode_state


SDK_TO_POLICY = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]
POLICY_TO_SDK = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]


def test_reference_joint_decode_observation_and_wire(upstream_manifest, envelope):
    from bosdyn.api.header_pb2 import CommonError
    from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse

    m = upstream_manifest
    proto = RobotStateStreamResponse()
    proto.header.error.code = CommonError.CODE_OK
    proto.joint_states.position.extend(np.arange(19) / 16)
    proto.joint_states.velocity.extend(-np.arange(19) / 8)
    proto.joint_states.load.extend(np.arange(19) / 4)
    proto.kinematic_state.odom_tform_body.rotation.w = 1
    state = decode_state(proto, received=100)
    np.testing.assert_array_equal(state.loads, np.arange(19) / 4)
    b = ObservationBuilder(m)
    obs = b.build(state)[0]
    assert obs.shape == (45,)
    np.testing.assert_allclose(
        obs[9:21], np.array(SDK_TO_POLICY) / 16 - m.actions.default_positions, atol=1e-7
    )
    np.testing.assert_array_equal(obs[21:33], -np.array(SDK_TO_POLICY) / 8)
    np.testing.assert_array_equal(obs[33:45], 0)
    # Distinct, signed, beyond-unit actions reveal reorder, clipping and sign errors.
    raw = (np.arange(12) - 6) / 2
    targets = b.targets(raw)
    defaults_sdk = np.array([0.1, 0.9, -1.5, -0.1, 0.9, -1.5, 0.1, 1.1, -1.5, -0.1, 1.1, -1.5])
    np.testing.assert_allclose(targets[:12], defaults_sdk + 0.2 * raw[POLICY_TO_SDK])
    np.testing.assert_array_equal(b.build(state)[0, 33:45], raw)
    wire = command_proto(Command(tuple(targets), 1000.08, 5), m, envelope).joint_command
    np.testing.assert_allclose(wire.position[:12], targets[:12], atol=1e-7)
    np.testing.assert_allclose(wire.position[12:], m.arm_stowed_positions, atol=1e-7)
    np.testing.assert_array_equal(wire.gains.k_q_p[:12], [60] * 12)
    np.testing.assert_array_equal(wire.gains.k_qd_p[:12], [1.5] * 12)
    np.testing.assert_array_equal(wire.velocity, [0] * 19)
    np.testing.assert_array_equal(wire.load, [0] * 19)


@pytest.mark.parametrize("policy_index,sdk_index", enumerate(SDK_TO_POLICY))
def test_each_action_drives_only_its_named_sdk_joint(upstream_manifest, policy_index, sdk_index):
    b = ObservationBuilder(upstream_manifest)
    base = b.targets(np.zeros(12))
    action = np.zeros(12)
    action[policy_index] = 2.5
    delta = b.targets(action) - base
    expected = np.zeros(19)
    expected[sdk_index] = 0.5
    np.testing.assert_allclose(delta, expected)
    assert upstream_manifest.actions.joint_order[policy_index] == LEGS[sdk_index]


def test_previous_action_stays_in_output_order_when_observation_order_differs(
    upstream_manifest, state
):
    m = upstream_manifest.model_copy(deep=True)
    m.observations.joint_order = list(LEGS)
    b = ObservationBuilder(m)
    raw = np.arange(12, dtype=np.float32) / 2
    b.targets(raw)
    np.testing.assert_array_equal(b.build(state)[0, 33:45], raw)
    b.reset()
    np.testing.assert_array_equal(b.build(state)[0, 33:45], 0)


def test_no_clipping_is_explicit_and_preserves_large_observations(upstream_manifest, state):
    state.velocities = [256.0 + i for i in range(19)]
    obs = ObservationBuilder(upstream_manifest).build(state)
    np.testing.assert_array_equal(obs[0, 21:33], 256 + np.array(SDK_TO_POLICY))
    for section, key in (
        ("observations", "clip"),
        ("actions", "clip_min"),
        ("actions", "clip_max"),
    ):
        value = upstream_manifest.model_dump()
        del value[section][key]
        with pytest.raises(ValidationError):
            Manifest.model_validate(value)
    value = upstream_manifest.model_dump()
    value["actions"]["clip_max"] = [1] * 12
    with pytest.raises(ValidationError, match="both bounds"):
        Manifest.model_validate(value)


def test_float32_overflow_is_rejected_before_onnx(upstream_manifest, state):
    state.linear_velocity_odom = [1e100, 0, 0]
    with pytest.raises(ContractError, match="float32"):
        ObservationBuilder(upstream_manifest).build(state)


def test_nonfinite_transform_not_hidden_by_clipping(manifest, state):
    manifest.observations.terms[0].scale = [1e308] * 3
    state.linear_velocity_odom = [1e308, 0, 0]
    with np.errstate(over="ignore", invalid="ignore"):
        with pytest.raises(ContractError, match="non-finite observation"):
            ObservationBuilder(manifest).build(state)


@pytest.mark.parametrize("quat_scale", [1, -1, 1.0003])
def test_all_frame_vectors_share_inverse_quaternion(upstream_manifest, state, quat_scale):
    # q = (.5,.5,.5,.5) cycles body x->odom y, y->z, z->x.
    state.odom_quaternion_wxyz = [quat_scale * 0.5] * 4
    state.linear_velocity_odom = [1, 2, 3]
    state.angular_velocity_odom = [-4, 5, -6]
    obs = ObservationBuilder(upstream_manifest).build(state)[0]
    np.testing.assert_allclose(obs[:9], [2, 3, 1, 5, -6, -4, 0, -1, 0], atol=1e-7)
