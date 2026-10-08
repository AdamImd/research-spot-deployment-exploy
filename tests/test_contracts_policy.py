import json

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.contracts import (
    ContractError,
    Envelope,
    JOINTS,
    LEGS,
    Manifest,
    RobotConfig,
    State,
    artifact_path,
    check_artifacts,
)
from spot_deploy.policy import ObservationBuilder, OnnxPolicy


@pytest.mark.parametrize(
    "mutation",
    [
        lambda m: m.update(task="handstand"),
        lambda m: m.update(morphology="spot-base"),
        lambda m: m.update(policy_hz=float("nan")),
        lambda m: m.update(stream_hz=50),
        lambda m: m.update(password="DO_NOT_PRINT"),
        lambda m: m["actions"].update(joint_order=["fl_hx"] * 12),
        lambda m: m["observations"].update(joint_order=list(LEGS)[:-1]),
        lambda m: m["observations"].update(std=[0] * 48),
        lambda m: m["observations"]["terms"].pop(0),
        lambda m: m["gains"].update(kp=[0] * 19),
        lambda m: m.update(arm_stowed_positions=[0] * 6),
        lambda m: m.update(training_commit="unrecorded"),
    ],
)
def test_manifest_rejects_incompatible_contract(manifest, mutation):
    value = manifest.model_dump()
    mutation(value)
    with pytest.raises(ValidationError):
        Manifest.model_validate(value)


@pytest.mark.parametrize(
    "field", ["max_state_age_s", "load_max", "tracking_error_max", "command_ttl_s"]
)
def test_no_implicit_envelope_limits(envelope, field):
    value = envelope.model_dump()
    del value[field]
    with pytest.raises(ValidationError):
        Envelope.model_validate(value)


def test_envelope_canonical_order(envelope):
    value = envelope.model_dump()
    value["joint_order"] = list(reversed(JOINTS))
    with pytest.raises(ValidationError):
        Envelope.model_validate(value)


def test_state_nan_and_bad_quaternion(state):
    for updates in (
        {"positions": [float("inf")] * 19},
        {"odom_quaternion_wxyz": [0] * 4},
        {"positions": [0] * 12},
    ):
        with pytest.raises(ValidationError):
            State.model_validate(state.model_dump() | updates)


def test_joint_permutations_and_offsets(manifest, state):
    value = manifest.model_dump()
    value["observations"]["joint_order"] = list(reversed(LEGS))
    value["actions"]["joint_order"] = list(reversed(LEGS))
    value["actions"]["default_positions"] = list(np.arange(12) * 0.01)
    value["actions"]["scale"] = [1] * 12
    m = Manifest.model_validate(value)
    builder = ObservationBuilder(m)
    state.positions = list(range(19))
    obs = builder.build(state)[0]
    np.testing.assert_allclose(obs[9:21], np.arange(11, -1, -1) - np.arange(12) * 0.01, atol=1e-6)
    output = np.linspace(-0.5, 0.5, 12)
    targets = builder.targets(output)
    np.testing.assert_allclose(targets[:12], (np.arange(12) * 0.01 + output)[::-1])
    np.testing.assert_array_equal(targets[12:], [0] * 7)


def test_body_frames_yaw_and_tilt(manifest, state):
    builder = ObservationBuilder(manifest)
    state.odom_quaternion_wxyz = [2**-0.5, 0, 0, 2**-0.5]
    state.linear_velocity_odom = [1, 0, 0]
    obs = builder.build(state)[0]
    np.testing.assert_allclose(obs[:3], [0, -1, 0], atol=1e-6)
    np.testing.assert_allclose(obs[6:9], [0, 0, -1], atol=1e-6)
    state.odom_quaternion_wxyz = [2**-0.5, 2**-0.5, 0, 0]
    np.testing.assert_allclose(builder.build(state)[0, 6:9], [0, -1, 0], atol=1e-6)


@pytest.mark.parametrize("initial", ["zeros", "repeat_first"])
def test_history_normalization_reset(manifest, state, initial):
    value = manifest.model_dump()
    value["observations"].update(
        history_length=3, history_initialization=initial, mean=[0.5] * 48, std=[2] * 48
    )
    m = Manifest.model_validate(value)
    b = ObservationBuilder(m)
    obs = b.build(state)[0].reshape(3, 48)
    np.testing.assert_allclose(obs[2, :6], [-0.25] * 6)
    np.testing.assert_allclose(obs[0], obs[2] if initial == "repeat_first" else 0)
    b.targets(np.ones(12))
    assert b.build(state)[0, -15] == pytest.approx(0.25)
    b.reset()
    assert b.build(state)[0, -15] == pytest.approx(-0.25)


def test_action_clipping_previous_semantics(manifest, state):
    b = ObservationBuilder(manifest)
    np.testing.assert_allclose(b.targets([5] * 12)[:12], 0.1)
    np.testing.assert_allclose(b.build(state)[0, 33:45], 5)
    for bad in ([0] * 19, [float("nan")] * 12):
        with pytest.raises(ContractError):
            b.targets(bad)


def test_real_onnx_fixture_and_golden(manifest, state, fixture_path):
    path = check_artifacts(fixture_path / "manifest.json", manifest)
    model = OnnxPolicy(path, manifest)
    targets, duration, obs, action = model.predict(state)
    golden = json.loads((fixture_path / "expected.json").read_text())
    np.testing.assert_allclose(obs[0], golden["observation"])
    np.testing.assert_allclose(action, golden["actions"])
    np.testing.assert_allclose(targets, golden["targets"])
    assert duration > 0


def test_model_shape_names_and_hash_mismatch(manifest, fixture_path):
    bad = manifest.model_copy(deep=True)
    bad.policy_sha256 = "1" * 64
    with pytest.raises(ContractError, match="hash"):
        check_artifacts(fixture_path / "manifest.json", bad)
    bad = manifest.model_copy(deep=True)
    bad.input_name = "wrong"
    with pytest.raises(ContractError, match="ONNX"):
        OnnxPolicy(fixture_path / "fixture.onnx", bad)


def test_artifact_traversal(fixture_path):
    for path in ("../../../secret", "/etc/passwd"):
        with pytest.raises(ContractError):
            artifact_path(fixture_path / "manifest.json", path)


def test_checked_in_schemas_match_contracts(fixture_path):
    from spot_deploy.readiness import EvidenceIndex

    for model in (Manifest, Envelope, RobotConfig, State, EvidenceIndex):
        expected = json.loads(
            (fixture_path.parents[1] / "schemas" / f"{model.__name__}.json").read_text()
        )
        assert expected == model.model_json_schema()


@pytest.mark.parametrize(
    "endpoint", ["https://robot", "robot;echo", "--version", "user:secret@robot"]
)
def test_no_endpoint_commands_or_credentials(endpoint):
    with pytest.raises(ValidationError):
        RobotConfig.model_validate(
            {
                "schema_version": 1,
                "endpoint": endpoint,
                "expected_serial": "test",
                "expected_firmware": "5.1.1",
                "expected_host": "rpm",
                "wired_interface": "eth0",
                "rpc_timeout_s": 1,
                "joint_control_feature": "joint_control",
            }
        )
