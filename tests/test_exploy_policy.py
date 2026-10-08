import copy
import json
from types import SimpleNamespace

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.contracts import JOINTS, ContractError
from spot_deploy.exploy_policy import (
    EXPLOY_COMMIT, INPUT_SIZES, SCRIPT_SHA256, TARGET,
    ExployReLICManifest, ExployReLICPolicy, validate_metadata,
)
from spot_deploy.relic_contract import load_manifest
from spot_deploy.relic_policy import ACTION_JOINTS, CHECKPOINT_SHA256, ROOT_COM_B
from test_relic_rollout import relic_manifest, relic_envelope  # noqa: F401


@pytest.fixture
def exploy_manifest(relic_manifest):  # noqa: F811
    data = relic_manifest.model_dump()
    data.update(schema_version=3, adapter="relic-exploy", policy_sha256="1" * 64,
                input_name="named-state-v1", output_name=TARGET,
                exploy=dict(contract="relic-exploy-v1", exporter_commit=EXPLOY_COMMIT,
                    source_policy_sha256=CHECKPOINT_SHA256, source_script_sha256=SCRIPT_SHA256,
                    export_record_file="export.json", export_record_sha256="2" * 64,
                    configuration_file="configuration.json", configuration_sha256="3" * 64))
    return ExployReLICManifest.model_validate(data)


@pytest.fixture
def metadata(exploy_manifest):
    manifest = exploy_manifest
    return dict(contract="relic-exploy-v1", exploy_commit=EXPLOY_COMMIT,
        source_onnx_sha256=CHECKPOINT_SHA256, source_torchscript_sha256=SCRIPT_SHA256,
        joint_names=list(reversed(JOINTS)), action_joint_names=list(ACTION_JOINTS),
        gains={n: dict(stiffness=manifest.gains.kp[i], damping=manifest.gains.kd[i])
               for i, n in enumerate(JOINTS)}, root_com_b=ROOT_COM_B.tolist(),
        policy_hz=50, stream_hz=200, graph="policy-step", mode="four-foot-standing",
        history="external-acknowledged-raw-actions; zero at activation", target_output=TARGET)


def test_distinct_manifest_artifact_and_dispatch(exploy_manifest, tmp_path):
    manifest = exploy_manifest
    path = tmp_path / "manifest.json"
    path.write_text(manifest.model_dump_json())
    assert isinstance(load_manifest(path), ExployReLICManifest)
    data = manifest.model_dump()
    data["policy_sha256"] = CHECKPOINT_SHA256
    with pytest.raises(ValidationError, match="environment-exported graph"):
        ExployReLICManifest.model_validate(data)


@pytest.mark.parametrize("key,value", [
    ("exporter_commit", "0" * 40), ("source_policy_sha256", "0" * 64),
    ("source_script_sha256", "0" * 64),
])
def test_pinned_export_inputs(exploy_manifest, key, value):
    manifest = exploy_manifest
    data = manifest.model_dump()
    data["exploy"][key] = value
    with pytest.raises(ValidationError):
        ExployReLICManifest.model_validate(data)


@pytest.mark.parametrize("key,value", [
    ("joint_names", list(JOINTS[:-1]) + [JOINTS[0]]),
    ("action_joint_names", list(reversed(ACTION_JOINTS))),
    ("root_com_b", [0., 0., 0.]), ("root_com_b", [float("nan"), 0., 0.]),
    ("graph", "decimation"), ("policy_hz", 200), ("mode", "three-leg"),
    ("history", "automatic-feedback"), ("exploy_commit", "0" * 40),
])
def test_reject_incompatible_graph_metadata(exploy_manifest, metadata, key, value):
    manifest = exploy_manifest
    metadata[key] = value
    with pytest.raises(ContractError):
        validate_metadata(metadata, manifest)


def test_accept_named_permutation_and_reject_changed_gains(exploy_manifest, metadata):
    manifest = exploy_manifest
    assert validate_metadata(metadata, manifest) == list(reversed(JOINTS))
    metadata["gains"]["arm0_sh0"]["stiffness"] = 1
    with pytest.raises(ContractError, match="gains"):
        validate_metadata(metadata, manifest)


def policy_without_runtime(manifest):
    policy = object.__new__(ExployReLICPolicy)
    policy.manifest = copy.deepcopy(manifest)
    policy.graph_from_sdk = policy.sdk_from_graph = list(reversed(range(19)))
    policy.reset()
    policy.body_height = .53
    return policy


def test_explicit_state_mapping_and_initialization(exploy_manifest, state):
    manifest = exploy_manifest
    policy = policy_without_runtime(manifest)
    frame = policy.inputs(state, np.zeros(12))
    assert set(frame) == set(INPUT_SIZES)
    for name, value in frame.items():
        assert value.shape == (1, INPUT_SIZES[name]) and value.dtype == np.float32
    np.testing.assert_array_equal(frame["joint_positions"][0],
                                  np.asarray(state.positions, np.float32)[::-1])
    np.testing.assert_array_equal(frame["previous_actions"], np.zeros((1, 12)))
    assert frame["torso_command"][0, 2] == np.float32(.53)
    policy.reset()
    with pytest.raises(ContractError, match="height"):
        policy.inputs(state, np.zeros(12))


def test_evaluate_does_not_advance_history_and_warmup_discards_actions(exploy_manifest, state):
    manifest = exploy_manifest
    policy = policy_without_runtime(manifest)
    calls = []

    def run(names, inputs):
        calls.append(inputs)
        actions = inputs["previous_actions"] + .1
        obs = np.zeros((1, 84), np.float32)
        obs[:, -12:] = inputs["previous_actions"]
        targets = np.zeros((1, 19), np.float32)
        targets[0, :7] = inputs["arm_command"][0, ::-1]
        return [actions] if names == ["actions"] else [actions, obs, targets]

    policy.runner = SimpleNamespace(session=SimpleNamespace(run=run))
    previous = np.arange(12, dtype=np.float32)
    target, _, obs, action = policy.evaluate(state, previous)
    np.testing.assert_array_equal(obs[0, -12:], previous)
    np.testing.assert_array_equal(action, previous + .1)
    np.testing.assert_array_equal(policy.previous_action, np.zeros(12))
    np.testing.assert_array_equal(target[12:], manifest.arm_stowed_positions)
    policy.reset()
    policy.warmup()
    assert policy.body_height is None
    np.testing.assert_array_equal(policy.previous_action, np.zeros(12))
    np.testing.assert_array_equal(calls[-1]["previous_actions"], np.zeros((1, 12)))


def test_reject_nonfinite_history(exploy_manifest, state):
    manifest = exploy_manifest
    policy = policy_without_runtime(manifest)
    with pytest.raises(ContractError):
        policy.inputs(state, np.full(12, np.nan))


def test_schema_tracks_export_record(exploy_manifest):
    manifest = exploy_manifest
    schema = ExployReLICManifest.model_json_schema()
    assert "export_record_sha256" in json.dumps(schema)
    assert manifest.policy_hz == 50 and manifest.stream_hz == 200


def test_exploy_direct_start_guard_requires_matching_preparation(exploy_manifest, relic_envelope):  # noqa: F811
    from spot_deploy.safety import Guard

    envelope = relic_envelope.model_copy(update={"transition_s": 0.})
    with pytest.raises(ContractError, match="direct-start"):
        Guard(exploy_manifest, envelope)
    data = exploy_manifest.model_dump()
    data["relic"]["preparation"].update(kind="direct", duration_s=0.)
    direct = ExployReLICManifest.model_validate(data)
    Guard(direct, envelope)
