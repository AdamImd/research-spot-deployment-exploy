"""Compare the pinned reference's pure pipeline and ONNX on synthetic SDK messages.

Never imports the reference robot/demo adapters or connects to a robot. Downloads
are opt-in, hash checked, and confined to ignored local/reference. The bundled
handstand policy is used only for numerical comparison, never as a candidate.
"""

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType
from urllib.request import urlopen

import numpy as np

from spot_deploy.contracts import ContractError, LEGS, Manifest, load, sha256
from spot_deploy.policy import ObservationBuilder, OnnxPolicy
from spot_deploy.records import RunRecord, atomic_json, root
from spot_deploy.safety import Command
from spot_deploy.sdk_control import command_proto
from spot_deploy.sdk_read import decode_state

SPEC = root() / "fixtures" / "upstream-reference.json"


def reference_files(spec, fetch):
    paths, directories = [], []
    for source in spec["sources"]:
        repository, commit = source["repository"], source["commit"]
        directory = root() / "local" / "reference" / repository.split("/")[-1] / commit
        directories.append(directory)
        for name, digest in source["files"].items():
            path = directory / name
            if not path.is_file() and fetch:
                url = f"https://raw.githubusercontent.com/{repository}/{commit}/{name}"
                with urlopen(url, timeout=20) as response:
                    data = response.read(2_000_001)
                if len(data) > 2_000_000 or hashlib.sha256(data).hexdigest() != digest:
                    raise ContractError(f"reference download hash/size mismatch: {name}")
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            if not path.is_file() or sha256(path) != digest:
                raise ContractError(f"reference missing/modified: {name}; use --fetch if missing")
            paths.append(path)
    return directories[0], paths


def import_reference(directory):
    # Explicit allowlist: do not execute upstream __init__, robot or demo files.
    # Every file below has already been checked against the versioned SHA-256 list.
    for name in ("orbit", "spot", "utils"):
        if name in sys.modules:
            raise ContractError("run this audit in a fresh Python process")
        package = ModuleType(name)
        package.__path__ = []
        sys.modules[name] = package
    modules = {}
    for name in (
        "orbit.orbit_constants",
        "spot.constants",
        "utils.dict_tools",
        "orbit.orbit_configuration",
        "orbit.observations",
        "orbit.onnx_command_generator",
    ):
        path = directory / "spot_rl_runtime" / (name.replace(".", "/") + ".py")
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        modules[name] = module
    return modules


def message_for(index, quaternion, defaults, rng):
    from bosdyn.api.header_pb2 import CommonError
    from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
    from bosdyn.util import seconds_to_timestamp

    message = RobotStateStreamResponse()
    message.header.error.code = CommonError.CODE_OK
    q = np.concatenate((defaults + rng.uniform(-0.2, 0.2, 12), np.arange(7) / 8))
    message.joint_states.position.extend(q)
    message.joint_states.velocity.extend(rng.uniform(-2, 2, 19))
    message.joint_states.load.extend(np.arange(19) / 4)
    message.joint_states.acquisition_timestamp.CopyFrom(seconds_to_timestamp(1000 + 0.02 * index))
    rotation = message.kinematic_state.odom_tform_body.rotation
    rotation.w, rotation.x, rotation.y, rotation.z = quaternion
    linear = message.kinematic_state.velocity_of_body_in_odom.linear
    angular = message.kinematic_state.velocity_of_body_in_odom.angular
    linear.x, linear.y, linear.z = rng.uniform(-1, 1, 3)
    angular.x, angular.y, angular.z = rng.uniform(-2, 2, 3)
    return message


def verify(spec, directory, record):
    modules = import_reference(directory)
    config = modules["orbit.orbit_configuration"].load_configuration(
        directory / "models/env_cfg.json"
    )
    network = modules["orbit.onnx_command_generator"]
    model_path = directory / "models/policy.onnx"
    base = load(root() / "fixtures" / "standing" / "manifest.json", Manifest).model_dump()
    base.update(spec["manifest_patch"])
    base.update(
        policy_sha256=sha256(model_path),
        training_config_sha256=sha256(directory / "models/env_cfg.json"),
    )
    manifest = Manifest.model_validate(base)
    # All zero provenance hashes and purpose=fixture are retained deliberately.
    # The original training commit and physical arm configuration are unknown.
    atomic_json(record.directory / "audit-contract.json", manifest.model_dump())
    orbit = modules["orbit.orbit_constants"].ordered_joint_names_orbit
    sdk = modules["spot.constants"].ordered_joint_names_bosdyn
    assert orbit == manifest.actions.joint_order == manifest.observations.joint_order
    assert tuple(sdk) == LEGS
    assert [sdk.index(joint) for joint in orbit] == spec["sdk_to_policy_indices"]
    assert [orbit.index(joint) for joint in sdk] == spec["policy_to_sdk_indices"]
    np.testing.assert_array_equal(
        [config.default_joints[x] for x in orbit], manifest.actions.default_positions
    )
    np.testing.assert_array_equal([config.kp[x] for x in sdk], manifest.gains.kp[:12])
    np.testing.assert_array_equal([config.kd[x] for x in sdk], manifest.gains.kd[:12])
    np.testing.assert_array_equal([config.action_scale] * 12, manifest.actions.scale)
    env = json.loads((directory / "models/env_cfg.json").read_text())
    assert 1 / (env["sim"]["dt"] * env["decimation"]) == manifest.policy_hz
    assert env["actions"]["joint_pos"]["clip"] is None
    assert env["scene"]["robot"]["init_state"]["joint_vel"] == {".*": 0.0}
    assert env["commands"] is None
    assert not env["observations"]["policy"]["enable_corruption"]
    for key in (
        "base_lin_vel",
        "base_ang_vel",
        "projected_gravity",
        "joint_pos",
        "joint_vel",
        "actions",
    ):
        term = env["observations"]["policy"][key]
        assert term["scale"] is term["clip"] is None
        assert term["history_length"] == 0

    maxima = {}

    def compare(label, actual, expected):
        a, b = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
        np.testing.assert_allclose(a, b, atol=spec["atol"], rtol=spec["rtol"], err_msg=label)
        maxima[label] = max(maxima.get(label, 0), float(np.max(np.abs(a - b))))

    context = network.OnnxControllerContext()
    reference = network.OnnxCommandGenerator(context, config, str(model_path), False)
    reference_session = reference._inference_session
    ours = OnnxPolicy(model_path, manifest)
    defaults = np.array([config.default_joints[x] for x in sdk])
    rng = np.random.default_rng(spec["seed"])
    quaternions = [[1, 0, 0, 0], [0.5] * 4, [-0.5] * 4, [2**-0.5, 0, 0, 2**-0.5], [0.50015] * 4]
    for _ in range(19):
        q = rng.normal(size=4)
        quaternions.append((q / np.linalg.norm(q)).tolist())
    messages = [message_for(i, q, defaults, rng) for i, q in enumerate(quaternions)]

    # Both signs, every action channel, beyond +/-1: one-hot checks cannot hide
    # a permutation behind identical gains, symmetric poses or a zero policy.
    builder = ObservationBuilder(manifest)
    for i in range(24):
        raw = np.zeros(12, dtype=np.float32)
        raw[i % 12] = 2.5 if i < 12 else -2.5

        class FixedOutput:
            def run(self, *args):
                return [raw[None, :]]

        context.latest_state = messages[i]
        reference._inference_session = FixedOutput()
        reference._count = 10  # Compare the full-scale map, outside the startup ramp.
        target = builder.targets(raw)
        command = reference().joint_command
        compare("basis_targets", target[:12], command.position)
        compare("basis_previous_action", builder.previous_action, reference._last_action)
        compare("arm_tail", target[12:], manifest.arm_stowed_positions)
        record.event(
            "basis",
            action=raw.tolist(),
            sdk_targets=target.tolist(),
            reference_targets=list(command.position),
        )

    reference._inference_session = reference_session
    reference._last_action = [0] * 12
    reference._count = 1
    ours.reset()
    max_abs_action = 0
    from spot_deploy.contracts import Envelope

    envelope = load(root() / "fixtures" / "standing" / "envelope.json", Envelope)
    for step, message in enumerate(messages, 1):
        context.latest_state = message
        state = decode_state(message, received=100 + step * 0.02)
        ref_obs = np.asarray(reference.collect_inputs(message, config), dtype=np.float32)
        target, _, obs, action = ours.predict(state)
        command = reference().joint_command
        compare("onnx_observations", obs[0], ref_obs)
        compare("onnx_raw_actions", action, reference._last_action)
        ref_full = (
            defaults
            + config.action_scale * np.array(reference._last_action)[spec["policy_to_sdk_indices"]]
        )
        compare("onnx_full_scale_targets", target[:12], ref_full)
        ramp = min(0.1 * step, 1)
        compare(
            "reference_ramp_formula", command.position, defaults + ramp * (target[:12] - defaults)
        )
        wire = command_proto(
            Command(tuple(target), state.robot_time_s + 0.08, step), manifest, envelope
        ).joint_command
        compare("sdk_wire_targets", wire.position, target)
        if step >= 10:
            compare("steady_wire_targets", wire.position[:12], command.position)
        assert len(command.position) == 12 and len(wire.position) == 19
        compare("wire_velocity", command.velocity, wire.velocity[:12])
        compare("wire_feedforward", command.load, wire.load[:12])
        if step == 1:
            compare("wire_kp", command.gains.k_q_p, wire.gains.k_q_p[:12])
            compare("wire_kd", command.gains.k_qd_p, wire.gains.k_qd_p[:12])
        else:
            assert len(command.gains.k_q_p) == len(command.gains.k_qd_p) == 0
        assert len(wire.gains.k_q_p) == len(wire.gains.k_qd_p) == 19
        assert command.extrapolation_duration.nanos == 5_000_000
        assert wire.extrapolation_duration.nanos == 0
        assert command.user_command_key == wire.user_command_key == step
        ref_expiry = command.end_time.seconds + command.end_time.nanos / 1e9
        assert abs(ref_expiry - state.robot_time_s - 0.1) < 1e-7
        max_abs_action = max(max_abs_action, float(np.max(np.abs(action))))
        record.event(
            "onnx_parity",
            step=step,
            state=state.model_dump(),
            observations=obs[0].tolist(),
            reference_observations=ref_obs.tolist(),
            raw_actions=action.tolist(),
            reference_raw_actions=reference._last_action,
            targets=target.tolist(),
            reference_command=list(command.position),
            reference_ramp=ramp,
        )
    ours.reset()
    compare(
        "reset_previous_action", ours.builder.build(decode_state(messages[0]))[0, 33:45], [0] * 12
    )
    return dict(
        passed=True,
        basis_cases=24,
        sequential_onnx_cases=len(messages),
        max_absolute_errors=maxima,
        max_absolute_raw_action=max_abs_action,
        tolerance={"atol": spec["atol"], "rtol": spec["rtol"]},
        runtime_commit=spec["sources"][0]["commit"],
        model_sha256=sha256(model_path),
        source_commit=record.source["commit"],
        source_dirty=record.source["dirty"],
        policy_hz=manifest.policy_hz,
        reference_nominal_policy_hz=333 / 6,
        sdk_to_policy_indices=spec["sdk_to_policy_indices"],
        policy_to_sdk_indices=spec["policy_to_sdk_indices"],
        spatialmath_version=importlib.metadata.version("spatialmath-python"),
        limitations=[
            "Synthetic inputs; no plant simulation, hardware or standing qualification",
            "Reference is an armless handstand model; its original training/export commit is unknown",
            "Our measured-pose/time blend differs from the reference's ten-call action-amplitude ramp",
            "Our seven fixed arm targets/gains here are synthetic sentinels",
            "Training tree includes a 60-to-40 gain curriculum; bundled runtime config uses 60",
        ],
        hardware_access=False,
        hardware_readiness="unverified",
        live_rl="blocked",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fetch", action="store_true", help="download missing hash-pinned references"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    spec = json.loads(SPEC.read_text())
    directory, paths = reference_files(spec, args.fetch)
    record = RunRecord(args.output, "upstream-numerical-parity", [SPEC, *paths])
    try:
        result = verify(spec, directory, record)
        record.finish("passed", result)
        print(json.dumps(result, sort_keys=True))
    except BaseException as exc:
        record.finish("failed", {"passed": False, "error_type": type(exc).__name__})
        raise


if __name__ == "__main__":
    main()
