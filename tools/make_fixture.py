"""Deterministic synthetic fixture generator. Never suitable for robot operation."""

import hashlib
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

from spot_deploy.contracts import JOINTS, LEGS, TERM_SIZES

BASE = Path(__file__).resolve().parents[1] / "fixtures" / "standing"


def write(name, value):
    path = BASE / name
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    BASE.mkdir(parents=True, exist_ok=True)
    terms = list(TERM_SIZES)
    size = sum(TERM_SIZES[n] for n in terms)
    # Real ONNX graph: output = input @ W + b, with explicit all-zero fixture weights.
    graph = helper.make_graph(
        [
            helper.make_node("MatMul", ["obs", "weights"], ["weighted"]),
            helper.make_node("Add", ["weighted", "bias"], ["actions"]),
        ],
        "FIXTURE_ONLY_NOT_A_ROBOT_POLICY",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, size])],
        [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 12])],
        [
            numpy_helper.from_array(np.zeros((size, 12), dtype=np.float32), "weights"),
            numpy_helper.from_array(np.zeros(12, dtype=np.float32), "bias"),
        ],
    )
    model = helper.make_model(
        graph,
        opset_imports=[helper.make_opsetid("", 17)],
        ir_version=9,
        producer_name="spot-deployment-fixture",
    )
    onnx.checker.check_model(model)
    path = BASE / "fixture.onnx"
    onnx.save(model, path)
    config_hash = write(
        "training-config.json", {"fixture": True, "trained": False, "seed": 0, "policy_hz": 50}
    )
    manifest = {
        "schema_version": 1,
        "purpose": "fixture",
        "task": "standing",
        "morphology": "spot-arm-stowed",
        "policy_file": "fixture.onnx",
        "policy_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "training_config_file": "training-config.json",
        "training_config_sha256": config_hash,
        "training_commit": "0" * 40,
        "input_name": "obs",
        "output_name": "actions",
        "policy_hz": 50,
        "stream_hz": 200,
        "observations": {
            "terms": [
                {"name": n, "scale": [1.0] * TERM_SIZES[n], "offset": [0.0] * TERM_SIZES[n]}
                for n in terms
            ],
            "joint_order": list(LEGS),
            "history_length": 1,
            "history_initialization": "zeros",
            "mean": [0.0] * size,
            "std": [1.0] * size,
            "clip": 100.0,
        },
        "actions": {
            "joint_order": list(LEGS),
            "default_positions": [0.0] * 12,
            "scale": [0.1] * 12,
            "clip_min": [-1.0] * 12,
            "clip_max": [1.0] * 12,
            "previous_action": "raw",
        },
        "gains": {"kp": [1.0] * 19, "kd": [0.1] * 19, "feedforward": [0.0] * 19},
        "arm_stowed_positions": [0.0] * 7,
        "robot_model_sha256": "0" * 64,
        "payload_config_sha256": "0" * 64,
        "payload_description": "SYNTHETIC FIXTURE; no physical robot or arm represented",
    }
    write("manifest.json", manifest)
    envelope = {
        "schema_version": 1,
        "joint_order": list(JOINTS),
        "position_min": [-2.0] * 19,
        "position_max": [2.0] * 19,
        "velocity_max": [2.0] * 19,
        "load_max": [2.0] * 19,
        "tracking_error_max": [0.5] * 19,
        "command_rate_max": [2.0] * 19,
        "max_roll_rad": 0.2,
        "max_pitch_rad": 0.2,
        "max_linear_speed": 0.3,
        "max_angular_speed": 0.3,
        "max_arm_pose_error": 0.02,
        "max_state_age_s": 0.1,
        "max_state_gap_s": 0.1,
        "max_future_skew_s": 0.001,
        "max_inference_s": 0.03,
        "max_policy_age_s": 0.2,
        "max_command_gap_s": 0.05,
        "max_command_ack_s": 0.06,
        "max_full_state_age_s": 0.5,
        "command_ttl_s": 0.08,
        "shutdown_timeout_s": 0.5,
        "transition_s": 0.2,
        "max_duration_s": 1.0,
        "min_battery_percent": 50.0,
    }
    write("envelope.json", envelope)
    states = []
    for i in range(20):
        states.append(
            {
                "robot_time_s": 1000 + i * 0.02,
                "received_monotonic_s": 100 + i * 0.02,
                "positions": [0.0] * 19,
                "velocities": [0.0] * 19,
                "loads": [0.0] * 19,
                "odom_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_odom": [0.0] * 3,
                "angular_velocity_odom": [0.0] * 3,
                "last_command_key": 0,
                "last_command_received_robot_s": 0.0,
            }
        )
    (BASE / "states.jsonl").write_text(
        "".join(json.dumps(s, sort_keys=True) + "\n" for s in states)
    )
    golden = [0.0] * size
    golden[8] = -1.0
    write("expected.json", {"observation": golden, "actions": [0.0] * 12, "targets": [0.0] * 19})
    schema_dir = BASE.parents[1] / "schemas"
    schema_dir.mkdir(exist_ok=True)
    from spot_deploy.contracts import Envelope, Manifest, RobotConfig, State, WatchRobotConfig
    from spot_deploy.readiness import EvidenceIndex
    from spot_deploy.deployment import DeploymentBundle
    from spot_deploy.estop_protocol import EstopProfile
    from spot_deploy.joystick_estop import JoystickProfile
    from spot_deploy.relic_contract import ReLICManifest
    from spot_deploy.exploy_policy import ExployReLICManifest
    from spot_deploy.walking import WalkingPlan

    for cls in (Manifest, Envelope, RobotConfig, WatchRobotConfig, State, EvidenceIndex,
                EstopProfile, JoystickProfile, ReLICManifest, ExployReLICManifest, DeploymentBundle, WalkingPlan):
        (schema_dir / f"{cls.__name__}.json").write_text(
            json.dumps(cls.model_json_schema(), indent=2) + "\n"
        )


if __name__ == "__main__":
    main()
