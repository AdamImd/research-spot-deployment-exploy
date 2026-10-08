"""Import unchanged ReLIC weights with explicit startup settings; no robot access.

The result is an unqualified candidate. Hardware limits/evidence are separate.
"""

import argparse
import json
import shutil
from pathlib import Path

from spot_deploy.contracts import sha256
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.relic_contract import ReLICManifest
from spot_deploy.relic_policy import ACTION_IDS, ACTION_JOINTS, CHECKPOINT_SHA256, DEFAULT_Q


def import_candidate(source, capture, settings, output):
    if not verify_run(capture):
        raise ValueError("capture completion/hash verification failed")
    state = json.loads((capture / "initial-state.json").read_text())
    snapshot = json.loads((capture / "robot-snapshot.json").read_text())
    startup = json.loads(settings.read_text())
    asset = source / "source/relic/relic/assets/spot"
    if sha256(asset / "pretrained/policy.onnx") != CHECKPOINT_SHA256:
        raise ValueError("not the unchanged released ReLIC checkpoint")
    paths = ["relic/assets/spot/constants.py", "relic/assets/spot/spot.py",
             "relic/actuators/actuator_spot.py", "relic/tasks/loco_manipulation/interlimb_env_cfg.py",
             "relic/tasks/loco_manipulation/config/spot/spot_env_cfg.py",
             "relic/tasks/loco_manipulation/mdp/actions/spot_joint_actions.py"]
    files = {name: (source / "source/relic" / name).read_text() for name in paths}
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(asset / "pretrained/policy.onnx", output / "policy.onnx")
    shutil.copy2(asset / "spot_with_arm.urdf", output / "support.urdf")
    shutil.copy2(source / "LICENSE", output / "RELIC-LICENSE")
    atomic_json(output / "training-config.json", {
        "upstream_commit": "27f8033c5064d32f049a17accb71cd1091422878", "source_files": files,
        "note": "Released source configuration; final hardware-training provenance is not asserted.",
    })
    manifest = dict(schema_version=2, adapter="relic84", purpose="candidate", task="standing",
        morphology="spot-arm-stowed", policy_file="policy.onnx", policy_sha256=CHECKPOINT_SHA256,
        training_config_file="training-config.json",
        training_config_sha256=sha256(output / "training-config.json"),
        training_commit="27f8033c5064d32f049a17accb71cd1091422878",
        input_name="obs", output_name="actions", policy_hz=50, stream_hz=200,
        observations={"contract": "relic84-v1"},
        actions=dict(joint_order=list(ACTION_JOINTS), default_positions=DEFAULT_Q[ACTION_IDS].tolist(),
                     scale=[.2] * 12, clip_min=None, clip_max=None, previous_action="raw"),
        gains=dict(kp=[60.] * 12 + [120., 120., 120., 100., 100., 100., 16.],
                   kd=[1.5] * 12 + [2., 2., 2., 2., 2., 2., .32], feedforward=[0.] * 19),
        arm_stowed_positions=state["positions"][12:],
        robot_model_sha256=snapshot["robot_model_sha256"],
        payload_config_sha256=snapshot["payload_config_sha256"],
        payload_description="Captured Spot with stowed arm; qualification remains unverified",
        relic=startup | {"preparation": startup["preparation"] | {
            "model_file": "support.urdf", "model_sha256": sha256(output / "support.urdf")}})
    parsed = ReLICManifest.model_validate(manifest)
    atomic_json(output / "manifest.json", parsed.model_dump())
    atomic_json(output / "import.json", {"hardware_qualified": False,
        "capture_complete_sha256": sha256(capture / "COMPLETE.json"),
        "settings_sha256": sha256(settings), "checkpoint_unchanged": True,
        "license_sha256": sha256(output / "RELIC-LICENSE")})
    return output / "manifest.json"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-source", required=True, type=Path)
    parser.add_argument("--capture", required=True, type=Path)
    parser.add_argument("--settings", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(import_candidate(args.simulation_source, args.capture, args.settings, args.output))
