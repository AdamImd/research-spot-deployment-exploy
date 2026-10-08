"""Capture one fresh, synchronized Spot pose/height and a cold ReLIC prediction.

Read-only SDK operations. No leases, power, E-stop writes or robot commands.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from spot_deploy.cli import safe_error
from spot_deploy.contracts import ContractError, JOINTS, RobotConfig, load
from spot_deploy.network import wired_route
from spot_deploy.records import RunRecord, atomic_json
from spot_deploy.relic_policy import ReLICPolicy
from spot_deploy.sdk_read import ReadOnlySpot, body_height_from_state, decode_full_state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, type=Path)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    record = RunRecord(args.output, "relic-pose-capture", [args.robot, args.checkpoint, Path(__file__)])
    reader = None
    try:
        config = load(args.robot, RobotConfig)
        record.event("route", **wired_route(config))
        reader = ReadOnlySpot(config)
        reader.connect(streaming=False)
        snapshot = reader.snapshot()
        if not all(snapshot[k] for k in ("identity_matches", "joint_layout_matches", "has_arm")):
            raise ContractError("capture identity or morphology mismatch")
        message = reader.state_client.get_robot_state(timeout=config.rpc_timeout_s)
        state = decode_full_state(message)
        height = body_height_from_state(message)
        age = reader.robot_now() - state.robot_time_s
        if abs(age) > .25:
            raise ContractError("pose capture older than 250 ms")
        height["state_age_s"] = age
        assert height["robot_time_s"] == state.robot_time_s
        policy = ReLICPolicy(args.checkpoint, body_height=height["height_m"])
        target, elapsed, observation, raw = policy.predict(state)
        assert not np.any(observation[72:84])
        atomic_json(args.output / "initial-state.json", state.model_dump())
        atomic_json(args.output / "initial-height.json", height)
        atomic_json(args.output / "robot-snapshot.json", snapshot)
        atomic_json(args.output / "physical-first-prediction.json", {
            "joint_names": JOINTS, "targets": target.tolist(), "raw_actions": raw.tolist(),
            "observations": observation.tolist(), "inference_s": elapsed,
            "previous_actions": "zeros", "motion_commands": 0,
        })
        result = dict(height_m=height["height_m"], state_age_s=age,
            robot_time_s=state.robot_time_s, captured_unix_s=time.time(),
            foot_contacts=height["foot_contacts"], motion_commands=0,
            control_lease_acquired=False,
            max_leg_target_gap_deg=float(np.rad2deg(np.abs(target[:12] - state.positions[:12])).max()))
        reader.close()
        reader = None
        record.finish("passed", result)
        print(json.dumps(result))
        return 0
    except Exception as exc:
        result = {"error": safe_error(exc), "motion_commands": 0}
        record.finish("failed", result)
        print(json.dumps(result))
        return 1
    finally:
        if reader is not None:
            reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
