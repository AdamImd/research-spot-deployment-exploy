"""Read-only live ReLIC predictions. There is no execution flag or command adapter."""

import argparse
import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .cli import safe_error
from .contracts import ContractError, RobotConfig, load
from .kinematics import from_urdf, vector
from .network import wired_route
from .records import RunRecord, atomic_json
from .relic_policy import ACTION_JOINTS, CHECKPOINT_SHA256, POLICY_HZ, ROOT_COM_B, ReLICPolicy
from .safety import Schedule, bounded_join, percentiles
from .sdk_read import ReadOnlySpot


def run(reader, policy, duration, record):
    first = reader.start_stream(duration)
    start = time.monotonic()
    schedule = Schedule(POLICY_HZ, start)
    stop, errors = threading.Event(), []

    def health():
        try:
            while not stop.is_set():
                record.event("health", **reader.health())
                stop.wait(1)
        except Exception as exc:
            errors.append(exc)
            stop.set()

    monitor = threading.Thread(target=health, name="shadow-health", daemon=True)
    monitor.start()
    latency, ages, intervals = [], [], []
    skipped_state_ticks = 0
    last_tick, last_state, last_status = None, None, start - 1
    try:
        while time.monotonic() - start < duration:
            if errors:
                raise ContractError("shadow health read failed") from errors[0]
            state = reader.mailbox.get()
            now = time.monotonic()
            age = reader.robot_now() - state.robot_time_s
            if abs(age) > .1 or now - state.received_monotonic_s > .1:
                raise ContractError("ReLIC shadow state older than 100 ms")
            if last_state is not None and state.robot_time_s < last_state:
                raise ContractError("ReLIC shadow state timestamp moved backwards")
            if state.robot_time_s == last_state:
                # Delivery jitter may leave the same fresh sample in the mailbox.
                # Wait for another tick without advancing unexecuted action history.
                # The age checks above still terminate a stalled stream at 100 ms;
                # the mailbox independently rejects duplicate/reordered messages.
                skipped_state_ticks += 1
                record.event("shadow_wait", reason="awaiting_new_state",
                             robot_time_s=state.robot_time_s, state_age_s=age,
                             receive_age_s=now - state.received_monotonic_s,
                             skipped_state_ticks=skipped_state_ticks)
                time.sleep(schedule.advance(time.monotonic()))
                continue
            targets, elapsed, obs, action = policy.predict(state)
            if elapsed > .05:
                raise ContractError("ReLIC shadow inference exceeded 50 ms")
            record.event("shadow_sample", state=state.model_dump(), observations=obs.tolist(),
                         raw_actions=action.tolist(), targets=targets.tolist(), inference_s=elapsed)
            latency.append(elapsed)
            ages.append(age)
            if last_tick is not None:
                intervals.append(now - last_tick)
            last_tick, last_state = now, state.robot_time_s
            if now - last_status >= 1:
                span = state.robot_time_s - first.robot_time_s
                atomic_json(record.directory / "shadow-status.json", {
                    "samples": len(latency), "updated_unix_s": time.time(),
                    "state_hz": (reader.received_count - 1) / span if span > 0 else None,
                    "policy_hz": (len(latency) - 1) / (now - start) if now > start else None,
                    "skipped_state_ticks": skipped_state_ticks,
                    "motion_commands": 0,
                })
                last_status = now
            time.sleep(schedule.advance(time.monotonic()))
    finally:
        stop.set()
        bounded_join(monitor, 2 * reader.config.rpc_timeout_s + 1)
    return {"samples": len(latency), "skipped_state_ticks": skipped_state_ticks,
            "stream": reader.stream_statistics(),
            "inference_s": percentiles(latency), "state_age_s": percentiles(ages),
            "policy_loop_s": percentiles(intervals), "motion_commands": 0,
            "control_lease_acquired": False, "hardware_control_qualified": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=3600)
    parser.add_argument("--body-height", default="initial", metavar="initial|METRES",
                        help="Latch measured ground-relative starting height (default), or a fixed height")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0 < args.duration <= 3600:
        parser.error("duration must be in (0, 3600] seconds")
    record = reader = None
    try:
        config = load(args.robot, RobotConfig)
        # Validate the model before connecting. Resolve the starting height only
        # after identity/morphology checks, immediately before starting inference.
        policy = ReLICPolicy(args.checkpoint)
        record = RunRecord(args.output, "relic-shadow", [args.robot, args.checkpoint])
        metadata = {
            "name": "ReLIC · predictions only", "policy_hz": POLICY_HZ,
            "state_source": "high-frequency SDK stream", "checkpoint_sha256": CHECKPOINT_SHA256,
            "action_joint_names": ACTION_JOINTS, "velocity_command": [0, 0, 0],
            "arm_mode": "Initial measured arm pose held as input; arm is not predicted",
            "mode": "four-foot",
            "observation_size": 84, "root_com_b": ROOT_COM_B.tolist(),
            "note": "12 leg predictions only. Resting-pose outputs are diagnostic; no targets sent.",
        }
        record.event("route", **wired_route(config))
        reader = ReadOnlySpot(config)
        reader.connect()
        snapshot = reader.snapshot()
        atomic_json(record.directory / "robot-snapshot.json", snapshot)
        if not all(snapshot[k] for k in ("identity_matches", "joint_layout_matches", "has_arm")):
            raise ContractError("ReLIC shadow robot identity or morphology mismatch")
        urdf_path = record.directory / "robot.urdf"
        urdf_path.write_text(reader.robot_urdf)
        model = from_urdf(urdf_path)
        tree = ET.fromstring(reader.robot_urdf)
        root_link = next(link for link in tree.findall("link") if link.get("name") == model["root"])
        origin = root_link.find("inertial/origin")
        if origin is None or not np.allclose(vector(origin.get("xyz"), [0, 0, 0]),
                                             ROOT_COM_B, atol=1e-6, rtol=0):
            raise ContractError("robot root COM differs from ReLIC observation mapping")
        if args.body_height == "initial":
            height = reader.read_body_height()
        else:
            value = float(args.body_height)
            if not math.isfinite(value) or not 0 < value <= 1.5:
                raise ContractError("fixed body height must be in (0, 1.5] metres")
            height = {"height_m": value, "source": "explicit fixed height"}
        policy.body_height = height["height_m"]
        metadata.update(torso_target=[0, 0, policy.body_height], body_height=height)
        atomic_json(record.directory / "initial-height.json", height)
        atomic_json(record.directory / "policy.json", metadata)
        record.event("height_latched", **height)
        result = run(reader, policy, args.duration, record)
        reader.close()
        reader = None
        record.finish("passed", result)
        print(json.dumps(result))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        result = {"error": safe_error(exc), "motion_commands": 0}
        if record:
            record.finish("failed", result)
        print(json.dumps(result))
        return 1
    finally:
        if reader:
            reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
