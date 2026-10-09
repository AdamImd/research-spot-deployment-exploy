"""Compact an existing rollout, or follow its log without connecting to a robot."""

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

from spot_deploy.contracts import JOINTS, State, sha256
from spot_deploy.records import atomic_json, source_identity, utcnow

MAX_LINE = 256_000
PREDICTIONS = {"policy", "shadow_sample", "replay_sample"}
FIELDS = {
    "command": ("key", "positions", "feedforward", "end_robot_time_s", "phase"),
    "policy_ack": ("command_key", "state_key", "raw_actions"),
    "relic_initialized": ("height", "requested_height_m", "previous_actions", "endpoint"),
    "relic_phase": ("phase",), "stock_phase": ("phase",),
    "relic_handover": ("torque_step_Nm", "previous_actions"),
    "preparation": ("phase", "reference", "feedforward"),
    "health": ("estop_ready", "battery_percent", "fault_count", "arm_stowed"),
    "stock_health": ("phase", "motor_state", "height", "faults", "stop_state"),
    "stock_command_received": ("phase", "command_id", "rpc_duration_s"),
    "stock_posture_reached": ("phase", "duration_s"),
    "height_latched": ("height_m", "source", "robot_time_s", "foot_contacts"),
    "first_failure": ("error_type", "reason", "error"),
    "cleanup_failure": ("reason", "error"),
    'walk_origin': ('odom_position_m', 'yaw_rad', 'robot_time_s', 'plan', 'source',
                    'reference_frame', 'distance_formula'),
    'walk_progress': ('phase', 'forward_m', 'lateral_m', 'heading_error_rad', 'remaining_m',
                      'speed_m_s', 'requested_forward_m_s', 'odom_position_m', 'robot_time_s'),
    'shutdown_result': ('motors_off_confirmed', 'lease_returned', 'elapsed_s', 'errors',
                        'within_shutdown_budget', 'motor_power_state'),
    "shadow_wait": ("reason", "robot_time_s", "state_age_s", "receive_age_s", "skipped_state_ticks"),
}
for _kind in PREDICTIONS:
    FIELDS[_kind] = ("targets", "raw_actions", "inference_s", 'velocity_command')
TIMES = ("time", "monotonic_s", "simulation_time_s")


def read_small(path):
    if not path.is_file():
        return {}
    if path.stat().st_size > 2_000_000:
        raise ValueError("metadata exceeds size limit")
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError("metadata must be an object")
    return value


class Projector:
    """Keep every prediction/command; sample only redundant measured-state copies."""

    def __init__(self, hz):
        if not math.isfinite(hz) or not 0 <= hz <= 1000:
            raise ValueError("state rate must be finite and in [0, 1000]")
        self.hz, self.origin, self.last_bucket = hz, None, -1
        self.last_stamp = None
        self.counts = Counter()

    def project(self, event, line):
        kind = event.get("event")
        if kind not in FIELDS and kind != "state":
            self.counts["omitted_events"] += 1
            return []
        base = {k: event[k] for k in TIMES if k in event}
        base["source_line"] = line
        rows = []
        state = None
        if "state" in event:
            state = State.model_validate(event["state"]).model_dump()
            # Preserve legacy records without inventing absent optional pose inputs.
            for field in ("body_position_odom", "body_pose_robot_time_s"):
                if field not in event["state"]:
                    state.pop(field)
            # Pydantic allows infinity on some scalar fields; the recording does not.
            json.dumps(state, allow_nan=False)
            stamp = state["robot_time_s"]
            base["state_robot_time_s"] = stamp
            base["state_last_command_key"] = state["last_command_key"]
            base["state_last_command_received_robot_s"] = state["last_command_received_robot_s"]
            if self.origin is None:
                self.origin = stamp
            bucket = math.floor((stamp - self.origin) * self.hz + 1e-4)
            due = ((not self.hz or bucket > self.last_bucket) and stamp != self.last_stamp)
            if kind in PREDICTIONS or due:
                self.last_bucket = max(bucket, self.last_bucket)
                self.last_stamp = stamp
                if kind not in PREDICTIONS:
                    rows.append(dict(base, event="state", state=state,
                                     phase=event.get("phase"), age_s=event.get("age_s")))
            elif kind == "state":
                self.counts["downsampled_states"] += 1
        if kind in FIELDS:
            row = dict(base, event=kind, **{k: event[k] for k in FIELDS[kind] if k in event})
            if kind in PREDICTIONS:
                if state is None:
                    raise ValueError("prediction missing measured state")
                for field, size in (("targets", 19), ("raw_actions", 12)):
                    value = row.get(field)
                    if (not isinstance(value, list) or len(value) != size
                            or not all(type(v) in (int, float) and math.isfinite(v) for v in value)):
                        raise ValueError("invalid prediction vector")
                row["state"] = state  # Preserve the exact inference input state, never interpolate.
            rows.append(row)
        for row in rows:
            self.counts[row["event"]] += 1
        return rows


def record(source, output, *, follow=False, duration=60., state_hz=50., manifest=None,
           stop=None):
    source, output = Path(source).resolve(), Path(output).resolve()
    if not source.is_dir() or output.is_relative_to(source):
        raise ValueError("source must exist; output must be outside the source run")
    if not math.isfinite(duration) or not 0 < duration <= 3600:
        raise ValueError("duration must be in (0, 3600] seconds")
    projector = Projector(state_hz)
    if not follow and not (source / "COMPLETE.json").is_file():
        raise ValueError("unfinished run: use --follow for bounded live recording")
    stop = stop or threading.Event()
    output.mkdir(parents=True, exist_ok=False)
    status, error, source_complete = "failed", None, {}
    line_count, read_bytes, pending, identity = 0, 0, b"", None
    digest = hashlib.sha256()
    started = time.monotonic()
    try:
        source_meta = read_small(source / "run.json")
        policy = read_small(source / "policy.json")
        config = read_small(source / "configuration.json")
        policy_manifest = read_small(Path(manifest)) if manifest else config.get("manifest", {})
        action_order = policy_manifest.get("actions", {}).get("joint_order") or policy.get("action_joint_names")
        identity_meta = source_identity()
        meta = {
            "schema_version": 1, "format": "gzip JSON Lines", "started_at": utcnow(),
            "source_run": str(source), "source_mode": source_meta.get("mode"),
            "source_commit": source_meta.get("source", {}).get("commit"),
            "recorder_commit": identity_meta["commit"], "recorder_dirty": identity_meta["dirty"],
            "recorder_sha256": sha256(Path(__file__)), "state_hz": state_hz,
            "all_policy_samples": True, "all_command_events": True,
            "joint_order": list(JOINTS), "raw_action_joint_order": action_order,
            "quaternion_order": "wxyz", "angular_units": "radians", "load_units": "N m",
            "source_metadata_sha256": {p.name: sha256(p) for p in (
                source / "run.json", source / "configuration.json", source / "policy.json") if p.is_file()},
            "manifest_sha256": sha256(Path(manifest)) if manifest else None,
            "robot_connection": False, "replaces_safety_log": False,
        }
        atomic_json(output / "recording.json", meta)
        path = source / "events.jsonl"
        with gzip.open(output / "rollout.jsonl.gz", "xt", compresslevel=1) as dest:
            while True:
                if stop.is_set() or (follow and time.monotonic() - started >= duration):
                    status = "interrupted" if stop.is_set() else "duration_limited"
                    break
                if path.exists():
                    stat = path.stat()
                    current_identity = (stat.st_dev, stat.st_ino)
                    if identity is not None and (current_identity != identity or stat.st_size < read_bytes):
                        raise ValueError("source events replaced or truncated")
                    identity = current_identity
                    with path.open("rb") as stream:
                        stream.seek(read_bytes)
                        chunk = stream.read(256_000)
                    if chunk:
                        read_bytes += len(chunk)
                        digest.update(chunk)
                        pieces = (pending + chunk).split(b"\n")
                        pending = pieces.pop()
                        if len(pending) > MAX_LINE:
                            raise ValueError("source event exceeds line limit")
                        for raw in pieces:
                            if len(raw) > MAX_LINE:
                                raise ValueError("source event exceeds line limit")
                            line_count += 1
                            event = json.loads(raw)
                            if not isinstance(event, dict):
                                raise ValueError("source event must be an object")
                            for row in projector.project(event, line_count):
                                dest.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
                        continue
                # Check marker before EOF again, so a final append racing the marker is drained.
                complete = read_small(source / "COMPLETE.json")
                if complete:
                    if path.exists() and path.stat().st_size > read_bytes:
                        continue
                    if identity is None:
                        raise ValueError("completed source has no event stream")
                    if pending:
                        raise ValueError("completed source has an unterminated event")
                    expected = complete.get("artifacts", {}).get("events.jsonl")
                    if expected and expected != digest.hexdigest():
                        raise ValueError("source event hash does not match completion record")
                    source_complete = complete
                    status = "completed"
                    break
                if not follow:
                    raise ValueError("source completion disappeared")
                dest.flush()
                stop.wait(.05)
    except Exception as exc:
        # Do not leak arbitrary source contents or SDK error messages.
        error = type(exc).__name__ + ": recording failed; inspect source format and paths"
    summary = {
        "status": status, "finished_at": utcnow(), "elapsed_s": time.monotonic() - started,
        "error": error, "source_lines": line_count, "source_bytes_read": read_bytes,
        "source_prefix_sha256": digest.hexdigest(), "partial_line_bytes": len(pending),
        "counts": dict(projector.counts), "source_status": source_complete.get("status"),
        "source_complete_sha256": sha256(source / "COMPLETE.json") if source_complete else None,
        "capture_complete": status == "completed", "hardware_qualified": False,
    }
    compressed = output / "rollout.jsonl.gz"
    summary["compressed_bytes"] = compressed.stat().st_size if compressed.exists() else 0
    atomic_json(output / "summary.json", summary)
    atomic_json(output / "COMPLETE.json", {
        "status": status, "artifacts": {p.name: sha256(p) for p in output.iterdir()
                                         if p.is_file() and p.name != "COMPLETE.json"}})
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--follow", action="store_true", help="read from beginning and follow live appends")
    parser.add_argument("--duration", type=float, default=60., help="follow time limit in seconds")
    parser.add_argument("--state-hz", type=float, default=50., help="measured-state cap; 0 keeps all unique states")
    parser.add_argument("--manifest", type=Path, help="optional policy manifest for raw-action joint labels")
    args = parser.parse_args()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        result = record(args.run, args.output, follow=args.follow, duration=args.duration,
                        state_hz=args.state_hz, manifest=args.manifest, stop=stop)
    except (OSError, ValueError):
        print("Recorder could not start; check paths, options and existing output directory.")
        return 1
    print(json.dumps(result, sort_keys=True))
    return 1 if result["status"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
