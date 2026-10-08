"""Bounded, read-only firmware/entitlement/state-stream diagnostic. Never commands Spot."""

import argparse
import json
import math
import time
from pathlib import Path

from spot_deploy.cli import safe_error
from spot_deploy.contracts import ContractError, RobotConfig, load
from spot_deploy.network import wired_route
from spot_deploy.records import RunRecord, atomic_json
from spot_deploy.sdk_read import ReadOnlySpot


def probe(reader, duration, record):
    from bosdyn.api import license_pb2
    from bosdyn.client.directory import DirectoryClient
    from bosdyn.client.license import LicenseClient
    from bosdyn.client.robot_state import RobotStateStreamingClient

    timeout = reader.config.rpc_timeout_s
    snapshot = reader.snapshot()
    atomic_json(record.directory / "robot-snapshot.json", snapshot)
    if not snapshot["identity_matches"] or not snapshot["joint_layout_matches"]:
        raise ContractError("compatibility probe identity or joint layout mismatch")
    directory = reader.robot.ensure_client(DirectoryClient.default_service_name).list(timeout=timeout)
    relevant = {"robot-state", "robot-state-streaming", "robot-command", "robot-command-streaming"}
    services = {entry.name: entry.type for entry in directory if entry.name in relevant}
    license_info = reader.robot.ensure_client(LicenseClient.default_service_name).get_license_info(
        timeout=timeout
    )
    # Never record the license identifier/key or raw SDK exception messages.
    result = {
        "firmware": snapshot["firmware"],
        "sdk_version": snapshot["sdk_version"],
        "joint_control_enabled": snapshot["joint_control_licensed"],
        "joint_control_feature_code": snapshot["joint_control_feature_code"],
        "license_status": license_pb2.LicenseInfo.Status.Name(license_info.status),
        "licensed_features": list(license_info.licensed_features),
        "services": services,
        "stream": {"available": False, "tested": False},
        "motion_commands": 0,
        "control_lease_acquired": False,
        "hardware_control_qualified": False,
    }
    if RobotStateStreamingClient.default_service_name not in services:
        result["stream"]["reason"] = "state-streaming service not advertised"
        return result
    result["stream"]["tested"] = True
    try:
        reader.stream_client = reader.robot.ensure_client(RobotStateStreamingClient.default_service_name)
        first = reader.start_stream(duration)
        record.event("state", state=first.model_dump())
        end = time.monotonic() + duration
        max_age = 0.0
        while time.monotonic() < end:
            state = reader.mailbox.get(timeout)
            age = reader.robot_now() - state.robot_time_s
            if abs(age) > 0.5 or time.monotonic() - state.received_monotonic_s > 0.5:
                raise ContractError("compatibility state stream is stale")
            max_age = max(max_age, age)
            time.sleep(0.02)
        record.event("state", state=state.model_dump())
        stats = reader.stream_statistics()
        result["stream"].update(available=stats["received_count"] >= 2,
                                 statistics=stats, max_observed_age_s=max_age,
                                 joints=len(state.positions))
    except Exception as exc:
        result["stream"].update(error=safe_error(exc), error_type=type(exc).__name__)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0.1 <= args.duration <= 5:
        parser.error("duration must be between 0.1 and 5 seconds")
    record = RunRecord(args.output, "compatibility-probe", [args.robot, Path(__file__)])
    reader = None
    try:
        config = load(args.robot, RobotConfig)
        record.event("route", **wired_route(config))
        reader = ReadOnlySpot(config)
        reader.connect(streaming=False)
        result = probe(reader, args.duration, record)
        reader.close()
        reader = None
        record.finish("passed", result)
        print(json.dumps(result, indent=2))
        return 0
    except Exception as exc:
        result = {"error": safe_error(exc)}
        record.finish("failed", result)
        print(json.dumps(result))
        return 1
    finally:
        if reader is not None:
            reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
