"""Explicit offline, read-only and control commands. Default failure is nonzero."""

import argparse
import json
import sys
from pathlib import Path

from pydantic import ValidationError

from .contracts import (
    ContractError, Envelope, RobotConfig, WatchRobotConfig, check_artifacts, load,
)
from .readiness import markdown_report, report, require_live
from .records import RunRecord, atomic_json
from .safety import Guard
from .relic_contract import ReLICManifest, load_manifest, make_policy


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="mode", required=True)
    prepare = commands.add_parser("prepare-deployment", help="Create an offline deployment review bundle")
    prepare.add_argument("--manifest", type=Path, required=True)
    prepare.add_argument("--robot", type=Path)
    prepare.add_argument("--envelope", type=Path, help="Reviewed hardware envelope, never simulation limits")
    prepare.add_argument("--output", type=Path, required=True)
    check = commands.add_parser("deployment-check", help="Check a bundle without contacting Spot")
    check.add_argument("--bundle", type=Path, required=True)
    check.add_argument("--preflight-run", type=Path)
    check.add_argument("--max-preflight-age-s", type=float, default=300)
    check.add_argument("--check-estop", action="store_true",
                       help="Read the local bridge status only; no setup, rearm or robot RPC")
    check.add_argument("--output", type=Path, required=True)
    for name in ("inspect-policy", "replay", "readiness", "preflight", "watch", "shadow", "stand"):
        cmd = commands.add_parser(name)
        cmd.add_argument(
            "--output", type=Path, required=True, help="New run directory; never overwritten"
        )
        cmd.add_argument(
            "--manifest",
            type=Path,
            required=name in ("inspect-policy", "replay", "shadow", "stand"),
        )
        cmd.add_argument("--envelope", type=Path, required=name in ("shadow", "stand"))
        cmd.add_argument(
            "--robot", type=Path, required=name in ("preflight", "watch", "shadow", "stand")
        )
        if name in ("readiness", "stand"):
            cmd.add_argument("--evidence", type=Path)
        if name == "replay":
            cmd.add_argument("--states", type=Path, required=True)
            cmd.add_argument("--height-record", type=Path,
                             help="Recorded initial height and foot contacts for ReLIC replay")
        if name in ("watch", "shadow", "stand"):
            cmd.add_argument(
                "--duration", type=float, required=True, help="Bounded duration in seconds"
            )
        if name == "watch":
            cmd.add_argument(
                "--state-source", choices=("stream", "poll"), default="stream",
                help="Read-only poll works without streaming entitlement and captures up to one hour",
            )
        if name == "stand":
            cmd.add_argument("--execute", action="store_true")
            cmd.add_argument("--operator", required=True)
            cmd.add_argument("--safety-operator", required=True)
    return result


def execute(args, record):
    if args.mode == "prepare-deployment":
        from .deployment import prepare

        return prepare(args.manifest, args.robot, args.envelope, record)
    if args.mode == "deployment-check":
        from .deployment import check

        result = check(args.bundle, args.preflight_run, args.max_preflight_age_s, args.check_estop)
        lines = ["# Deployment preparation check", "", result["note"], "",
                 "| Check | Status | Detail |", "| --- | --- | --- |"]
        for row in result["checks"]:
            reason = row["reason"].replace("|", "\\|").replace("\n", " ")
            lines.append(f"| {row['check']} | {row['status']} | {reason} |")
        (record.directory / "deployment-check.md").write_text("\n".join(lines) + "\n")
        return result
    manifest = load_manifest(args.manifest) if args.manifest else None
    envelope = load(args.envelope, Envelope) if args.envelope else None
    config_type = WatchRobotConfig if args.mode == "watch" else RobotConfig
    config = load(args.robot, config_type) if args.robot else None
    policy = None
    if manifest:
        policy_path = check_artifacts(args.manifest, manifest)
        policy = make_policy(policy_path, manifest, args.manifest)
    if manifest and envelope:
        Guard(manifest, envelope)
    if args.mode == "inspect-policy":
        return {
            "valid": True,
            "purpose": manifest.purpose,
            "hardware_qualified": False,
            "input_size": manifest.observations.input_size,
            "output_size": 12,
            "command_joints": 19,
            "policy_hz": manifest.policy_hz,
            "stream_hz": manifest.stream_hz,
            "morphology": manifest.morphology,
            "adapter": getattr(manifest, "adapter", "generic"),
        }
    if args.mode == "replay":
        from .runtime import replay

        if isinstance(manifest, ReLICManifest) and args.height_record:
            policy.initialize_height(json.loads(args.height_record.read_text()))
        return replay(args.states, policy, envelope, record)
    if args.mode == "readiness":
        result = report(args.manifest, args.envelope, args.robot, args.evidence)
        result["candidate_present"] = bool(manifest and manifest.purpose == "candidate")
        result["offline_contract_complete"] = bool(manifest and envelope and config)
        result["live_ready"] = False
        result["note"] = (
            "Live readiness also requires a fresh robot preflight and operator activation."
        )
        (record.directory / "readiness.md").write_text(markdown_report(result))
        return result

    if args.mode in ("watch", "shadow", "stand"):
        limit = 3600 if args.mode == "watch" and args.state_source == "poll" else 60
        if not 0 < args.duration <= limit:
            raise ContractError(f"duration must be finite and in (0, {limit}] seconds")
    if args.mode == "stand":
        result = require_live(
            manifest,
            envelope,
            config,
            args.manifest,
            args.envelope,
            args.robot,
            args.evidence,
            args.operator,
            args.safety_operator,
            args.execute,
            args.duration,
        )
        atomic_json(record.directory / "evidence-check.json", result)
        record.event("operators", operator=args.operator, safety_operator=args.safety_operator)

    # These imports and construction occur only after all offline gates above.
    from .network import wired_route
    from .sdk_read import ReadOnlySpot, validate_snapshot

    route = wired_route(config)
    record.event("route", **route)
    reader = ReadOnlySpot(config)
    primary_error = None
    try:
        if args.mode == "preflight" or (args.mode == "watch" and args.state_source == "poll"):
            reader.connect(streaming=False)
        else:
            reader.connect()
        snapshot = reader.snapshot()
        checks = validate_snapshot(snapshot, config, manifest, envelope, args.mode == "stand")
        atomic_json(record.directory / "robot-snapshot.json", snapshot)
        if getattr(reader, "robot_urdf", None):
            (record.directory / "robot.urdf").write_text(reader.robot_urdf)
        if args.mode == "preflight":
            return {
                "snapshot": snapshot,
                "checks": checks,
                "passed": all(checks.values()),
                "route": route,
                "motion_commands": 0,
                "control_lease_acquired": False,
                "note": "Snapshot only; use shadow for stream and inference timing.",
            }
        if not all(checks.values()):
            if args.mode == "watch":
                # Observability remains available while stopped/faulted; identity still matters.
                if not all(
                    checks[k] for k in ("identity_firmware", "joint_layout", "physical_arm")
                ):
                    raise ContractError("watch robot identity or morphology mismatch")
            else:
                raise ContractError("robot preflight failed; inspect robot-snapshot.json")
        if args.mode == "watch":
            from .runtime import watch

            return watch(reader, args.duration, record, args.state_source)
        if args.mode == "shadow":
            from .runtime import shadow

            if isinstance(manifest, ReLICManifest):
                height = reader.read_body_height()
                policy.initialize_height(height)
                record.event("height_latched", **height)
            return shadow(reader, policy, envelope, args.duration, record)
        if isinstance(manifest, ReLICManifest):
            from .relic_control import stand
        else:
            from .control_loop import stand
        from .sdk_control import ControlSpot, command_proto
        from .estop_interlock import HardwareInterlock

        return stand(
            reader,
            ControlSpot(reader, envelope),
            policy,
            envelope,
            args.duration,
            record,
            command_proto,
            HardwareInterlock(config) if config.hardware_estop else None,
        )
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        try:
            reader.close()
        except BaseException:
            record.event("read_cleanup_failure", reason="Reader did not close cleanly")
            if primary_error is None:
                raise


def safe_error(exc):
    # Do not print SDK exceptions or Pydantic input values: either may contain supplied secrets.
    if isinstance(exc, ContractError):
        return str(exc)
    if isinstance(exc, ValidationError):
        fields = [".".join(map(str, e["loc"])) for e in exc.errors(include_input=False)]
        return "Invalid contract fields: " + ", ".join(fields)
    return f"{type(exc).__name__}: operation failed; raw exception omitted to protect credentials"


def main(argv=None):
    args = parser().parse_args(argv)
    record = None
    try:
        inputs = [
            getattr(args, key, None)
            for key in ("manifest", "envelope", "robot", "states", "evidence", "height_record")
        ]
        if args.mode == "deployment-check":
            inputs.append(args.bundle / "bundle.json")
            if args.preflight_run:
                inputs.append(args.preflight_run / "COMPLETE.json")
        record = RunRecord(args.output, args.mode, inputs)
        result = execute(args, record)
        status = "failed" if result.get("passed") is False else "passed"
        record.finish(status, result)
        print(
            json.dumps(
                {
                    "status": status,
                    "mode": args.mode,
                    "output": str(args.output),
                    "live_ready": result.get("live_ready", False),
                }
            )
        )
        return 0 if status == "passed" else 1
    except BaseException as exc:
        if isinstance(exc, SystemExit):
            raise
        message = safe_error(exc)
        if record:
            try:
                record.finish("failed", {"error": message, "error_type": type(exc).__name__})
            except BaseException:
                message += "; completion record unavailable (partial artifacts preserved)"
        print(message, file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == "__main__":
    raise SystemExit(main())
