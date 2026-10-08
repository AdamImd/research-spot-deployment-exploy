"""Explicit USB E-stop bench, inspection, setup and persistent bridge commands."""

import argparse
import json
import signal
import secrets
import sys
import threading
from pathlib import Path

from .cli import safe_error
from .contracts import ContractError, RobotConfig, load, sha256
from .estop_bridge import SerialLink, StatusWriter, run_bridge, signal_stop
from .estop_interlock import robot_binding
from .estop_protocol import EstopProfile, parse
from .joystick_estop import JoystickLink, JoystickProfile, run_joystick_bridge
from .records import RunRecord


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="mode", required=True)
    for mode in ("identify", "bench", "inspect", "configure", "bridge"):
        cmd = commands.add_parser(mode)
        cmd.add_argument("--profile", type=Path, required=True)
        cmd.add_argument("--input", dest="input_type", choices=("esp32", "joystick"),
                         default="esp32", help="Physical input source (default: esp32)")
        cmd.add_argument("--output", type=Path, required=True, help="New run directory")
        if mode not in ("bench", "identify"):
            cmd.add_argument("--robot", type=Path, required=True)
        if mode in ("configure", "bridge"):
            cmd.add_argument("--execute", action="store_true")
            cmd.add_argument("--expected-config-id", required=True)
        if mode == "bridge":
            cmd.add_argument("--expected-endpoint-id", required=True)
        if mode in ("bench", "bridge"):
            cmd.add_argument("--status", type=Path, required=True)
            cmd.add_argument(
                "--duration",
                type=float,
                help="Optional bounded runtime; default until SIGINT/SIGTERM",
            )
    return result


def execute(args, record):
    joystick = args.input_type == "joystick"
    profile = load(args.profile, JoystickProfile if joystick else EstopProfile)
    config = load(args.robot, RobotConfig) if args.mode not in ("bench", "identify") else None
    profile_hash = sha256(args.profile)
    if args.mode in ("configure", "bridge") and not args.execute:
        raise ContractError("E-stop writes require explicit --execute")
    if (
        args.mode in ("bench", "bridge")
        and args.duration is not None
        and not 0 < args.duration <= 86400
    ):
        raise ContractError("E-stop duration must be finite and in (0, 86400] seconds")
    if args.mode == "bridge":
        requirement = config.hardware_estop
        if (
            requirement is None
            or requirement.profile_sha256 != profile_hash
            or Path(requirement.status_file).resolve() != args.status.resolve()
        ):
            raise ContractError(
                "robot configuration must bind this hardware profile and status path"
            )
    reader = endpoint = link = status = None
    primary_error = None
    bridge_entered = False
    phase = "input_setup"
    saved_signals = {}
    try:
        if args.mode == "identify":
            if joystick:
                link = JoystickLink(profile)
                link.poll()
                return dict(link.metadata, buttons=link.buttons, stop_button=profile.stop_button,
                            rearm_button=profile.rearm_button, robot_connected=False)
            link = SerialLink(profile)
            nonce = secrets.token_hex(8)
            frame = link.exchange(nonce)
            try:
                device_id = frame.decode("ascii").split(",")[1]
            except (UnicodeError, IndexError):
                raise ContractError("ESP32 did not return a valid identification frame") from None
            packet = parse(frame, nonce, device_id)
            if packet.armed:
                raise ContractError("press physical STOP before identifying the board")
            return {
                "device_id": packet.device_id,
                "boot_id": packet.boot,
                "stopped": packet.stopped,
            }
        # Claim the output and serial before any endpoint registration.
        if args.mode in ("bench", "bridge"):
            status = StatusWriter(
                args.status, profile, profile_hash, robot_binding(config) if config else None
            )
            status.write("stopped", "Starting; physical stop and rearm required")
            link = JoystickLink(profile) if joystick else SerialLink(profile)
        elif joystick and args.mode == "configure":
            # Validate actual USB identity before any configuration mutation.
            link = JoystickLink(profile)
            link.poll()
        if config:
            from .network import wired_route
            from .sdk_read import ReadOnlySpot
            from .sdk_estop import SpotEstop

            record.event("route", **wired_route(config))
            phase = "connection"
            reader = ReadOnlySpot(config)
            reader.connect(streaming=False)
            phase = "estop_service"
            endpoint = SpotEstop(reader, profile)
        if args.mode == "inspect":
            phase = "inspection"
            return endpoint.inspect()
        if args.mode == "configure":
            phase = "configuration"
            return endpoint.configure(args.expected_config_id)
        stop = threading.Event()
        saved_signals = signal_stop(stop)
        if endpoint:
            phase = "registration"
            endpoint.register(args.expected_config_id, args.expected_endpoint_id)
        phase = "input_loop"
        bridge_entered = True
        runner = run_joystick_bridge if joystick else run_bridge
        return runner(profile, link, status, record, endpoint, stop, args.duration)
    except BaseException as exc:
        primary_error = exc
        if phase == "connection" and reader:
            phase = reader.connection_stage
        record.event("estop_failure", phase=phase, error_type=type(exc).__name__)
        raise
    finally:
        # Registration and early setup errors also attempt STOP, then let the watchdog expire.
        cleanup_errors = []

        def cleanup(action):
            try:
                action()
            except BaseException as exc:
                cleanup_errors.append(exc)

        if endpoint and endpoint.endpoint and not bridge_entered:
            cleanup(endpoint.stop)
        if status:
            if not bridge_entered:
                cleanup(
                    lambda: status.write(
                        "fault" if primary_error else "stopped",
                        f"Bridge inactive ({phase}); restart and physical rearm required",
                        endpoint,
                    )
                )
            cleanup(status.close)
        if link:
            cleanup(link.close)
        if reader:
            cleanup(reader.close)
        for signum, handler in saved_signals.items():
            cleanup(lambda s=signum, h=handler: signal.signal(s, h))
        if cleanup_errors and primary_error is None:
            raise cleanup_errors[0]


def main(argv=None):
    args = parser().parse_args(argv)
    record = None
    try:
        record = RunRecord(
            args.output, f"estop-{args.mode}", [args.profile, getattr(args, "robot", None)]
        )
        result = execute(args, record)
        record.finish("passed", result)
        print(json.dumps(result))
        return 0
    except BaseException as exc:
        if isinstance(exc, SystemExit):
            raise
        message = safe_error(exc)
        if record:
            try:
                record.finish("failed", {"error": message, "error_type": type(exc).__name__})
            except BaseException:
                message += "; completion record unavailable"
        print(message, file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == "__main__":
    raise SystemExit(main())
