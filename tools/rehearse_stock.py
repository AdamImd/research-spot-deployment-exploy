"""Supervised stock sit/power-on/stand/sit/power-off rehearsal. Never enters joint control."""

import argparse
import json
from pathlib import Path
import signal
import threading
import time

from spot_deploy.cli import safe_error
from spot_deploy.contracts import ContractError, Envelope, RobotConfig, load
from spot_deploy.network import wired_route
from spot_deploy.records import RunRecord, atomic_json
from spot_deploy.relic_contract import load_manifest
from spot_deploy.sdk_read import ReadOnlySpot, body_height_from_state, estop_observation, validate_snapshot


def faults_from_state(state):
    from google.protobuf.json_format import MessageToDict

    return {kind: [MessageToDict(f) for f in getattr(state, kind).faults]
            for kind in ("system_fault_state", "behavior_fault_state", "service_fault_state")}


def require_stock_faults(faults, allow_payload_info):
    """One diagnostic-only exception; never changes the ReLIC all-faults gate."""
    for kind, entries in faults.items():
        for fault in entries:
            allowed = (allow_payload_info and kind == "system_fault_state"
                       and fault.get("name") == "payload.fault" and fault.get("code") == 9
                       and fault.get("severity") == "SEVERITY_INFO")
            if not allowed:
                raise ContractError("stock rehearsal blocked by active fault")


def require_operators(operator, safety_operator):
    if not operator.strip() or not safety_operator.strip():
        raise ContractError("two named operators are required")
    if operator.strip().casefold() == safety_operator.strip().casefold():
        raise ContractError("robot and tablet-stop operators must be distinct")


def stock_sequence(io, initially_on):
    """Exact bounded motion sequence; io owns observation, cancellation and timeouts."""
    if initially_on:
        io.sit("initial_sit")
        io.power_off("initial_power_off")
    io.power_on()
    io.stand()
    io.hold("standing_observation", 10.)
    io.sit("sit")
    io.hold("sitting_observation", 2.)
    io.power_off("power_off")


class StockIO:
    def __init__(self, reader, command, record, check, set_phase):
        self.reader, self.command, self.record = reader, command, record
        self.check, self.set_phase = check, set_phase
        self.durations = {}
        self.power_managed = False

    def power_on(self):
        self.check()
        self.set_phase("power_on")
        self.power_managed = True  # A timed-out request can still have taken effect.
        started = time.monotonic()
        self.reader.robot.power_on(timeout_sec=20, update_frequency=10,
                                   timeout=self.reader.config.rpc_timeout_s)
        self.durations["power_on"] = time.monotonic() - started
        self.check()

    def power_off(self, phase):
        self.check()
        self.set_phase(phase)
        self.power_managed = True
        started = time.monotonic()
        self.reader.robot.power_off(cut_immediately=False, timeout_sec=15, update_frequency=10,
                                    timeout=self.reader.config.rpc_timeout_s)
        require_power_off(self.reader)
        self.durations[phase] = time.monotonic() - started
        self.check()

    def posture(self, phase, posture):
        from bosdyn.api import basic_command_pb2
        from bosdyn.client.robot_command import RobotCommandBuilder

        self.check()
        self.set_phase(phase)
        self.power_managed = True
        started = time.monotonic()
        build = (RobotCommandBuilder.synchro_stand_command if posture == "stand"
                 else RobotCommandBuilder.synchro_sit_command)
        command_id = self.command.robot_command(build(), timeout=self.reader.config.rpc_timeout_s)
        self.record.event("stock_command_received", phase=phase, command_id=command_id,
                          rpc_duration_s=time.monotonic() - started)
        expected = (basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING if posture == "stand"
                    else basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING)
        while time.monotonic() - started < 15:
            self.check()
            feedback = self.command.robot_command_feedback(
                command_id, timeout=self.reader.config.rpc_timeout_s)
            mobility = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility.status != basic_command_pb2.RobotCommandFeedbackStatus.STATUS_PROCESSING:
                raise ContractError("stock posture command rejected or interrupted")
            if getattr(mobility, posture + "_feedback").status == expected:
                self.durations[phase] = time.monotonic() - started
                self.record.event("stock_posture_reached", phase=phase,
                                  duration_s=self.durations[phase])
                return
            time.sleep(.05)
        raise ContractError("stock posture timeout")

    def stand(self):
        self.posture("stand", "stand")

    def sit(self, phase):
        self.posture(phase, "sit")

    def hold(self, phase, duration):
        self.set_phase(phase)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.check()
            time.sleep(.02)


def require_power_off(reader):
    from bosdyn.api.robot_state_pb2 import PowerState

    state = reader.state_client.get_robot_state(timeout=reader.config.rpc_timeout_s)
    if state.power_state.motor_power_state != PowerState.STATE_OFF:
        raise ContractError("motor shutdown not confirmed")


def run(args, record):
    from bosdyn.api.robot_state_pb2 import ManipulatorState, PowerState
    from bosdyn.client.estop import EstopClient
    from bosdyn.client.lease import LeaseClient

    config = load(args.robot, RobotConfig)
    manifest, envelope = load_manifest(args.manifest), load(args.envelope, Envelope)
    if config.estop_authority != "tablet" or config.hardware_estop is not None:
        raise ContractError("stock rehearsal requires the selected tablet authority")
    require_operators(args.operator, args.safety_operator)
    reader = ReadOnlySpot(config)
    record.event("operators", operator=args.operator, safety_operator=args.safety_operator)
    record.event("route", **wired_route(config))
    stop, cancelled = threading.Event(), threading.Event()
    errors, phase, summaries = [], ["preflight"], []
    lease, lease_client, monitor, io = None, None, None, None
    result = {"completed": False, "joint_control_activated": False,
              "policy_inference": False, "estop_writes": 0, "fault_clear_requests": 0}
    deadline = time.monotonic() + 120

    def check():
        if errors:
            raise ContractError(errors[0])
        if cancelled.is_set() or time.monotonic() > deadline:
            raise ContractError("stock rehearsal cancelled or deadline reached")

    def set_phase(name):
        check()
        phase[0] = name
        record.event("stock_phase", phase=name, monotonic_s=time.monotonic())

    previous_handlers = {sig: signal.signal(sig, lambda *_: cancelled.set())
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        reader.connect()
        snapshot = reader.snapshot()
        atomic_json(record.directory / "robot-snapshot.json", snapshot)
        state = reader.state_client.get_robot_state(timeout=config.rpc_timeout_s)
        faults = faults_from_state(state)
        require_stock_faults(faults, args.allow_known_payload_info)
        checks = validate_snapshot(snapshot, config, manifest, envelope)
        checks["no_unaccepted_stock_faults"] = True
        # Only the recorded diagnostic exception above applies to stock motion.
        checks.pop("no_active_faults")
        record.event("stock_preflight", checks=checks, faults=faults)
        if not all(checks.values()):
            raise ContractError("stock preflight failed")
        power = state.power_state.motor_power_state
        if power not in (PowerState.STATE_OFF, PowerState.STATE_ON):
            raise ContractError("motor power is not in a stable ON/OFF state")
        lease_client = reader.robot.ensure_client(LeaseClient.default_service_name)
        resources = lease_client.list_leases(timeout=config.rpc_timeout_s)
        record.event("lease_observation", resources=[
            {"resource": x.resource, "owner_client": x.lease_owner.client_name} for x in resources])
        result.update(initial_motor_state=PowerState.MotorPowerState.Name(power),
                      initial_faults=faults, checks=checks)
        if not args.execute:
            result["inspection_only"] = True
            return result

        # Explicit execution, two operators and stock health checks precede all writes.
        from bosdyn.client.robot_command import RobotCommandClient

        lease = lease_client.acquire(timeout=config.rpc_timeout_s)  # Never take/steal a lease.
        record.event("lease_acquired")
        command = reader.robot.ensure_client(RobotCommandClient.default_service_name)
        estop_client = reader.robot.ensure_client(EstopClient.default_service_name)
        reader.start_stream(145)

        def observe():
            next_health, next_lease, last_stamp = 0., 0., None
            try:
                while not stop.is_set():
                    measured = reader.mailbox.get(config.rpc_timeout_s)
                    now = time.monotonic()
                    age = reader.robot_now() - measured.robot_time_s
                    if not -.01 <= age <= .1 or now - measured.received_monotonic_s > .1:
                        raise ContractError("stock observation stream stale")
                    if measured.robot_time_s != last_stamp:
                        record.event("state", phase=phase[0], state=measured.model_dump(), age_s=age)
                        last_stamp = measured.robot_time_s
                    if now >= next_lease:
                        lease_client.retain_lease(lease, timeout=config.rpc_timeout_s)
                        next_lease = now + 1.
                    if now >= next_health:
                        full = reader.state_client.get_robot_state(timeout=config.rpc_timeout_s)
                        stop_status = estop_client.get_status(timeout=config.rpc_timeout_s)
                        stops = estop_observation(config, full, stop_status)
                        active_faults = faults_from_state(full)
                        require_stock_faults(active_faults, args.allow_known_payload_info)
                        if not stops["estop_ready"]:
                            raise ContractError("tablet/robot stop state interrupted stock rehearsal")
                        if full.manipulator_state.stow_state != ManipulatorState.STOWSTATE_STOWED:
                            raise ContractError("arm no longer stowed")
                        if min(b.charge_percentage.value for b in full.battery_states) < envelope.min_battery_percent:
                            raise ContractError("battery below reviewed floor")
                        try:
                            height = body_height_from_state(full)
                        except ContractError:
                            height = None  # Resting GPE can be invalid; never substitute a height.
                        observation = dict(phase=phase[0], motor_state=PowerState.MotorPowerState.Name(
                            full.power_state.motor_power_state), height=height, faults=active_faults,
                            stop_state=stops)
                        summaries.append(observation)
                        record.event("stock_health", **observation)
                        next_health = now + .1
                    stop.wait(.01)
            except Exception as exc:
                errors.append(safe_error(exc))

        monitor = threading.Thread(target=observe, name="stock-observer", daemon=True)
        monitor.start()
        io = StockIO(reader, command, record, check, set_phase)
        # Get at least one monitored sample/health check before the first motion.
        start = time.monotonic()
        while not summaries:
            check()
            if time.monotonic() - start > config.rpc_timeout_s * 3:
                raise ContractError("stock observer failed to start")
            time.sleep(.01)
        stock_sequence(io, initially_on=power == PowerState.STATE_ON)
        require_power_off(reader)
        check()
        result.update(completed=True, final_motors_off=True, durations_s=io.durations,
                      state_stream=reader.stream_statistics())
    except Exception as exc:
        result["error"] = safe_error(exc)
        result["error_type"] = type(exc).__name__
    finally:
        # Cleanup must still execute when logging or observation has failed.
        if io is not None and io.power_managed and not result.get("final_motors_off"):
            try:
                reader.robot.power_off(cut_immediately=False, timeout_sec=15, update_frequency=10,
                                       timeout=config.rpc_timeout_s)
                require_power_off(reader)
                result["cleanup_motors_off"] = True
            except Exception as exc:
                result["cleanup_motors_off"] = False
                result["cleanup_error"] = safe_error(exc)
        stop.set()
        if monitor is not None:
            monitor.join(10)
            if monitor.is_alive():
                result.update(completed=False, observer_shutdown_unconfirmed=True)
        if errors:
            result.update(completed=False, observer_error=errors[0])
        if lease is not None:
            try:
                lease_client.return_lease(lease, timeout=config.rpc_timeout_s)
                result["lease_returned"] = True
            except Exception as exc:
                result.update(completed=False, lease_return_error=safe_error(exc))
        try:
            reader.close()
        except Exception as exc:
            result.update(completed=False, reader_shutdown_error=safe_error(exc))
        for sig, previous in previous_handlers.items():
            signal.signal(sig, previous)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("robot", "manifest", "envelope", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--safety-operator", required=True)
    parser.add_argument("--allow-known-payload-info", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    record = RunRecord(args.output, "stock-rehearsal", [
        args.robot, args.manifest, args.envelope, Path(__file__)])
    try:
        result = run(args, record)
    except Exception as exc:
        result = {"completed": False, "error": safe_error(exc)}
    passed = (result.get("completed") or result.get("inspection_only")) and not any(
        key in result for key in ("error", "reader_shutdown_error", "observer_error",
                                  "observer_shutdown_unconfirmed", "lease_return_error"))
    record.finish("passed" if passed else "failed", result)
    print(json.dumps({"passed": bool(passed), "output": str(args.output),
                      "completed": result.get("completed"), "error": result.get("error"),
                      "motors_off": result.get("final_motors_off", result.get("cleanup_motors_off"))}))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
