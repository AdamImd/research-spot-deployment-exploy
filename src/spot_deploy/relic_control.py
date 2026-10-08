"""Guarded orchestration around the SDK-free ReLIC rollout core.

Imported for physical execution only after the existing evidence/preflight gates.
Policy computation runs in the command producer; a stalled computation cannot renew
the last command's expiry. An independent watchdog cancels a stalled producer.
"""

import threading
import time

from .contracts import ContractError
from .relic_rollout import ReLICRollout
from .safety import Schedule, bounded_join, percentiles


def stand(reader, control, policy, envelope, duration, record, encode, interlock=None):
    core = ReLICRollout(policy, envelope, record)
    stop, emitted, lock = threading.Event(), threading.Event(), threading.Lock()
    shared = dict(error=None, health_at=None, active=False, last_emit=None)
    gaps, acknowledgements = [], []
    worker = None

    def fail(exc):
        with lock:
            if shared["error"] is None:
                shared["error"] = exc
        stop.set()
        emitted.set()

    def check_error():
        with lock:
            error = shared["error"]
        if error is not None:
            raise error

    def health_monitor():
        try:
            while not stop.is_set():
                control.heartbeat()
                health = reader.health()
                if interlock:
                    interlock.check(health)
                if not health["estop_ready"] or health["fault_count"]:
                    raise ContractError("E-stop or robot fault stopped ReLIC")
                if health["battery_percent"] < envelope.min_battery_percent:
                    raise ContractError("battery below operating envelope")
                record.event("health", **health)
                with lock:
                    shared["health_at"] = health["time"]
                    active = shared["active"]
                if active:
                    control.check_stream()
                    if not control.active():
                        raise ContractError("joint controller unexpectedly inactive")
                stop.wait(min(.1, envelope.max_full_state_age_s / 3))
        except BaseException as exc:
            fail(exc)

    primary_error = None
    try:
        if interlock:
            interlock.check(reader.health())
        control.acquire()
        worker = threading.Thread(target=health_monitor, name="relic-health", daemon=True)
        worker.start()
        control.native_stand(stop)
        check_error()
        height = reader.read_body_height()
        first = reader.start_stream(duration + reader.config.rpc_timeout_s + 5)
        core.initialize(first, height, time.monotonic(), reader.robot_now())
        deadline = time.monotonic() + reader.config.rpc_timeout_s
        while shared["health_at"] is None:
            check_error()
            if time.monotonic() > deadline:
                raise ContractError("health startup timeout")
            stop.wait(.001)
        sent, last_ack = {}, first.last_command_key
        last_ack_at = time.monotonic()

        def commands():
            nonlocal last_ack, last_ack_at, sent
            schedule = Schedule(policy.manifest.stream_hz)
            previous_send = None
            try:
                while not stop.is_set():
                    now = time.monotonic()
                    with lock:
                        health_at = shared["health_at"]
                        active = shared["active"]
                    if now - health_at > envelope.max_full_state_age_s:
                        raise ContractError("full-state health watchdog expired")
                    if interlock:
                        interlock.check()
                    state = reader.mailbox.get()
                    now = time.monotonic()
                    if state.last_command_key > last_ack:
                        if state.last_command_key not in sent:
                            raise ContractError("unknown command acknowledgement")
                        delay = now - sent[state.last_command_key]
                        if delay > envelope.max_command_ack_s:
                            raise ContractError("command acknowledgement latency exceeded")
                        acknowledgements.append(delay)
                        last_ack, last_ack_at = state.last_command_key, now
                        sent = {key: stamp for key, stamp in sent.items() if key > last_ack}
                    if now - last_ack_at > envelope.max_command_ack_s:
                        raise ContractError("command acknowledgement watchdog expired")
                    if active and core.active_at is None:
                        core.activate(now)
                    command = core.command(state, now, reader.robot_now())
                    finished = time.monotonic()
                    if finished - now > envelope.max_inference_s:
                        raise ContractError("ReLIC command computation deadline exceeded")
                    if stop.is_set():
                        break
                    if previous_send is not None:
                        gap = finished - previous_send
                        if gap > envelope.max_command_gap_s:
                            raise ContractError("command scheduling deadline exceeded")
                        gaps.append(gap)
                    previous_send = finished
                    sent[command.key] = finished
                    record.event("command", key=command.key, positions=list(command.positions),
                                 feedforward=list(command.feedforward), phase=core.phase,
                                 end_robot_time_s=command.end_robot_time_s, monotonic_s=finished,
                                 state=state.model_dump())
                    with lock:
                        shared["last_emit"] = finished
                    yield encode(command, policy.manifest, envelope)
                    emitted.set()
                    stop.wait(schedule.advance(time.monotonic()))
            except BaseException as exc:
                if not isinstance(exc, GeneratorExit):
                    fail(exc)

        control.start(commands(), duration + 2 * reader.config.rpc_timeout_s)
        if not emitted.wait(reader.config.rpc_timeout_s):
            raise ContractError("command stream startup timeout")
        check_error()
        control.activate()
        deadline = time.monotonic() + reader.config.rpc_timeout_s
        while not control.active():
            check_error()
            if time.monotonic() > deadline:
                raise ContractError("joint-control activation timeout")
            stop.wait(.005)
        start = time.monotonic()
        with lock:
            shared["active"] = True
        while time.monotonic() - start < duration:
            check_error()
            with lock:
                last_emit = shared["last_emit"]
            if last_emit is None or time.monotonic() - last_emit > envelope.max_command_gap_s:
                raise ContractError("ReLIC command producer stalled")
            stop.wait(min(.005, envelope.max_command_gap_s / 3))
        check_error()
        if core.phase != "policy" or core.policy_samples < 1:
            raise ContractError("standing trial ended before ReLIC handover")
    except BaseException as exc:
        primary_error = exc
        record.event("first_failure", error_type=type(exc).__name__,
                     reason=str(exc) if isinstance(exc, ContractError) else "External operation failed")
        raise
    finally:
        stop.set()
        errors = []
        try:
            control.close()
        except BaseException as exc:
            errors.append(exc)
        if worker:
            try:
                bounded_join(worker, envelope.shutdown_timeout_s)
            except BaseException as exc:
                errors.append(exc)
        if errors:
            record.event("cleanup_failure", reason="Independent E-stop operator required",
                         error_types=[type(e).__name__ for e in errors])
            if primary_error is None:
                raise ContractError("ReLIC cleanup incomplete; independent E-stop operator required")
    record.event("relic_phase", phase="stopped")
    return dict(commands=core.guard.key - first.last_command_key,
                policy_samples=core.policy_samples, command_gap_s=percentiles(gaps),
                command_ack_s=percentiles(acknowledgements), shutdown_confirmed=True,
                phase="stopped", duration_s=duration)
