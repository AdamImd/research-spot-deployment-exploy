"""Independent policy and command schedules, with a single stop authority."""

import threading
import time

import numpy as np

from .contracts import ContractError
from .safety import Guard, Schedule, bounded_join, percentiles


def stand(reader, control, policy, envelope, duration, record, encode, interlock=None):
    stop = threading.Event()
    ready = threading.Event()
    emitted = threading.Event()
    lock = threading.Lock()
    shared = {"error": None, "result": None, "health_at": 0.0, "active": False}
    guard = Guard(policy.manifest, envelope)
    input_guard = Guard(policy.manifest, envelope)
    workers = []
    durations, command_gaps, acknowledgement = [], [], []
    epoch = None
    primary_error = None

    def fail(exc):
        with lock:
            if shared["error"] is None:
                shared["error"] = exc
        stop.set()
        ready.set()
        emitted.set()

    try:

        def infer():
            schedule = Schedule(policy.manifest.policy_hz)
            try:
                while not stop.is_set():
                    began = time.monotonic()
                    state = reader.mailbox.get()
                    input_guard.check_state(state, time.monotonic(), reader.robot_now())
                    targets, elapsed, obs, action = policy.predict(state)
                    if elapsed > envelope.max_inference_s:
                        raise ContractError("inference deadline exceeded")
                    with lock:
                        shared["result"] = (
                            targets,
                            min(began, state.received_monotonic_s),
                            elapsed,
                        )
                    durations.append(elapsed)
                    record.event(
                        "policy",
                        state=state.model_dump(),
                        observations=obs[0].tolist(),
                        raw_actions=action.tolist(),
                        targets=targets.tolist(),
                        inference_s=elapsed,
                    )
                    ready.set()
                    stop.wait(schedule.advance(time.monotonic()))
            except BaseException as exc:
                fail(exc)

        def monitor():
            try:
                while not stop.is_set():
                    control.heartbeat()
                    health = reader.health()
                    if interlock:
                        interlock.check(health)
                    record.event("health", **health)
                    if not health["estop_ready"] or health["fault_count"]:
                        raise ContractError("E-stop or active fault stopped standing")
                    if health["battery_percent"] < envelope.min_battery_percent:
                        raise ContractError("battery below operating envelope")
                    with lock:
                        shared["health_at"] = health["time"]
                        active = shared["active"]
                    if active:
                        control.check_stream()
                        if not control.active():
                            raise ContractError("joint controller unexpectedly inactive")
                    stop.wait(min(0.2, envelope.max_full_state_age_s / 3))
            except BaseException as exc:
                fail(exc)

        if interlock:
            interlock.check(reader.health())
        control.acquire()
        # Maintain lease/health while native power-on and standing are in progress too.
        health_worker = threading.Thread(target=monitor, name="spot-health", daemon=True)
        workers.append(health_worker)
        health_worker.start()
        control.native_stand(stop)
        with lock:
            if shared["error"]:
                raise shared["error"]
        first = reader.start_stream(duration + 5)
        guard.check_state(first, time.monotonic(), reader.robot_now())
        initial = np.array(first.positions)
        initial[12:] = policy.manifest.arm_stowed_positions
        policy.reset()
        epoch = time.monotonic()
        policy_worker = threading.Thread(target=infer, name="spot-policy", daemon=True)
        workers.append(policy_worker)
        policy_worker.start()
        if not ready.wait(reader.config.rpc_timeout_s):
            raise ContractError("policy startup timeout")
        while True:
            with lock:
                error, health_at = shared["error"], shared["health_at"]
            if error:
                raise error
            if health_at:
                break
            if time.monotonic() - epoch > reader.config.rpc_timeout_s:
                raise ContractError("health startup timeout")
            stop.wait(0.001)

        sent_times = {}
        last_ack = 0
        last_ack_at = time.monotonic()
        trial_start = None

        def commands():
            nonlocal last_ack, last_ack_at
            schedule = Schedule(policy.manifest.stream_hz)
            previous_send = None
            try:
                while not stop.is_set():
                    with lock:
                        result, health_at = shared["result"], shared["health_at"]
                    state = reader.mailbox.get()
                    now = time.monotonic()
                    if interlock:
                        interlock.check()
                    if now - health_at > envelope.max_full_state_age_s:
                        raise ContractError("full-state health watchdog expired")
                    targets, policy_at, elapsed = result
                    # Hold the measured standing pose until activation is acknowledged.
                    fraction = (
                        0
                        if trial_start is None
                        else min(1, (now - trial_start) / envelope.transition_s)
                    )
                    target = initial + fraction * (targets - initial)
                    target[12:] = policy.manifest.arm_stowed_positions
                    command = guard.command(
                        target, state, now, reader.robot_now(), policy_at, elapsed
                    )
                    if previous_send is not None:
                        command_gaps.append(now - previous_send)
                    previous_send = now
                    sent_times[command.key] = now
                    if state.last_command_key > last_ack and state.last_command_key in sent_times:
                        ack_delay = now - sent_times[state.last_command_key]
                        if ack_delay > envelope.max_command_ack_s:
                            raise ContractError("command acknowledgement latency exceeded")
                        acknowledgement.append(ack_delay)
                        last_ack, last_ack_at = state.last_command_key, now
                        for key in list(sent_times):
                            if key <= last_ack:
                                del sent_times[key]
                    if now - last_ack_at > envelope.max_command_ack_s:
                        raise ContractError("command acknowledgement watchdog expired")
                    record.event(
                        "command",
                        key=command.key,
                        positions=list(command.positions),
                        end_robot_time_s=command.end_robot_time_s,
                        monotonic_s=now,
                        state=state.model_dump(),
                    )
                    yield encode(command, policy.manifest, envelope)
                    emitted.set()
                    stop.wait(schedule.advance(time.monotonic()))
            except BaseException as exc:
                if not isinstance(exc, GeneratorExit):
                    fail(exc)

        control.start(commands(), duration + reader.config.rpc_timeout_s)
        if not emitted.wait(reader.config.rpc_timeout_s):
            raise ContractError("command stream startup timeout")
        with lock:
            if shared["error"]:
                raise shared["error"]
        control.activate()
        activation_deadline = time.monotonic() + reader.config.rpc_timeout_s
        while not control.active():
            with lock:
                if shared["error"]:
                    raise shared["error"]
            if time.monotonic() > activation_deadline:
                raise ContractError("joint control activation timeout")
            stop.wait(0.01)
        with lock:
            shared["active"] = True
        trial_start = time.monotonic()
        while time.monotonic() - trial_start < duration:
            with lock:
                if shared["error"]:
                    raise shared["error"]
            stop.wait(0.01)
        with lock:
            if shared["error"]:
                raise shared["error"]
    except BaseException as exc:
        primary_error = exc
        record.event(
            "first_failure",
            error_type=type(exc).__name__,
            reason=str(exc) if isinstance(exc, ContractError) else "External operation failed",
        )
        raise
    finally:
        stop.set()
        # Cancel commands and attempt the registered native safe-power-off before joining inference.
        errors = []
        try:
            control.close()
        except BaseException as exc:
            errors.append(exc)
        deadline = time.monotonic() + envelope.shutdown_timeout_s
        for worker in workers:
            try:
                bounded_join(worker, max(0, deadline - time.monotonic()))
            except BaseException as exc:
                errors.append(exc)
        if errors:
            record.event(
                "cleanup_failure",
                reason="Independent E-stop operator required",
                error_types=[type(e).__name__ for e in errors],
            )
            if primary_error is None:
                raise ContractError(
                    "Standing cleanup incomplete; independent E-stop operator required"
                ) from errors[0]
    return {
        "duration_s": duration,
        "inference_s": percentiles(durations),
        "command_gap_s": percentiles(command_gaps),
        "acknowledgement_s": percentiles(acknowledgement),
        "commands": guard.key,
        "walking": False,
        "arm_manipulation": False,
    }
