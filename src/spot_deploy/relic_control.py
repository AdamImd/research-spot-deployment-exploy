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
from .sdk_read import classify_faults


def stand(reader, control, policy, envelope, duration, record, encode, interlock=None, teleop_source=None, modes=None, verify_enable=None):
    # Construct and validate the first rollout before acquiring the lease or powering Spot.
    initial_core = ReLICRollout(policy, envelope, record, teleop_source)
    if verify_enable and modes is None:
        raise ContractError("a deferred policy handoff requires operator mode requests")
    feedback_lock = threading.Lock()
    stop, emitted, lock = threading.Event(), threading.Event(), threading.Lock()
    shared = dict(error=None, health_at=None, active=False, last_emit=None, motion_done=False, native_id=None)
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

    finish_requested = getattr(teleop_source, "finish_requested", lambda: False)

    def check_startup_cancel():
        if finish_requested():
            raise ContractError("operator cancelled before joystick policy activation")

    def health_monitor():
        try:
            while not stop.is_set():
                if not shared["active"] and (modes is None or not shared.get("standing")):
                    check_startup_cancel()
                control.heartbeat()
                health = reader.health()
                health.update(classify_faults(health.get('fault_details', []),
                                             health['fault_count'], reader.config))
                record.event("health", **health)
                if interlock:
                    interlock.check(health)
                if not health["estop_ready"] or not health['fault_policy_ready']:
                    raise ContractError("E-stop or robot fault stopped ReLIC")
                if verify_enable and not shared["active"]:
                    if health["fault_count"] or not health["arm_stowed"]:
                        raise ContractError("native standing requires a stowed arm and no active faults")
                    measured = reader.mailbox.get(reader.config.rpc_timeout_s)
                    age = reader.robot_now() - measured.robot_time_s
                    if not -.01 <= age <= .1 or time.monotonic() - measured.received_monotonic_s > .1:
                        raise ContractError("native standing observation stream stale")
                    record.event("state", phase="native", state=measured.model_dump(), age_s=age)
                if health["battery_percent"] < envelope.min_battery_percent:
                    raise ContractError("battery below operating envelope")
                with lock:
                    shared["health_at"] = health["time"]
                with feedback_lock:
                    if shared["active"]:
                        control.check_stream()
                        if not control.active():
                            raise ContractError("joint controller unexpectedly inactive")
                    elif shared["native_id"] is not None:
                        if not control.is_standing(shared["native_id"]):
                            raise ContractError("native standing feedback lost")
                stop.wait(min(.1, envelope.max_full_state_age_s / 3))
        except BaseException as exc:
            fail(exc)

    def run_segment():
        nonlocal stream_started
        core = initial_core if not samples else ReLICRollout(policy, envelope, record, teleop_source)
        segment_stop, producer_done = threading.Event(), threading.Event()
        emitted.clear()
        with lock:
            shared.update(last_emit=None, motion_done=False)
        reset_input = getattr(teleop_source, 'reset_after_native_stand', None)
        if reset_input is not None:
            reset_input()
            record.event('joystick_startup_reset', pre_policy_events_discarded=True,
                         fresh_neutral_enable_required=True)
        height = reader.read_body_height()
        if not stream_started:
            first = reader.start_stream(duration + reader.config.rpc_timeout_s + 5)
            stream_started = True
        else:
            first = reader.mailbox.get(reader.config.rpc_timeout_s)
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
                while not stop.is_set() and not segment_stop.is_set():
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
                    if stop.is_set() or segment_stop.is_set():
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
                                 guard_monotonic_s=now,
                                 state=state.model_dump())
                    with lock:
                        shared["last_emit"] = finished
                    yield encode(command, policy.manifest, envelope)
                    emitted.set()
                    if (core.motion and core.motion.completed) or (core.teleop and core.teleop.completed):
                        with lock:
                            shared['motion_done'] = True
                    stop.wait(schedule.advance(time.monotonic()))
            except BaseException as exc:
                if not isinstance(exc, GeneratorExit) and not segment_stop.is_set():
                    if isinstance(exc, ContractError) and 'state' in locals():
                        try:
                            record.event('guard_failure', reason=str(exc), state=state.model_dump())
                        except BaseException:
                            pass  # Preserve the first error; cleanup still runs.
                    fail(exc)
            finally:
                producer_done.set()

        control.start(commands(), duration + 2 * reader.config.rpc_timeout_s)
        if not emitted.wait(reader.config.rpc_timeout_s):
            raise ContractError("command stream startup timeout")
        check_error()
        check_startup_cancel()
        with feedback_lock:
            if modes and not control.is_standing(shared["native_id"]):
                raise ContractError("native stand lost before ReLIC handover")
            shared["native_id"] = None
            control.activate()
        deadline = time.monotonic() + reader.config.rpc_timeout_s
        while not control.active():
            check_error()
            check_startup_cancel()
            if time.monotonic() > deadline:
                raise ContractError("joint-control activation timeout")
            stop.wait(.005)
        start = time.monotonic()
        with lock:
            shared["active"] = True
        if modes:
            modes.publish("relic")
        handback = False
        while time.monotonic() - start < duration and (not modes or time.monotonic() < session_end):
            check_error()
            with lock:
                last_emit = shared["last_emit"]
                motion_done = shared['motion_done']
            if modes and finish_requested():
                break
            if modes and modes.requested():
                handback = True
                break
            if motion_done:
                break
            if last_emit is None or time.monotonic() - last_emit > envelope.max_command_gap_s:
                raise ContractError("ReLIC command producer stalled")
            stop.wait(min(.005, envelope.max_command_gap_s / 3))
        check_error()
        if not modes and (core.phase != "policy" or core.policy_samples < 1):
            raise ContractError("standing trial ended before ReLIC handover")
        if (core.teleop and core.teleop.plan.study_commands is not None and
                (core.policy_samples != len(core.teleop.plan.study_commands) or core.pending is not None)):
            raise ContractError("registered trace ended early or final acknowledgement missing")
        if core.motion and not core.motion.completed:
            raise ContractError('walking distance not reached and settled before timeout')
        active_duration = time.monotonic() - start
        if handback:
            modes.publish("returning")
            # Continue feeding supported targets until the native command has
            # been accepted. Serialize the expected override with feedback polling.
            with feedback_lock:
                check_error()
                native_id = control.request_stand()
                shared["active"] = False
                segment_stop.set()
                control.future.cancel()
            if not producer_done.wait(reader.config.rpc_timeout_s):
                raise ContractError("previous joint command producer did not stop")
            class HandoffCancellation:
                def is_set(self):
                    return stop.is_set() or finish_requested()

            control.wait_standing(native_id, HandoffCancellation())
            with feedback_lock:
                shared["native_id"] = native_id
            check_error()
        return core, core.guard.key - first.last_command_key, active_duration, handback

    primary_error = None
    try:
        if interlock:
            interlock.check(reader.health())
        check_startup_cancel()
        control.acquire()
        stream_started = False
        if verify_enable:
            reader.start_stream(duration + 145)
            stream_started = True
        worker = threading.Thread(target=health_monitor, name="relic-health", daemon=True)
        worker.start()
        try:
            native_id = control.native_stand(stop)
            if modes:
                with feedback_lock:
                    shared["native_id"] = native_id
        except BaseException:
            # A health failure can cancel power/stand. Preserve its original
            # cause instead of replacing it with generic cancellation text.
            check_error()
            raise
        check_error()
        shared["standing"] = True
        session_end = time.monotonic() + duration
        commands_sent = samples = 0
        active_duration = 0.
        core = None
        while True:
            if modes:
                modes.publish("native")
                if verify_enable:
                    teleop_source.close()
                else:
                    teleop_source.reset_after_native_stand()
                enable_requested = False
                while time.monotonic() < session_end:
                    check_error()
                    if teleop_source.poll(time.monotonic()).finish:
                        break
                    if modes.requested():
                        if verify_enable:
                            modes.publish("checking")
                            try:
                                verify_enable()
                            except (ContractError, OSError, ValueError) as exc:
                                check_error()
                                teleop_source.close()
                                reason = str(exc) if isinstance(exc, ContractError) else f"ReLIC input unavailable ({type(exc).__name__})"
                                modes.publish("native", reason=reason)
                                continue
                            check_error()
                            if finish_requested() or time.monotonic() >= session_end:
                                break
                        enable_requested = True
                        break
                    stop.wait(.01)
                else:
                    break
                if not enable_requested or finish_requested():
                    break
                modes.publish("enabling")
            core, count, elapsed, handback = run_segment()
            commands_sent += count
            samples += core.policy_samples
            active_duration += elapsed
            if not handback:
                break
        check_error()
    except BaseException as exc:
        primary_error = exc
        record.event("first_failure", error_type=type(exc).__name__,
                     reason=str(exc) if isinstance(exc, ContractError) else "External operation failed")
        raise
    finally:
        stop.set()
        errors = []
        if modes:
            try:
                modes.publish("releasing")
            except Exception as exc:
                errors.append(exc)
        try:
            control.close()
        except BaseException as exc:
            errors.append(exc)
        shutdown = getattr(control, 'shutdown_result', None)
        if shutdown is not None:
            try:
                record.event('shutdown_result', **shutdown)
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
    return dict(commands=commands_sent,
                policy_samples=samples, command_gap_s=percentiles(gaps),
                command_ack_s=percentiles(acknowledgements), shutdown_confirmed=True,
                shutdown=shutdown,
                phase="stopped", duration_s=active_duration if modes or (core.motion or core.teleop) else duration,
                joystick=core.teleop.status if core and core.teleop else None,
                walking=core.motion.progress if core and core.motion else None)
