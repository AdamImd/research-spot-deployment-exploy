"""Offline replay and read-only shadow inference. No command adapter imports."""

import time

import numpy as np

from .contracts import ContractError, State
from .safety import Guard, Schedule, percentiles


def replay(states_path, policy, envelope, record):
    guard = Guard(policy.manifest, envelope) if envelope else None
    inference, gaps = [], []
    previous = None
    count = 0
    with states_path.open() as stream:
        for line in stream:
            state = State.model_validate_json(line)
            if previous is not None:
                gap = state.robot_time_s - previous
                if gap <= 0:
                    raise ContractError("replay states must be strictly increasing")
                gaps.append(gap)
            previous = state.robot_time_s
            targets, elapsed, obs, action = policy.predict(state)
            inference.append(elapsed)
            if guard:
                guard.command(
                    targets,
                    state,
                    state.received_monotonic_s,
                    state.robot_time_s,
                    state.received_monotonic_s,
                    elapsed,
                )
            record.event(
                "replay_sample",
                state=state.model_dump(),
                observations=obs[0].tolist(),
                raw_actions=action.tolist(),
                targets=targets.tolist(),
                inference_s=elapsed,
            )
            count += 1
    if not count:
        raise ContractError("empty replay")
    return {
        "samples": count,
        "inference_s": percentiles(inference),
        "state_gap_s": percentiles(gaps),
        "hardware_access": False,
        "qualification": False,
        "note": "Replay checks sample-by-sample parity, not real-time scheduling or hardware stability.",
    }


def shadow(reader, policy, envelope, duration, record):
    guard = Guard(policy.manifest, envelope)
    first = reader.start_stream(duration)
    guard.check_state(first, time.monotonic(), reader.robot_now())
    start = time.monotonic()
    schedule = Schedule(policy.manifest.policy_hz, start)
    latency, ages, gaps, loop = [], [], [], []
    previous, previous_tick = None, None
    while time.monotonic() - start < duration:
        state = reader.mailbox.get()
        now = time.monotonic()
        guard.check_state(state, now, reader.robot_now())
        if previous is not None:
            gaps.append(state.robot_time_s - previous)
            loop.append(now - previous_tick)
        previous, previous_tick = state.robot_time_s, now
        targets, elapsed, obs, action = policy.predict(state)
        if elapsed > envelope.max_inference_s:
            raise ContractError("shadow inference deadline exceeded")
        if np.any(targets < envelope.position_min) or np.any(targets > envelope.position_max):
            raise ContractError("shadow target exceeds position bounds")
        n = guard.motion_joint_count
        if np.any(np.abs(targets[:n] - state.positions[:n]) > envelope.tracking_error_max[:n]):
            raise ContractError("shadow target tracking bound exceeded")
        latency.append(elapsed)
        ages.append(reader.robot_now() - state.robot_time_s)
        record.event(
            "shadow_sample",
            state=state.model_dump(),
            observations=obs[0].tolist(),
            raw_actions=action.tolist(),
            targets=targets.tolist(),
            inference_s=elapsed,
        )
        time.sleep(schedule.advance(time.monotonic()))
    stream_stats = reader.stream_statistics()
    for name in ("acquisition_gap_s", "receive_gap_s"):
        if stream_stats[name] and stream_stats[name]["max"] > envelope.max_state_gap_s:
            raise ContractError("received stream gap exceeded envelope")
    return {
        "samples": len(latency),
        "stream": stream_stats,
        "inference_s": percentiles(latency),
        "state_age_s": percentiles(ages),
        "sample_gap_s": percentiles(gaps),
        "policy_loop_s": percentiles(loop),
        "motion_commands": 0,
        "command_network_latency_s": None,
        "note": "Shadow cannot measure robot command acknowledgement latency or qualify stability.",
    }


def watch(reader, duration, record, state_source="stream"):
    """Bounded read-only telemetry, including stopped/faulted robots; no policy needed."""
    if state_source not in ("stream", "poll"):
        raise ContractError("unknown watch state source")
    if state_source == "stream":
        reader.start_stream(duration)
    start = time.monotonic()
    schedule = Schedule(20, start)
    last_health = -float("inf")
    samples = 0
    previous = None
    gaps, polls = [], []
    while time.monotonic() - start < duration:
        before = time.monotonic()
        state = reader.read_state() if state_source == "poll" else reader.mailbox.get()
        now = time.monotonic()
        if now - state.received_monotonic_s > 0.5:
            raise ContractError("watch state stream stale")
        if state_source == "poll":
            if abs(reader.robot_now() - state.robot_time_s) > 0.5:
                raise ContractError("watch polled state timestamp stale")
            if previous is not None:
                if state.robot_time_s <= previous:
                    raise ContractError("watch polled state timestamp did not advance")
                gaps.append(state.robot_time_s - previous)
            previous = state.robot_time_s
            polls.append(now - before)
        record.event("state", state=state.model_dump())
        if now - last_health >= 1:
            record.event("health", **reader.health())
            last_health = now
        samples += 1
        time.sleep(schedule.advance(time.monotonic()))
    return {
        "samples": samples,
        "motion_commands": 0,
        "control_lease_acquired": False,
        "state_source": state_source,
        "stream": reader.stream_statistics() if state_source == "stream" else None,
        "poll_rpc_s": percentiles(polls),
        "acquisition_gap_s": percentiles(gaps),
    }
