"""Checks shared by replay, shadow and the independently scheduled command loop."""

from dataclasses import dataclass
from threading import Condition
from time import monotonic

import numpy as np

from .contracts import ContractError, Envelope, Manifest, State
from .policy import rotation


class LatestState:
    """Single-slot mailbox; no backlog, with errors propagated to every consumer."""

    def __init__(self):
        self._condition = Condition()
        self._state = None
        self._error = None
        self._closed = False

    def publish(self, state: State):
        with self._condition:
            if self._state and state.robot_time_s <= self._state.robot_time_s:
                self._error = ContractError("duplicate or reordered state timestamp")
                self._condition.notify_all()
                raise self._error
            self._state = state
            self._condition.notify_all()

    def fail(self, error):
        with self._condition:
            self._error = error
            self._condition.notify_all()

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def get(self, timeout_s=0):
        with self._condition:
            self._condition.wait_for(
                lambda: self._state is not None or self._error or self._closed, timeout_s
            )
            if self._error:
                raise ContractError("state stream failed") from self._error
            if self._closed:
                raise ContractError("state stream closed")
            if self._state is None:
                raise ContractError("state stream startup timeout")
            return self._state


@dataclass(frozen=True)
class Command:
    positions: tuple[float, ...]
    end_robot_time_s: float
    key: int
    feedforward: tuple[float, ...] | None = None


class Guard:
    def __init__(self, manifest: Manifest, envelope: Envelope):
        self.manifest = manifest
        self.envelope = envelope
        self.last_state_time = None
        self.last_received = None
        self.last_command = None
        self.last_sent = None
        self.key = 0
        self.observe_arm_motion = envelope.arm_motion_guard == "observe"
        if (self.observe_arm_motion
                and getattr(manifest, "adapter", None) not in ("relic84", "relic-exploy")):
            raise ContractError("arm observation mode requires a ReLIC leg controller")
        self.motion_joint_count = 12 if self.observe_arm_motion else 19
        self.arm_motion_observation = None
        if envelope.transition_s == 0:
            if (getattr(manifest, "adapter", None) not in ("relic84", "relic-exploy")
                    or manifest.relic.preparation.kind != "direct"
                    or manifest.relic.preparation.duration_s != 0):
                raise ContractError("zero transition requires the explicit ReLIC direct-start contract")
        if 1 / manifest.policy_hz >= envelope.max_policy_age_s:
            raise ContractError("policy period exceeds its freshness limit")
        if 1 / manifest.stream_hz >= envelope.max_command_gap_s:
            raise ContractError("stream period exceeds command gap limit")
        if np.any(np.abs(manifest.gains.feedforward) > envelope.load_max):
            raise ContractError("feedforward exceeds envelope")
        arm = np.array(manifest.arm_stowed_positions)
        if np.any(arm < envelope.position_min[12:]) or np.any(arm > envelope.position_max[12:]):
            raise ContractError("arm stow target exceeds position envelope")

    def check_actuator_load(self, positions, loads, context):
        if self.envelope.actuator_limit_profile:
            from .actuator_limits import check_torque

            try:
                check_torque(positions, loads)
            except ValueError as exc:
                raise ContractError(f"{context}: {exc}") from exc

    def check_state(self, state: State, now: float, robot_now: float):
        env = self.envelope
        if (
            now < state.received_monotonic_s
            or now - state.received_monotonic_s > env.max_state_age_s
        ):
            raise ContractError("stale local state")
        age = robot_now - state.robot_time_s
        if age < -env.max_future_skew_s or age > env.max_state_age_s:
            raise ContractError("stale or future robot timestamp")
        if self.last_state_time is not None:
            delta = state.robot_time_s - self.last_state_time
            # Reusing the latest sample at a faster command tick is allowed.
            if delta < 0 or delta > env.max_state_gap_s:
                raise ContractError("state timestamp order or gap violation")
            if delta == 0 and state.received_monotonic_s != self.last_received:
                raise ContractError("replayed state timestamp")
        self.last_state_time = state.robot_time_s
        self.last_received = state.received_monotonic_s
        q, v, load = map(np.asarray, (state.positions, state.velocities, state.loads))
        n = self.motion_joint_count
        if self.observe_arm_motion:
            observation = dict(
                position_indices=np.flatnonzero((q[12:] < np.asarray(env.position_min)[12:])
                    | (q[12:] > np.asarray(env.position_max)[12:])).tolist(),
                velocity_indices=np.flatnonzero(np.abs(v[12:])
                    > np.asarray(env.velocity_max)[12:]).tolist(),
                stow_displaced=bool(np.max(np.abs(q[12:] - self.manifest.arm_stowed_positions))
                    > env.max_arm_pose_error),
                tracking_indices=[])
            if self.last_command is not None:
                observation['tracking_indices'] = np.flatnonzero(np.abs(q[12:]
                    - self.last_command[12:]) > np.asarray(env.tracking_error_max)[12:]).tolist()
            self.arm_motion_observation = observation
        if np.any(q[:n] < env.position_min[:n]) or np.any(q[:n] > env.position_max[:n]):
            raise ContractError("measured joint position limit")
        if np.any(np.abs(v[:n]) > env.velocity_max[:n]) or np.any(np.abs(load) > env.load_max):
            raise ContractError("measured joint velocity or load limit")
        self.check_actuator_load(q, load, "measured load")
        r = rotation(state)
        roll = np.arctan2(r[2, 1], r[2, 2])
        pitch = np.arcsin(np.clip(-r[2, 0], -1, 1))
        if abs(roll) > env.max_roll_rad or abs(pitch) > env.max_pitch_rad:
            raise ContractError("body attitude limit")
        if (
            np.linalg.norm(state.linear_velocity_odom) > env.max_linear_speed
            or np.linalg.norm(state.angular_velocity_odom) > env.max_angular_speed
        ):
            raise ContractError("body speed limit")
        if (not self.observe_arm_motion
                and np.max(np.abs(q[12:] - self.manifest.arm_stowed_positions)) > env.max_arm_pose_error):
            raise ContractError("arm is outside registered stowed pose")
        if self.last_command is not None:
            if np.any(np.abs(q[:n] - self.last_command[:n]) > env.tracking_error_max[:n]):
                raise ContractError("measured joint tracking error")

    def command(self, targets, state, now, robot_now, policy_at, inference_s, feedforward=None):
        self.check_state(state, now, robot_now)
        env = self.envelope
        if not 0 <= now - policy_at <= env.max_policy_age_s:
            raise ContractError("stale policy result")
        if not np.isfinite(inference_s) or inference_s < 0 or inference_s > env.max_inference_s:
            raise ContractError("inference deadline exceeded")
        target = np.asarray(targets)
        if target.shape != (19,) or not np.isfinite(target).all():
            raise ContractError("command must contain nineteen finite positions")
        if np.any(target < env.position_min) or np.any(target > env.position_max):
            raise ContractError("command position limit")
        n = self.motion_joint_count
        if np.any(np.abs(target[:n] - state.positions[:n]) > env.tracking_error_max[:n]):
            raise ContractError("command tracking error limit")
        feedforward = np.asarray(self.manifest.gains.feedforward if feedforward is None
                                 else feedforward, dtype=float)
        if (feedforward.shape != (19,) or not np.isfinite(feedforward).all()
                or np.any(np.abs(feedforward) > env.load_max)):
            raise ContractError("command feedforward exceeds envelope")
        torque = (
            np.asarray(self.manifest.gains.kp) * (target - state.positions)
            - np.asarray(self.manifest.gains.kd) * state.velocities
            + feedforward
        )
        if np.any(np.abs(torque) > env.load_max):
            raise ContractError("predicted PD-plus-feedforward load limit")
        self.check_actuator_load(state.positions, feedforward, "feedforward")
        self.check_actuator_load(state.positions, torque, "predicted PD-plus-feedforward load")
        if not np.allclose(target[12:], self.manifest.arm_stowed_positions, rtol=0, atol=1e-9):
            raise ContractError("leg-only policy attempted arm movement")
        if self.last_sent is not None:
            dt = now - self.last_sent
            if dt <= 0 or dt > env.max_command_gap_s:
                raise ContractError("command scheduling deadline exceeded")
            if np.any(np.abs(target - self.last_command) > np.asarray(env.command_rate_max) * dt):
                raise ContractError("command slew limit")
        self.last_command = target.copy()
        self.last_sent = now
        self.key += 1
        # Expiry is capped by sample age as well as send time; stale data cannot be renewed forever.
        expiry = min(robot_now + env.command_ttl_s, state.robot_time_s + env.max_state_age_s)
        if expiry <= robot_now:
            raise ContractError("command already expired")
        return Command(tuple(float(v) for v in target), expiry, self.key,
                       tuple(float(v) for v in feedforward))


def bounded_join(thread, timeout_s):
    thread.join(timeout_s)
    if thread.is_alive():
        raise ContractError("worker failed to stop within shutdown deadline")


def percentiles(values):
    if not values:
        return None
    array = np.asarray(values)
    return {
        "count": len(values),
        "min": float(array.min()),
        "p50": float(np.quantile(array, 0.5)),
        "p95": float(np.quantile(array, 0.95)),
        "p99": float(np.quantile(array, 0.99)),
        "max": float(array.max()),
    }


class Schedule:
    """Independent monotonic deadlines; skip catch-up bursts after an overrun."""

    def __init__(self, hz, start=None):
        self.period = 1 / hz
        self.next = monotonic() if start is None else start

    def advance(self, now):
        self.next += self.period
        if self.next <= now:
            self.next = now + self.period
        return max(0.0, self.next - now)
