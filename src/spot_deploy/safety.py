"""Checks shared by replay, shadow and the independently scheduled command loop."""

from dataclasses import dataclass
from threading import Condition
from time import monotonic

import numpy as np

from .contracts import ContractError, Envelope, Manifest, State
from .policy import rotation
from .physical_limits import PhysicalLimits


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
    def __init__(self, manifest: Manifest, envelope: Envelope, *, event=None):
        self.manifest = manifest
        self.envelope = envelope
        self.physical_limits = PhysicalLimits(envelope.physical_limit_persistence_s, event)
        self.last_state_time = None
        self.last_received = None
        self.last_command = None
        self.last_sent = None
        self.key = 0
        self.command_rate_observation = None
        if (envelope.command_rate_guard == "observe" and envelope.scope != "simulation"
                and (getattr(manifest, "adapter", None) != "relic-exploy-leg"
                     or manifest.task != "three-leg-demo")):
            raise ContractError("command rate observation requires the selected-leg demo")
        self.mobile_arm = manifest.task == 'joystick-arm'
        self.arm_reference = np.asarray(manifest.arm_stowed_positions, dtype=float)
        self.observe_arm_tracking = envelope.arm_tracking_guard == 'observe'
        if self.observe_arm_tracking and not self.mobile_arm:
            raise ContractError('arm tracking observation requires joystick-arm task')
        self.arm_tracking_observation = None
        if self.mobile_arm and envelope.arm_motion_guard != 'enforce':
            raise ContractError('joystick arm control requires enforced arm motion limits')
        self.observe_arm_motion = envelope.arm_motion_guard == "observe"
        if (self.observe_arm_motion
                and getattr(manifest, "adapter", None) not in ("relic84", "relic-exploy", "relic-exploy-leg")):
            raise ContractError("arm observation mode requires a ReLIC leg controller")
        self.motion_joint_count = 12 if self.observe_arm_motion else 19
        self.tracking_joint_count = 12 if self.observe_arm_tracking else self.motion_joint_count
        self.arm_motion_observation = None
        self.observe_leg_positions = envelope.leg_position_guard == "observe"
        self.leg_position_observation = None
        if (self.observe_leg_positions
                and getattr(manifest, "adapter", None) not in ("relic84", "relic-exploy", "relic-exploy-leg")):
            raise ContractError("leg position observation requires a ReLIC controller")
        if envelope.transition_s == 0:
            if (getattr(manifest, "adapter", None) not in ("relic84", "relic-exploy", "relic-exploy-leg")
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
        if np.any(arm < envelope.command_lower[12:]) or np.any(arm > envelope.command_upper[12:]):
            raise ContractError("arm stow target exceeds command position envelope")

        if self.mobile_arm:
            from .joystick_teleop import ArmPath
            path = ArmPath(manifest.arm_stowed_positions, manifest.teleop)
            points = path.points
            if (np.any(points < np.array(envelope.position_min)[12:])
                    or np.any(points > np.array(envelope.position_max)[12:])
                    or np.any(points < np.array(envelope.command_lower)[12:])
                    or np.any(points > np.array(envelope.command_upper)[12:])):
                raise ContractError('arm path exceeds reviewed position envelope')
            if np.any(np.array(envelope.command_rate_max)[12:18] < manifest.teleop.arm_speed_rad_s):
                raise ContractError('arm path speed exceeds reviewed target slew envelope')

    def set_arm_reference(self, reference):
        if not self.mobile_arm:
            raise ContractError('dynamic arm reference requires joystick-arm task')
        arm = np.asarray(reference, dtype=float)
        if arm.shape != (7,) or not np.isfinite(arm).all():
            raise ContractError('invalid arm command reference')
        self.arm_reference = arm.copy()

    def check_actuator_load(self, positions, loads, context, now, *, requested=False):
        if self.envelope.actuator_limit_profile:
            from .actuator_limits import COUPLED_MOTOR_TORQUE, KNEE_INDICES, KNEE_TABLE, torque_limits

            q, torque = np.asarray(positions, dtype=float), np.asarray(loads, dtype=float)
            if q.shape != (19,) or torque.shape != (19,) or not np.isfinite([q, torque]).all():
                raise ContractError(f'{context}: manufacturer torque check requires finite joint data')
            k = list(KNEE_INDICES)
            outside = (q[k] < KNEE_TABLE[0, 0]) | (q[k] > KNEE_TABLE[-1, 0])
            self.physical_limits.check(outside, now, 'knee position outside manufacturer torque table',
                                       key='measured knee table domain', values=q[k])
            # During the explicitly requested persistence window use the nearest
            # endpoint's torque cap. Never extrapolate an unreviewed torque curve.
            bounded_q = q.copy()
            bounded_q[k] = np.clip(q[k], KNEE_TABLE[0, 0], KNEE_TABLE[-1, 0])
            limits = torque_limits(bounded_q)
            if requested and self.envelope.leg_requested_torque_max is not None:
                limits[:12] = self.envelope.leg_requested_torque_max
            self.physical_limits.check(np.abs(torque) > limits, now,
                f'{context}: manufacturer joint torque limit', values=torque)
            coupling = abs(torque[13]-torque[14])
            self.physical_limits.check(coupling > COUPLED_MOTOR_TORQUE, now,
                f'{context}: manufacturer coupled SH1/EL0 motor torque limit', values=coupling)

    def check_state(self, state: State, now: float, robot_now: float):
        env = self.envelope
        if not np.isfinite([*state.positions, *state.velocities, *state.loads,
                *state.odom_quaternion_wxyz, *state.linear_velocity_odom,
                *state.angular_velocity_odom, state.received_monotonic_s,
                state.robot_time_s, now, robot_now]).all():
            raise ContractError('nonfinite state or clock')
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
        position_bad = (q[:n] < env.position_min[:n]) | (q[:n] > env.position_max[:n])
        if self.observe_leg_positions:
            self.leg_position_observation = np.flatnonzero(position_bad[:12]).tolist()
            position_bad[:12] = False
        self.physical_limits.check(position_bad, now, 'measured joint position limit', values=q[:n])
        self.physical_limits.check(np.abs(v[:n]) > env.velocity_max[:n], now,
            'measured joint velocity or load limit', key='measured joint velocity', values=v[:n])
        self.physical_limits.check(np.abs(load) > env.load_max, now,
            'measured joint velocity or load limit', key='measured joint load', values=load)
        self.check_actuator_load(q, load, "measured load", now)
        r = rotation(state)
        roll = np.arctan2(r[2, 1], r[2, 2])
        pitch = np.arcsin(np.clip(-r[2, 0], -1, 1))
        self.physical_limits.check([abs(roll) > env.max_roll_rad, abs(pitch) > env.max_pitch_rad],
                                   now, 'body attitude limit', values=[roll, pitch])
        speeds = [np.linalg.norm(state.linear_velocity_odom), np.linalg.norm(state.angular_velocity_odom)]
        self.physical_limits.check(np.array(speeds) > [env.max_linear_speed, env.max_angular_speed],
                                   now, 'body speed limit', values=speeds)
        if self.observe_arm_tracking:
            self.arm_tracking_observation = dict(
                reference_indices=np.flatnonzero(np.abs(q[12:] - self.arm_reference)
                    > env.max_arm_pose_error).tolist(), measured_tracking_indices=[],
                command_tracking_indices=[])
            if self.last_command is not None:
                self.arm_tracking_observation['measured_tracking_indices'] = np.flatnonzero(
                    np.abs(q[12:] - self.last_command[12:])
                    > np.asarray(env.tracking_error_max)[12:]).tolist()
        if not self.observe_arm_motion and not self.observe_arm_tracking:
            arm_error = np.abs(q[12:] - self.arm_reference)
            self.physical_limits.check(arm_error > env.max_arm_pose_error, now,
                                       ('arm is outside commanded pose' if self.mobile_arm else
                                        'arm is outside registered stowed pose'), values=arm_error)
        if self.last_command is not None:
            n = self.tracking_joint_count
            error = np.abs(q[:n] - self.last_command[:n])
            self.physical_limits.check(error > env.tracking_error_max[:n], now,
                                       'measured joint tracking error', values=error)

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
        self.physical_limits.check((target < env.command_lower) | (target > env.command_upper),
                                   now, 'command position limit', values=target)
        n = self.tracking_joint_count
        error = np.abs(target[:n] - state.positions[:n])
        self.physical_limits.check(error > env.tracking_error_max[:n], now,
                                   'command tracking error limit', values=error)
        if self.observe_arm_tracking:
            self.arm_tracking_observation['command_tracking_indices'] = np.flatnonzero(
                np.abs(target[12:] - np.asarray(state.positions)[12:])
                > np.asarray(env.tracking_error_max)[12:]).tolist()
        feedforward = np.asarray(self.manifest.gains.feedforward if feedforward is None
                                 else feedforward, dtype=float)
        if feedforward.shape != (19,) or not np.isfinite(feedforward).all():
            raise ContractError("command feedforward exceeds envelope")
        self.physical_limits.check(np.abs(feedforward) > env.load_max, now,
                                   'command feedforward exceeds envelope', values=feedforward)
        torque = (
            np.asarray(self.manifest.gains.kp) * (target - state.positions)
            - np.asarray(self.manifest.gains.kd) * state.velocities
            + feedforward
        )
        requested_limits = np.array(env.load_max)
        if env.leg_requested_torque_max is not None:
            requested_limits[:12] = env.leg_requested_torque_max
        if not np.isfinite(torque).all():
            raise ContractError('nonfinite predicted PD torque')
        self.physical_limits.check(np.abs(torque) > requested_limits, now,
                                   'predicted PD-plus-feedforward load limit', values=torque)
        self.check_actuator_load(state.positions, feedforward, "feedforward", now)
        self.check_actuator_load(state.positions, torque, "predicted PD-plus-feedforward load", now, requested=True)
        if not np.allclose(target[12:], self.arm_reference, rtol=0, atol=1e-9):
            raise ContractError("leg-only policy attempted arm movement")
        if self.last_sent is not None:
            dt = now - self.last_sent
            if dt <= 0 or dt > env.max_command_gap_s:
                raise ContractError("command scheduling deadline exceeded")
            rates = (target - self.last_command) / dt
            exceeded = np.flatnonzero(np.abs(rates) > np.asarray(env.command_rate_max))
            self.command_rate_observation = None
            if env.command_rate_guard == "enforce":
                self.physical_limits.check(np.abs(rates) > np.asarray(env.command_rate_max), now,
                                           'command slew limit', values=rates)
            if exceeded.size:
                self.command_rate_observation = dict(indices=exceeded.tolist(),
                    target_rates_rad_s=rates.tolist(), reference_max=env.command_rate_max)
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
