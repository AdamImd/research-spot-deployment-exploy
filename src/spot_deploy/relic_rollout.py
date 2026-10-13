"""Deterministic ReLIC preparation/handover core; shared by simulator and live adapter.

No SDK imports, threads, clock reads or robot I/O. A command is not a measured
state. Previous-action history advances only after an emitted command is acknowledged.
"""

import numpy as np

from .contracts import ContractError
from .relic_policy import DEFAULT_Q
from .safety import Guard


class ReLICRollout:
    def __init__(self, policy, envelope, record):
        self.policy, self.manifest, self.envelope, self.record = (
            policy, policy.manifest, envelope, record)
        self.spec = self.manifest.relic.preparation
        if envelope.transition_s != self.spec.duration_s:
            raise ContractError("preparation duration and envelope transition differ")
        self.guard = Guard(self.manifest, envelope, event=record.event)
        # Startup qualification is immediate; persistence applies after activation.
        self.guard.physical_limits.enabled = False
        self.support = policy.support
        self.initial = self.active_at = self.target = self.last_command = None
        self.previous = np.zeros(12, dtype=np.float32)
        self.pending = None
        self.next_policy_at = None
        self.policy_at = None
        self.inference_s = 0.
        self.phase = "uninitialized"
        self.policy_samples = 0
        self.last_arm_motion_observation = None
        self.last_leg_position_observation = None
        self.motion = None
        if self.manifest.task == 'walking':
            from .walking import ForwardTravel
            self.motion = ForwardTravel(self.manifest.walking, record,
                                        physical_limits=self.guard.physical_limits)
            if self.manifest.walking.max_duration_s > envelope.max_duration_s:
                raise ContractError('walking timeout exceeds reviewed envelope')
        self.velocity_command = None

    def _operating_check(self, condition, message):
        if not condition:
            raise ContractError(message)

    def initialize(self, state, height, now, robot_now):
        if self.initial is not None:
            raise ContractError("rollout cannot be reinitialized")
        self.guard.check_state(state, now, robot_now)
        self.guard.key = state.last_command_key
        self.policy.reset()
        self.policy.initialize_height(height)
        self.initial = np.asarray(state.positions).copy()
        self.initial[12:] = self.manifest.arm_stowed_positions
        self.endpoint = (self.initial.copy() if self.spec.kind == "direct"
                         else DEFAULT_Q.astype(float).copy())
        self.endpoint[12:] = self.manifest.arm_stowed_positions
        self.phase = "supported_hold"
        if self.motion:
            self.motion.latch(state)
        self.record.event("relic_initialized", state=state.model_dump(), height=height,
                          requested_height_m=self.policy.body_height,
                          previous_actions=self.previous.tolist(), endpoint=self.endpoint.tolist())
        if self.envelope.command_rate_guard == 'observe':
            self.record.event('command_rate_mode', mode='observe',
                              reason=self.envelope.command_rate_reason,
                              measured_speed_and_torque_enforced=True)
        if self.guard.observe_arm_motion:
            self.record.event('arm_motion_mode', mode='observe',
                              reason=self.envelope.arm_motion_reason,
                              torque_limits_enforced=True, arm_targets_held=True)
        if self.guard.observe_leg_positions:
            self.record.event('leg_position_mode', mode='observe',
                              reason=self.envelope.leg_position_reason,
                              torque_profile=self.envelope.actuator_limit_profile,
                              command_bounds_enforced=True)

    def activate(self, now):
        if self.initial is None or self.active_at is not None or not np.isfinite(now):
            raise ContractError("invalid or repeated ReLIC activation")
        self.active_at = now
        limits = getattr(self.guard, 'physical_limits', None)
        if limits is not None:
            limits.enabled = True
        if self.envelope.physical_limit_persistence_s:
            self.record.event('physical_limit_persistence', monotonic_s=now,
                duration_s=self.envelope.physical_limit_persistence_s,
                reason=self.envelope.physical_limit_persistence_reason,
                semantics='continuous per-limit/per-joint; strict greater-than; reset on recovery')
        if self.motion:
            self.motion.activate(now)
        self.phase = "direct_handover" if self.spec.kind == "direct" else "preparation"
        self.record.event("relic_phase", phase=self.phase, monotonic_s=now)

    def command(self, state, now, robot_now):
        if self.initial is None:
            raise ContractError("rollout was not initialized")
        self.guard.check_state(state, now, robot_now)
        position_observation = self.guard.leg_position_observation
        if (position_observation is not None
                and position_observation != self.last_leg_position_observation):
            self.record.event('leg_position_observation', indices=position_observation,
                              positions=state.positions[:12],
                              reference_min=self.envelope.position_min[:12],
                              reference_max=self.envelope.position_max[:12], monotonic_s=now)
            self.last_leg_position_observation = position_observation.copy()
        if self.motion and self.active_at is not None:
            self.velocity_command = self.motion.update(state, now)
        observation = self.guard.arm_motion_observation
        if observation is not None and observation != self.last_arm_motion_observation:
            self.record.event('arm_motion_observation', **observation,
                              positions=state.positions[12:], velocities=state.velocities[12:],
                              monotonic_s=now)
            self.last_arm_motion_observation = observation.copy()
        if state.last_command_key > self.guard.key:
            raise ContractError("acknowledgement belongs to an unknown command")
        # Receiving an earlier tick from this action confirms that it reached the
        # command stream. This is receipt evidence, not proof of physical tracking.
        if self.pending is not None and state.last_command_key >= self.pending[0]:
            self.previous = self.pending[1].copy()
            self.record.event("policy_ack", command_key=self.pending[0],
                              state_key=state.last_command_key,
                              raw_actions=self.previous.tolist(), monotonic_s=now)
            self.pending = None
        elapsed = 0. if self.active_at is None else now - self.active_at
        if elapsed < 0:
            raise ContractError("rollout clock moved backwards")
        if self.manifest.task == 'three-leg-demo':
            self.policy.update_command(state, elapsed)
        if self.active_at is None or elapsed + 1e-10 < self.spec.duration_s:
            alpha = (np.clip(elapsed / self.spec.duration_s, 0., 1.)
                     if self.spec.duration_s else 0.)
            alpha = alpha ** 3 * (10 + alpha * (-15 + 6 * alpha))
            target = self.initial + alpha * (self.endpoint - self.initial)
            feedforward, support = self.support.compute(state)
            policy_at, inference_s = min(now, state.received_monotonic_s), 0.
            self.record.event("preparation", phase=self.phase, monotonic_s=now,
                              reference=target.tolist(), feedforward=feedforward.tolist(),
                              support=support)
            new_action = None
        else:
            handover = self.phase != "policy"
            if handover:
                self._operating_check(
                    np.max(np.abs(np.asarray(state.positions)[:12] - self.endpoint[:12]))
                    <= self.spec.handover_pose_error_rad, "ReLIC handover pose gate failed")
                self._operating_check(
                    np.max(np.abs(state.velocities[:self.guard.motion_joint_count]))
                    <= self.spec.handover_joint_speed_rad_s, "ReLIC handover velocity gate failed")
                self.next_policy_at = now
            new_action = None
            if now + 1e-10 >= self.next_policy_at and self.pending is None:
                if self.motion:
                    target, inference_s, obs, action = self.policy.evaluate(
                        state, self.previous, velocity_command=self.velocity_command)
                else:
                    target, inference_s, obs, action = self.policy.evaluate(state, self.previous)
                self._operating_check(inference_s <= self.envelope.max_inference_s,
                                      "ReLIC inference deadline exceeded")
                self.target, self.inference_s = target.copy(), inference_s
                self.policy_at = min(now, state.received_monotonic_s)
                # Skip missed periods, never issue catch-up bursts.
                period = 1 / self.manifest.policy_hz
                self.next_policy_at += period
                if self.next_policy_at <= now:
                    self.next_policy_at = now + period
                self.policy_samples += 1
                new_action = action.copy()
                self.record.event("policy", state=state.model_dump(), observations=obs[0].tolist(),
                                  raw_actions=action.tolist(), targets=target.tolist(),
                                  inference_s=inference_s, monotonic_s=now,
                                  velocity_command=self.velocity_command or [0., 0., 0.])
                if self.manifest.task == 'three-leg-demo':
                    self.record.event('demo_prediction', state=state.model_dump(),
                                      monotonic_s=now, targets=target.tolist(),
                                      raw_actions=action.tolist(), **self.policy.demo_record(state))
            target = self.target
            feedforward = np.zeros(19)
            policy_at, inference_s = self.policy_at, self.inference_s
            if handover:
                # Compare commanded torques at one identical measured state;
                # damping cancels, so this measures the command discontinuity.
                if self.last_command is None:
                    raise ContractError("handover requires a preceding supported command")
                delta = (np.asarray(self.manifest.gains.kp)
                         * (target - self.last_command.positions)
                         - self.last_command.feedforward)
                self.record.event("relic_handover", torque_step_Nm=delta.tolist(),
                                  previous_actions=self.previous.tolist(), monotonic_s=now)
                self._operating_check(np.all(np.abs(delta) <= self.spec.handover_torque_step_max),
                                      "ReLIC handover torque-step gate failed")
                self.phase = "policy"
                self.record.event("relic_phase", phase=self.phase, monotonic_s=now)
        command = self.guard.command(target, state, now, robot_now, policy_at, inference_s,
                                     feedforward=feedforward)
        rate_observation = getattr(self.guard, 'command_rate_observation', None)
        if rate_observation is not None:
            self.record.event('command_rate_observation',
                              **rate_observation, monotonic_s=now)
        self.last_command = command
        if new_action is not None:
            self.pending = (command.key, new_action)
        return command
