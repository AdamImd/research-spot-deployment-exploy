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
        self.guard = Guard(self.manifest, envelope)
        self.support = policy.support
        self.initial = self.active_at = self.target = self.last_command = None
        self.previous = np.zeros(12, dtype=np.float32)
        self.pending = None
        self.next_policy_at = None
        self.policy_at = None
        self.inference_s = 0.
        self.phase = "uninitialized"
        self.policy_samples = 0

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
        self.record.event("relic_initialized", state=state.model_dump(), height=height,
                          requested_height_m=self.policy.body_height,
                          previous_actions=self.previous.tolist(), endpoint=self.endpoint.tolist())

    def activate(self, now):
        if self.initial is None or self.active_at is not None or not np.isfinite(now):
            raise ContractError("invalid or repeated ReLIC activation")
        self.active_at = now
        self.phase = "direct_handover" if self.spec.kind == "direct" else "preparation"
        self.record.event("relic_phase", phase=self.phase, monotonic_s=now)

    def command(self, state, now, robot_now):
        if self.initial is None:
            raise ContractError("rollout was not initialized")
        self.guard.check_state(state, now, robot_now)
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
                if (np.max(np.abs(np.asarray(state.positions)[:12] - self.endpoint[:12]))
                        > self.spec.handover_pose_error_rad):
                    raise ContractError("ReLIC handover pose gate failed")
                if np.max(np.abs(state.velocities)) > self.spec.handover_joint_speed_rad_s:
                    raise ContractError("ReLIC handover velocity gate failed")
                self.next_policy_at = now
            new_action = None
            if now + 1e-10 >= self.next_policy_at and self.pending is None:
                target, inference_s, obs, action = self.policy.evaluate(state, self.previous)
                if inference_s > self.envelope.max_inference_s:
                    raise ContractError("ReLIC inference deadline exceeded")
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
                                  inference_s=inference_s, monotonic_s=now)
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
                if np.any(np.abs(delta) > self.spec.handover_torque_step_max):
                    raise ContractError("ReLIC handover torque-step gate failed")
                self.phase = "policy"
                self.record.event("relic_phase", phase=self.phase, monotonic_s=now)
        command = self.guard.command(target, state, now, robot_now, policy_at, inference_s,
                                     feedforward=feedforward)
        self.last_command = command
        if new_action is not None:
            self.pending = (command.key, new_action)
        return command
