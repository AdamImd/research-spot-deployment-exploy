"""Explicit offline-only, unguarded operating-envelope diagnostic.

Retain finite data, action/ACK semantics, held-arm targets and model physics.
There is no corresponding option on the hardware command interface.
"""

from time import monotonic

import numpy as np

from .contracts import JOINTS, ContractError
from .relic_rollout import ReLICRollout
from .safety import Command, Guard
from .walking import ForwardTravel


class DiagnosticChecks:
    def _operating_check(self, condition, message):
        if not condition and message not in self.violations:
            self.violations.add(message)
            self.record.event('disabled_operating_check', reason=message)


class DiagnosticTravel(DiagnosticChecks, ForwardTravel):
    def _physical_check(self, violations, now, message):
        self._operating_check(not np.any(violations), message)

    def __init__(self, plan, record, forward_speed=None, speed_profile='taper'):
        super().__init__(plan, record)
        self.violations = set()
        if speed_profile not in ('taper', 'constant'):
            raise ContractError('unknown simulation speed profile')
        if forward_speed is not None and (not np.isfinite(forward_speed) or not 0 < forward_speed <= 1):
            raise ContractError('simulation forward speed must be finite and within (0,1] m/s')
        if speed_profile != 'taper' and forward_speed is None:
            raise ContractError('constant simulation profile requires an explicit speed')
        self.forward_speed, self.speed_profile = forward_speed, speed_profile
        if forward_speed is not None:
            record.event('simulation_speed_override', forward_speed_m_s=forward_speed,
                         profile=speed_profile, manifest_unchanged=True, hardware_access=False)

    def _desired_speed(self, remaining):
        if self.forward_speed is None:
            return super()._desired_speed(remaining)
        if self.speed_profile == 'constant':
            return self.forward_speed if remaining > 0 else 0.
        return min(self.forward_speed, max(0., remaining) * .8)


class SimulationVelocityPolicy:
    """Offline Exploy wrapper with a separate bound on the velocity input only."""
    def __init__(self, policy, maximum, envelope):
        from .exploy_policy import ExployReLICPolicy
        if envelope.scope != 'simulation' or not isinstance(policy, ExployReLICPolicy):
            raise ContractError('speed-input override requires simulation scope and Exploy policy')
        if policy.manifest.task != 'walking' or not np.isfinite(maximum) or not 0 < maximum <= 1:
            raise ContractError('invalid simulation walking speed override')
        self.policy, self.maximum = policy, maximum

    def __getattr__(self, name):
        return getattr(self.policy, name)

    def evaluate(self, state, previous, velocity_command=None):
        from .relic_policy import finite
        start = monotonic()
        velocity = finite([0., 0., 0.] if velocity_command is None else velocity_command, 3)
        if (velocity[1] != 0 or velocity[2] != 0
                or not 0 <= velocity[0] <= self.maximum+1e-7):
            raise ContractError('velocity outside explicit simulation forward override')
        # Preserve every normal state/height/history transform and output check.
        # The base adapter still validates its original manifest for live calls.
        inputs = self.policy.inputs(state, previous, [0., 0., 0.])
        inputs['velocity_command'] = velocity[None]
        return self.policy._evaluate_inputs(inputs, start)


class DiagnosticCommands:
    def __init__(self, manifest, envelope, record):
        if envelope.scope != 'simulation':
            raise ContractError('unguarded diagnostics require simulation scope')
        self.manifest, self.envelope, self.record = manifest, envelope, record
        self.reference = Guard(manifest, envelope)
        self.violations = set()
        self.key = 0
        self.observe_arm_motion = False
        self.observe_leg_positions = False
        self.leg_position_observation = None
        self.motion_joint_count = 19
        self.arm_motion_observation = None

    def check_state(self, state, now, robot_now):
        # model_copy() can bypass Pydantic validation; explicitly retain integrity.
        values = [*state.positions, *state.velocities, *state.loads,
                  *state.odom_quaternion_wxyz, *state.linear_velocity_odom,
                  *state.angular_velocity_odom, now, robot_now]
        if not np.isfinite(values).all():
            raise ContractError('nonfinite simulation state')
        try:
            self.reference.check_state(state, now, robot_now)
        except ContractError as exc:
            self._record(str(exc), state, now)

    def _record(self, reason, state, now):
        if reason not in self.violations:
            self.violations.add(reason)
            self.record.event('disabled_guard', reason=reason, state=state.model_dump(),
                              monotonic_s=now)

    def command(self, targets, state, now, robot_now, policy_at, inference_s, feedforward=None):
        self.check_state(state, now, robot_now)
        target = np.asarray(targets, dtype=float)
        ff = np.asarray(self.manifest.gains.feedforward if feedforward is None else feedforward)
        if (target.shape != (19,) or ff.shape != (19,) or not np.isfinite(target).all()
                or not np.isfinite(ff).all() or not np.isfinite([policy_at, inference_s]).all()):
            raise ContractError('nonfinite or malformed simulation command')
        if not np.allclose(target[12:], self.manifest.arm_stowed_positions, rtol=0, atol=1e-9):
            raise ContractError('leg-only simulation attempted arm movement')
        try:
            self.reference.command(target, state, now, robot_now, policy_at, inference_s, ff)
        except ContractError as exc:
            self._record(str(exc), state, now)
        # Keep the reference's preceding target/time even if its first check rejected.
        self.reference.last_command, self.reference.last_sent = target.copy(), now
        self.key += 1
        return Command(tuple(target), robot_now + self.envelope.command_ttl_s, self.key, tuple(ff))


class SimulationDiagnosticRollout(DiagnosticChecks, ReLICRollout):
    def __init__(self, policy, envelope, record, forward_speed=None, speed_profile='taper'):
        if envelope.scope != 'simulation':
            raise ContractError('unguarded diagnostics require simulation scope')
        if policy.manifest.relic.preparation.kind != 'direct':
            raise ContractError('unguarded diagnostic supports direct startup only')
        super().__init__(policy, envelope, record)
        self.violations = set()
        self.guard = DiagnosticCommands(self.manifest, envelope, record)
        if (forward_speed is not None or speed_profile != 'taper') and not self.motion:
            raise ContractError('simulation speed override requires a walking task')
        if self.motion:
            self.motion = DiagnosticTravel(self.manifest.walking, record, forward_speed, speed_profile)
        record.event('unguarded_simulation', hardware_access=False, hardware_qualified=False,
                     retained='finite data, parity, held arm, ACK/history, physical actuator saturation')


def demand_summary(rows, config):
    """Observed demand, not safe limits. Applied torque precedes the sampled command."""
    names = [n.replace('arm_', 'arm0_') for n in config['simulation_joint_names']]
    ids = [names.index(n) for n in JOINTS]
    t = np.array([r['trial_time'] for r in rows])
    q = np.array([r['q'] for r in rows])[:, ids]
    v = np.array([r['dq'] for r in rows])[:, ids]
    target = np.array([r['positions'] for r in rows])
    error = target - q
    requested = np.array([r['pd_torque_Nm'] for r in rows])
    applied = np.array([r['preceding_applied_torque_Nm'] for r in rows])
    env = config['envelope']
    fallen = np.array([r['height'] < .25 or r['tilt'] > .9 for r in rows])
    first_fall = int(np.flatnonzero(fallen)[0]) if fallen.any() else len(rows)
    def window(end):
        if end == 0:
            return None
        result = {}
        slew = np.zeros_like(target)
        if len(rows) > 1:
            slew[1:] = np.diff(target, axis=0) / np.diff(t)[:, None]
        for j, name in enumerate(JOINTS):
            metrics = {}
            for label, values, bound in (
                    ('speed_rad_s', v[:, j], env['velocity_max'][j]),
                    ('target_error_rad', error[:, j], env['tracking_error_max'][j]),
                    ('requested_PD_Nm', requested[:, j],
                     env.get('leg_requested_torque_max') if j < 12 and env.get('leg_requested_torque_max')
                     is not None else env['load_max'][j]),
                    ('preceding_applied_Nm', applied[:, j], env['load_max'][j]),
                    ('target_slew_rad_s', slew[:, j], env['command_rate_max'][j])):
                peak = int(np.argmax(np.abs(values[:end])))
                crossings = np.flatnonzero(np.abs(values[:end]) > bound)
                metrics[label] = dict(peak_abs=float(abs(values[peak])), signed_peak=float(values[peak]),
                    peak_time_s=float(t[peak]), reference_bound=bound,
                    first_exceedance_s=float(t[crossings[0]]) if len(crossings) else None,
                    samples_exceeding=int(len(crossings)))
            metrics.update(measured_position_min_rad=float(q[:end, j].min()),
                measured_position_max_rad=float(q[:end, j].max()),
                target_min_rad=float(target[:end, j].min()), target_max_rad=float(target[:end, j].max()))
            result[name] = metrics
        return result
    return dict(interpretation='Observed simulated demands; not safe hardware limits',
        first_fall_proxy_s=float(t[first_fall]) if first_fall < len(rows) else None,
        fall_proxy='body height <0.25 m or tilt >0.9 rad; recorded without stopping',
        min_body_height_m=min(r['height'] for r in rows),
        max_body_tilt_rad=max(r['tilt'] for r in rows),
        max_body_linear_speed_m_s=max(float(np.linalg.norm(r['linear_velocity'])) for r in rows),
        max_body_angular_speed_rad_s=max(float(np.linalg.norm(r['angular_velocity'])) for r in rows),
        all_samples=window(len(rows)), before_first_fall=window(first_fall),
        physical_saturation_retained=True, hardware_qualified=False)
