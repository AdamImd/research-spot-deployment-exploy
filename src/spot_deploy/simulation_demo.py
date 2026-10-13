"""Simulation-only three-leg commands; intentionally absent from the live CLI.

The nine-input standing Exploy export cannot represent the twelve leg-command
values. The selected-leg phase uses the unchanged released actor and the native
ReLIC rule: overwrite only that leg's three position targets.
"""

from typing import Literal
from time import monotonic

import numpy as np
from pydantic import Field, model_validator

from .contracts import JOINTS, ContractError, StrictModel
from .relic_policy import ReLICPolicy, finite, joint_targets, observation


class CartesianLegPath(StrictModel):
    frame: Literal['body']
    waypoints_m: list[list[float]] = Field(min_length=2, max_length=9)
    segment_s: float = Field(ge=2, le=5)
    max_joint_speed_rad_s: float = Field(gt=0, le=1)
    max_tracking_error_m: float = Field(gt=0, le=.05)

    @model_validator(mode='after')
    def check(self):
        for point in self.waypoints_m:
            xyz = finite(point, 3)
            if not (.45 <= xyz[0] <= .75 and .10 <= xyz[1] <= .35
                    and -.40 <= xyz[2] <= -.15):
                raise ValueError('IK demo waypoints must be in the bounded front-left body workspace')
        if not np.allclose(self.waypoints_m[0], self.waypoints_m[-1], rtol=0, atol=1e-8):
            raise ValueError('IK path must return to its first waypoint before lowering')
        return self

    @property
    def duration_s(self):
        return (len(self.waypoints_m)-1)*self.segment_s


class ThreeLegPlan(StrictModel):
    kind: Literal['front-left-lift-hold-return', 'front-left-cartesian-ik']
    leg: Literal['fl']
    initial_hold_s: float = Field(ge=2, le=10)
    lift_s: float = Field(ge=2, le=10)
    hold_s: float = Field(ge=2, le=10)
    lower_s: float = Field(ge=2, le=10)
    final_hold_s: float = Field(ge=5, le=20)
    return_blend_s: float = Field(default=0, ge=0, le=5)
    entry_blend_s: float = Field(default=0, ge=0, le=5)
    lifted_positions: list[float] | None = Field(default=None, min_length=3, max_length=3)
    cartesian_path: CartesianLegPath | None = None
    min_clearance_m: float = Field(gt=0, le=.15)
    support_clearance_max_m: float = Field(gt=0, le=.03)
    min_hold_fraction: float = Field(ge=.8, le=1)
    foot_radius_m: float = Field(gt=0, le=.1)

    @model_validator(mode='after')
    def mode(self):
        if (self.return_blend_s or self.entry_blend_s) and self.kind != 'front-left-cartesian-ik':
            raise ValueError('return blend requires the Cartesian sequence')
        if self.kind == 'front-left-cartesian-ik':
            if self.cartesian_path is None or self.lifted_positions is not None:
                raise ValueError('IK demo requires a Cartesian path, without a joint-space lifted pose')
        elif self.lifted_positions is None or self.cartesian_path is not None:
            raise ValueError('joint-space demo requires only a lifted pose')
        return self

    @property
    def duration_s(self):
        return sum(getattr(self, k) for k in ('initial_hold_s', 'lift_s', 'hold_s',
                                            'lower_s', 'final_hold_s')) + (
            self.cartesian_path.duration_s if self.cartesian_path else 0.) + self.return_blend_s + self.entry_blend_s


def smoothstep(alpha):
    a = float(np.clip(alpha, 0, 1))
    return a**3 * (10 + a * (-15 + 6*a))


def mix_entry(sequence, four_target, four_raw, selected_target, selected_raw):
    """Mix two unchanged actor evaluations using the SAME acknowledged history.

    Mix raw actions alongside targets so the next ACK reflects this controller's
    emitted mixture rather than the unused selected-mode branch. Held arm remains
    identical in both branches; selected-leg override is part of selected_target.
    """
    alpha = sequence.entry_alpha
    return (four_target+alpha*(selected_target-four_target),
            np.asarray(four_raw+alpha*(selected_raw-four_raw),dtype=np.float32))


class ThreeLegSequence:
    @property
    def entry_alpha(self):
        if self.phase != 'three-leg-handover':
            return 1.
        return smoothstep((self.last_time-self.plan.initial_hold_s)/self.plan.entry_blend_s)

    def __init__(self, plan, ik=None):
        self.plan = plan
        self.start = None
        self.phase = 'four-foot-initial'
        self.leg = None
        self.pose = np.zeros(3, dtype=np.float32)
        self.last_time = -np.inf
        self.ids = [JOINTS.index(f'{plan.leg}_{axis}') for axis in ('hx', 'hy', 'kn')]
        self.ik = ik
        self.target_body_m = None
        self.ik_residual_m = None
        self.start_body_m = None
        if plan.cartesian_path and ik is None:
            raise ContractError('Cartesian demo requires pinned model IK')

    def _cartesian_update(self, state, elapsed, dt, previously_active):
        p, path = self.plan, self.plan.cartesian_path
        lift_start = p.initial_hold_s+p.entry_blend_s
        lift_end = lift_start+p.lift_s
        hold_end = lift_end+p.hold_s
        move_end = hold_end+path.duration_s
        lower_end = move_end+p.lower_s
        if elapsed < p.initial_hold_s:
            self.phase, self.leg, self.target_body_m = 'four-foot-initial', None, None
            return
        if elapsed >= lower_end:
            self.phase = ('four-foot-handover' if elapsed < lower_end+p.return_blend_s
                          else 'four-foot-final')
            self.leg, self.target_body_m = None, None
            return
        if self.start is None:
            self.start = finite(np.asarray(state.positions)[self.ids], 3).copy()
            self.pose = self.start.copy()
            self.start_body_m = self.ik.forward(self.start)[0]
        self.leg = p.leg
        first = np.asarray(path.waypoints_m[0])
        if elapsed < lift_start:
            self.phase, target = 'three-leg-handover', self.start_body_m
        elif elapsed < lift_end:
            self.phase = 'lifting'
            a = smoothstep((elapsed-lift_start)/p.lift_s)
            target = self.start_body_m+a*(first-self.start_body_m)
        elif elapsed < hold_end:
            self.phase, target = 'three-leg-hold', first
        elif elapsed < move_end:
            self.phase = 'three-leg-ik-motion'
            segment = min(int((elapsed-hold_end)/path.segment_s), len(path.waypoints_m)-2)
            a = smoothstep((elapsed-hold_end-segment*path.segment_s)/path.segment_s)
            begin, end = np.array(path.waypoints_m[segment:segment+2])
            target = begin+a*(end-begin)
        else:
            self.phase = 'lowering'
            a = smoothstep((elapsed-move_end)/p.lower_s)
            target = first+a*(self.start_body_m-first)
        solved, residual = self.ik.solve(target, self.pose)
        if previously_active and dt > 0 and np.any(
                np.abs(solved-self.pose) > path.max_joint_speed_rad_s*dt+1e-6):
            raise ContractError('IK selected-leg command speed exceeded')
        self.pose, self.target_body_m, self.ik_residual_m = solved, target, residual

    def update(self, state, elapsed):
        if not np.isfinite(elapsed) or elapsed < 0 or elapsed < self.last_time:
            raise ContractError('three-leg simulation clock moved backwards')
        dt, previously_active = elapsed-self.last_time, self.leg is not None
        self.last_time = elapsed
        p = self.plan
        if p.cartesian_path:
            self._cartesian_update(state, elapsed, dt, previously_active)
            command = np.zeros(12, dtype=np.float32)
            if self.leg:
                command[self.ids] = self.pose
            return command
        boundaries = np.cumsum([p.initial_hold_s, p.lift_s, p.hold_s, p.lower_s])
        if elapsed < boundaries[0]:
            self.phase, self.leg = 'four-foot-initial', None
        elif elapsed < boundaries[3]:
            if self.start is None:
                self.start = finite(np.asarray(state.positions)[self.ids], 3).copy()
            self.leg = p.leg
            lifted = finite(p.lifted_positions, 3)
            if elapsed < boundaries[1]:
                self.phase = 'lifting'
                blend = smoothstep((elapsed-boundaries[0])/p.lift_s)
            elif elapsed < boundaries[2]:
                self.phase, blend = 'three-leg-hold', 1.
            else:
                self.phase = 'lowering'
                blend = 1-smoothstep((elapsed-boundaries[2])/p.lower_s)
            self.pose = finite(self.start + blend * (lifted-self.start), 3)
        else:
            self.phase, self.leg = 'four-foot-final', None
        command = np.zeros(12, dtype=np.float32)
        if self.leg:
            command[self.ids] = self.pose
        return command


class SimulationThreeLegPolicy:
    """Wrap a standing adapter solely inside an offline simulator process."""
    def __init__(self, baseline, checkpoint, plan, selected_leg_graph=None):
        if baseline.manifest.task != 'standing':
            raise ContractError('three-leg demo requires the standing baseline, not a walking task')
        self.baseline = baseline
        self.actor = ReLICPolicy(checkpoint)
        self.runner = self.actor  # Simulator provenance records the actual inference provider.
        self.ik = None
        if plan.cartesian_path:
            from .leg_ik import LegPositionIK
            self.ik = LegPositionIK(baseline.support, baseline.support_model, plan.leg)
        self.sequence = ThreeLegSequence(plan, self.ik)
        self.manifest, self.support = baseline.manifest, baseline.support
        self.command = np.zeros(12, dtype=np.float32)
        self.selected_session = None
        if selected_leg_graph is not None:
            import json
            import onnxruntime as ort
            from .exploy_policy import META_KEY, INPUT_SIZES
            from .contracts import sha256
            record = json.loads(selected_leg_graph.with_name('export.json').read_text())
            if record['policy_sha256'] != sha256(selected_leg_graph):
                raise ContractError('selected-leg graph differs from its parity record')
            options = ort.SessionOptions()
            options.intra_op_num_threads = options.inter_op_num_threads = 1
            self.selected_session = ort.InferenceSession(str(selected_leg_graph),
                sess_options=options, providers=['CPUExecutionProvider'])
            metadata = json.loads(self.selected_session.get_modelmeta().custom_metadata_map[META_KEY])
            if (metadata['contract'] != 'relic-exploy-leg-v1'
                    or metadata['joint_names'] != list(JOINTS)
                    or metadata['selected_leg'] != 'fl'
                    or record['weights_changed'] or record['parity_samples'] != 64):
                raise ContractError('selected-leg graph contract/parity mismatch')
            sizes = dict(INPUT_SIZES, leg_command=12)
            fields = self.selected_session.get_inputs()
            if {f.name for f in fields} != set(sizes) or any(
                f.shape != [1,sizes[f.name]] or f.type != 'tensor(float)' for f in fields):
                raise ContractError('selected-leg graph input layout mismatch')
            self.runner = type('Runner', (), {'session':self.selected_session})()

    @property
    def body_height(self):
        return self.baseline.body_height

    def initialize_height(self, height):
        self.baseline.initialize_height(height)

    def reset(self):
        self.baseline.reset()
        self.actor.previous_action[:] = 0
        self.sequence = ThreeLegSequence(self.sequence.plan, self.ik)
        self.command[:] = 0

    def warmup(self):
        if self.selected_session is not None:
            from .exploy_policy import INPUT_SIZES
            from .relic_policy import DEFAULT_Q
            values = {k:np.zeros((1,n),dtype=np.float32) for k,n in dict(INPUT_SIZES,leg_command=12).items()}
            values['base_quaternion'][0,0] = 1
            values['joint_positions'][0] = DEFAULT_Q[self.baseline.graph_from_sdk]
            values['arm_command'][0] = self.manifest.arm_stowed_positions
            values['torso_command'][0,2] = .55
            self.selected_session.run(['actions'],values)
            return
        self.actor.session.run(['actions'], {'obs':np.zeros((1,84),dtype=np.float32)})

    def update(self, state, elapsed):
        self.command = self.sequence.update(state, elapsed)

    def return_blend(self, target):
        """Blend only the selected-leg OUTPUT after native mode switches to zero.

        Raw actions/history and all other targets remain unchanged. This is a
        return handoff, not a startup fade or altered observation/history rule.
        """
        if self.sequence.phase != 'four-foot-handover':
            return target
        p = self.sequence.plan
        begin = p.initial_hold_s+p.entry_blend_s+p.lift_s+p.hold_s+p.lower_s+p.cartesian_path.duration_s
        alpha = smoothstep((self.sequence.last_time-begin)/p.return_blend_s)
        target = np.asarray(target).copy()
        ids = self.sequence.ids
        target[ids] = self.sequence.pose+alpha*(target[ids]-self.sequence.pose)
        return target

    def kinematic_record(self, state):
        if self.ik is None or self.sequence.leg is None:
            return {}
        measured, _ = self.ik.forward(np.asarray(state.positions)[self.sequence.ids])
        return dict(ik_frame='body', ik_target_body_m=self.sequence.target_body_m.tolist(),
                    ik_measured_body_m=measured.tolist(), ik_solve_residual_m=self.sequence.ik_residual_m,
                    ik_tracking_error_m=float(np.linalg.norm(measured-self.sequence.target_body_m)))

    def evaluate(self, state, previous):
        began = monotonic()
        if self.selected_session is not None:
            from .exploy_policy import TARGET
            values = self.baseline.inputs(state, previous)
            values['leg_command'] = self.command[None]
            action, obs, target = self.selected_session.run(['actions','obs',TARGET],values)
            target = finite(target[0],19).astype(float)
            target[12:] = self.manifest.arm_stowed_positions
            action = finite(action[0],12)
            if self.sequence.phase == 'three-leg-handover':
                values['leg_command'] = np.zeros((1,12),dtype=np.float32)
                four_raw, four_target = self.selected_session.run(['actions',TARGET],values)
                four_target = finite(four_target[0],19).astype(float)
                four_target[12:] = self.manifest.arm_stowed_positions
                target, action = mix_entry(self.sequence, four_target, four_raw[0], target, action)
            target = self.return_blend(target)
            return target, monotonic()-began, obs, action
        obs = observation(state, self.manifest.arm_stowed_positions, previous, self.body_height)
        obs[19:31] = self.command
        action = finite(self.actor.session.run(['actions'], {'obs':obs[None]})[0][0],12)
        target = joint_targets(action, self.manifest.arm_stowed_positions).astype(float)
        target[12:] = self.manifest.arm_stowed_positions
        if self.sequence.leg:
            target[self.sequence.ids] = self.sequence.pose
        if self.sequence.phase == 'three-leg-handover':
            four_obs = obs.copy()
            four_obs[19:31] = 0
            four_raw = finite(self.actor.session.run(['actions'],{'obs':four_obs[None]})[0][0],12)
            four_target = joint_targets(four_raw,self.manifest.arm_stowed_positions).astype(float)
            four_target[12:] = self.manifest.arm_stowed_positions
            target, action = mix_entry(self.sequence, four_target, four_raw, target, action)
        target = self.return_blend(target)
        return target, monotonic()-began, obs[None], action


def demo_metrics(rows, plan):
    """Geometric evidence is a clearance proxy, never a foot load/contact claim."""
    hold = [r for r in rows if r['demo_phase'] in ('three-leg-hold','three-leg-ik-motion')]
    final = [r for r in rows if r['demo_phase']=='four-foot-final']
    last = [r for r in final if r['trial_time'] >= plan.duration_s-2]
    def clearances(items):
        return np.asarray([r['feet'] for r in items])[:,:,2]-plan.foot_radius_m
    fraction = 0.
    if hold:
        c = clearances(hold)
        fraction = float(np.mean((c[:,0] >= plan.min_clearance_m)
                        & np.all(c[:,1:] <= plan.support_clearance_max_m,axis=1)))
    final_fraction = (float(np.mean(np.all(clearances(last)
                        <= plan.support_clearance_max_m,axis=1))) if last else 0.)
    motion = [r for r in rows if r['demo_phase']=='three-leg-ik-motion']
    ik_passed = None
    if plan.cartesian_path:
        errors = [r.get('ik_tracking_error_m',np.inf) for r in motion]
        ik_passed = bool(errors and np.quantile(errors,.95) <= plan.cartesian_path.max_tracking_error_m)
    return dict(hold_samples=len(hold), geometric_three_leg_fraction=fraction,
        ik_motion_samples=len(motion), ik_tracking_passed=ik_passed,
        final_four_feet_near_floor_fraction=final_fraction,
        pose_sequence_finished=bool(last and rows[-1]['trial_time'] >= plan.duration_s-1e-8),
        clearance_proxy_passed=bool(fraction >= plan.min_hold_fraction
                                   and final_fraction >= plan.min_hold_fraction),
        support_evidence='foot clearance geometry only; foot forces/loaded contacts not measured',
        hardware_qualified=False)
