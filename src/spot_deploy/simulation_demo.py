"""Simulation-only three-leg commands; intentionally absent from the live CLI.

The nine-input standing Exploy export cannot represent the twelve leg-command
values. The selected-leg phase uses the unchanged released actor and the native
ReLIC rule: overwrite only that leg's three position targets.
"""

from typing import Literal
from time import monotonic

import numpy as np
from pydantic import Field

from .contracts import JOINTS, ContractError, StrictModel
from .relic_policy import ReLICPolicy, finite, joint_targets, observation


class ThreeLegPlan(StrictModel):
    kind: Literal['front-left-lift-hold-return']
    leg: Literal['fl']
    initial_hold_s: float = Field(ge=2, le=10)
    lift_s: float = Field(ge=2, le=10)
    hold_s: float = Field(ge=2, le=10)
    lower_s: float = Field(ge=2, le=10)
    final_hold_s: float = Field(ge=5, le=20)
    lifted_positions: list[float] = Field(min_length=3, max_length=3)
    min_clearance_m: float = Field(gt=0, le=.15)
    support_clearance_max_m: float = Field(gt=0, le=.03)
    min_hold_fraction: float = Field(ge=.8, le=1)
    foot_radius_m: float = Field(gt=0, le=.1)

    @property
    def duration_s(self):
        return sum(getattr(self, k) for k in ('initial_hold_s', 'lift_s', 'hold_s',
                                            'lower_s', 'final_hold_s'))


def smoothstep(alpha):
    a = float(np.clip(alpha, 0, 1))
    return a**3 * (10 + a * (-15 + 6*a))


class ThreeLegSequence:
    def __init__(self, plan):
        self.plan = plan
        self.start = None
        self.phase = 'four-foot-initial'
        self.leg = None
        self.pose = np.zeros(3, dtype=np.float32)
        self.last_time = -np.inf
        self.ids = [JOINTS.index(f'{plan.leg}_{axis}') for axis in ('hx', 'hy', 'kn')]

    def update(self, state, elapsed):
        if not np.isfinite(elapsed) or elapsed < 0 or elapsed < self.last_time:
            raise ContractError('three-leg simulation clock moved backwards')
        self.last_time = elapsed
        p = self.plan
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
    def __init__(self, baseline, checkpoint, plan):
        if baseline.manifest.task != 'standing':
            raise ContractError('three-leg demo requires the standing baseline, not a walking task')
        self.baseline = baseline
        self.actor = ReLICPolicy(checkpoint)
        self.runner = self.actor  # Simulator provenance records the actual inference provider.
        self.sequence = ThreeLegSequence(plan)
        self.manifest, self.support = baseline.manifest, baseline.support
        self.command = np.zeros(12, dtype=np.float32)

    @property
    def body_height(self):
        return self.baseline.body_height

    def initialize_height(self, height):
        self.baseline.initialize_height(height)

    def reset(self):
        self.baseline.reset()
        self.actor.previous_action[:] = 0
        self.sequence = ThreeLegSequence(self.sequence.plan)
        self.command[:] = 0

    def warmup(self):
        self.actor.session.run(['actions'], {'obs':np.zeros((1,84),dtype=np.float32)})

    def update(self, state, elapsed):
        self.command = self.sequence.update(state, elapsed)

    def evaluate(self, state, previous):
        began = monotonic()
        obs = observation(state, self.manifest.arm_stowed_positions, previous, self.body_height)
        obs[19:31] = self.command
        action = finite(self.actor.session.run(['actions'], {'obs':obs[None]})[0][0],12)
        target = joint_targets(action, self.manifest.arm_stowed_positions).astype(float)
        target[12:] = self.manifest.arm_stowed_positions
        if self.sequence.leg:
            target[self.sequence.ids] = self.sequence.pose
        return target, monotonic()-began, obs[None], action


def demo_metrics(rows, plan):
    """Geometric evidence is a clearance proxy, never a foot load/contact claim."""
    hold = [r for r in rows if r['demo_phase']=='three-leg-hold']
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
    return dict(hold_samples=len(hold), geometric_three_leg_fraction=fraction,
        final_four_feet_near_floor_fraction=final_fraction,
        pose_sequence_finished=bool(last and rows[-1]['trial_time'] >= plan.duration_s-1e-8),
        clearance_proxy_passed=bool(fraction >= plan.min_hold_fraction
                                   and final_fraction >= plan.min_hold_fraction),
        support_evidence='foot clearance geometry only; foot forces/loaded contacts not measured',
        hardware_qualified=False)
