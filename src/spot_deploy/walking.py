"""Bounded forward travel using robot odom/body poses, never elapsed-time distance."""

import math

import numpy as np
from pydantic import Field, model_validator

from .contracts import ContractError, StrictModel
from .policy import rotation


class WalkingPlan(StrictModel):
    distance_m: float = Field(gt=0, le=1)
    max_speed_m_s: float = Field(gt=0, le=.15)
    acceleration_m_s2: float = Field(gt=0, le=.2)
    tolerance_m: float = Field(gt=0, le=.03)
    max_lateral_m: float = Field(gt=0, le=.15)
    max_heading_rad: float = Field(gt=0, le=.2)
    max_pose_age_s: float = Field(gt=0, le=.03)
    max_pose_step_m: float = Field(gt=0, le=.03)
    initial_hold_s: float = Field(ge=.5, le=3)
    final_hold_s: float = Field(ge=.5, le=3)
    stopped_speed_m_s: float = Field(gt=0, le=.04)
    max_duration_s: float = Field(gt=0, le=30)

    @model_validator(mode='after')
    def feasible(self):
        if (self.tolerance_m >= self.distance_m / 2
                or self.distance_m / self.max_speed_m_s + self.initial_hold_s + self.final_hold_s
                >= self.max_duration_s):
            raise ValueError('walking tolerance or timeout is infeasible')
        return self


class ForwardTravel:
    def __init__(self, plan, record):
        self.plan, self.record = plan, record
        self.origin = self.started_at = self.last_at = self.last_pose_time = None
        self.last_position = None
        self.settled_at = None
        self.velocity = 0.
        self.phase = 'uninitialized'
        self.completed = False
        self.progress = None
        self.last_recorded = -math.inf

    def pose(self, state):
        p, stamp = state.body_position_odom, state.body_pose_robot_time_s
        if p is None or stamp is None:
            raise ContractError('walking requires robot odom/body position and acquisition time')
        if abs(state.robot_time_s - stamp) > self.plan.max_pose_age_s:
            raise ContractError('walking odom/body pose is stale or future-dated')
        p = np.asarray(p)
        if self.last_pose_time is not None:
            if stamp < self.last_pose_time:
                raise ContractError('walking pose timestamp moved backwards')
            change = np.linalg.norm(p[:2] - self.last_position[:2])
            if ((stamp == self.last_pose_time and change > 1e-8)
                    or change > self.plan.max_pose_step_m):
                raise ContractError('walking odometry discontinuity')
        self.last_pose_time, self.last_position = stamp, p.copy()
        r = rotation(state)
        yaw = math.atan2(r[1, 0], r[0, 0])
        return p, yaw

    def latch(self, state):
        p, yaw = self.pose(state)
        self.origin, self.yaw = p.copy(), yaw
        self.axes = np.array([[math.cos(yaw), math.sin(yaw)],
                              [-math.sin(yaw), math.cos(yaw)]])
        self.phase = 'initial_hold'
        self.record.event('walk_origin', odom_position_m=p.tolist(), yaw_rad=yaw,
            robot_time_s=state.body_pose_robot_time_s, plan=self.plan.model_dump(),
            source='robot odom_tform_body', reference_frame='initial gravity-aligned body',
            distance_formula='initial_heading_rotation.T @ (current_odom_xy - initial_odom_xy)')

    def activate(self, now):
        if self.origin is None or self.started_at is not None:
            raise ContractError('invalid walking activation')
        self.started_at = self.last_at = now

    def update(self, state, now):
        p, yaw = self.pose(state)
        if self.started_at is None or now < self.last_at:
            raise ContractError('walking clock moved backwards or is not active')
        x, y = self.axes @ (p[:2] - self.origin[:2])
        heading = math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
        remaining = self.plan.distance_m - x
        if abs(y) > self.plan.max_lateral_m or abs(heading) > self.plan.max_heading_rad:
            raise ContractError('walking lateral or heading deviation limit')
        if x < -self.plan.tolerance_m or x > self.plan.distance_m + self.plan.tolerance_m:
            raise ContractError('walking backwards or overshoot limit')
        elapsed, dt = now - self.started_at, now - self.last_at
        old_phase = self.phase
        speed = float(np.linalg.norm(np.asarray(state.linear_velocity_odom)[:2]))
        if not self.completed and elapsed > self.plan.max_duration_s:
            raise ContractError('walking distance timeout')
        if self.completed:
            desired = 0.
            if abs(remaining) > self.plan.tolerance_m:
                raise ContractError('walking final position drift')
        elif elapsed < self.plan.initial_hold_s:
            desired = 0.
        elif abs(remaining) <= self.plan.tolerance_m:
            # Immediate zero request at the distance boundary; hold until actually stopped.
            self.phase, desired = 'final_hold', 0.
            if speed <= self.plan.stopped_speed_m_s:
                if self.settled_at is None:
                    self.settled_at = now
                if now - self.settled_at >= self.plan.final_hold_s:
                    self.completed, self.phase = True, 'completed'
            else:
                self.settled_at = None
        else:
            self.phase, self.settled_at = 'forward', None
            desired = min(self.plan.max_speed_m_s, max(0., remaining) * .8)
        if desired == 0.:
            self.velocity = 0.
        else:
            step = self.plan.acceleration_m_s2 * dt
            self.velocity += float(np.clip(desired - self.velocity, -step, step))
        self.last_at = now
        self.progress = dict(phase=self.phase, forward_m=float(x), lateral_m=float(y),
            heading_error_rad=heading, remaining_m=float(remaining), speed_m_s=speed,
            requested_forward_m_s=self.velocity, odom_position_m=p.tolist(),
            robot_time_s=state.body_pose_robot_time_s, monotonic_s=now)
        if now - self.last_recorded >= .05 or self.phase != old_phase:
            self.record.event('walk_progress', **self.progress)
            self.last_recorded = now
        return [self.velocity, 0., 0.]
