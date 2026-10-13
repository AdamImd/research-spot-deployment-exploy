"""Bounded USB motion input and reversible arm path; no robot or SDK capability."""
from dataclasses import dataclass
from typing import Literal
import time

import numpy as np
from pydantic import Field, model_validator

from .contracts import ContractError, StrictModel, Vector7
from .joystick_estop import JoystickDevice, JoystickLink


class Axis(StrictModel):
    index: int = Field(ge=0, le=31)
    sign: Literal[-1, 1]
    maximum: float = Field(gt=0, le=1)
    acceleration: float = Field(gt=0, le=1)


class JoystickTeleopPlan(StrictModel):
    device: JoystickDevice
    axes: list[Axis] = Field(min_length=3, max_length=3)  # forward, left, CCW yaw
    deadzone: float = Field(ge=.03, le=.4)
    enable_button: int = Field(ge=0, le=31)
    arm_button: int = Field(ge=0, le=31)
    finish_button: int = Field(ge=0, le=31)
    input_timeout_s: float = Field(gt=.02, le=.25)
    initial_hold_s: float = Field(ge=2, le=10)
    duration_s: float = Field(gt=5, le=300)
    # Intermediate clearance poses, then deployed pose, in SDK arm joint order.
    arm_waypoints: list[Vector7] = Field(min_length=1, max_length=5)
    arm_speed_rad_s: float = Field(gt=0, le=.5)
    arm_segment_min_s: float = Field(ge=2, le=10)

    @model_validator(mode='after')
    def distinct(self):
        if len({self.enable_button, self.arm_button, self.finish_button}) != 3:
            raise ValueError('enable, arm toggle and finish must use separate buttons')
        if len({a.index for a in self.axes}) != 3:
            raise ValueError('three distinct joystick axes required')
        if self.duration_s <= self.initial_hold_s:
            raise ValueError('duration must include the initial hold')
        return self


@dataclass(frozen=True)
class JoystickSample:
    at: float
    velocity: tuple[float, float, float]
    enabled: bool
    toggle: bool
    finish: bool


class JoystickInput:
    """Release then centered press to enable. INIT and held trigger never toggle."""
    def __init__(self, plan, link=None):
        self.plan = plan
        try:
            self.link = link if link is not None else JoystickLink(plan.device)
        except FileNotFoundError:
            raise ContractError('configured USB joystick is absent; reconnect it and run preview') from None
        except PermissionError:
            raise ContractError('USB joystick permission denied; grant the operator device access') from None
        self.last_at = None
        self.enabled = self.released = False
        self.buttons = {}
        try:
            if (max(a.index for a in plan.axes) >= self.link.metadata['axis_count']
                    or max(plan.enable_button, plan.arm_button, plan.finish_button)
                    >= self.link.metadata['button_count']):
                raise ContractError('teleop mapping exceeds joystick axes/buttons')
        except BaseException:
            self.close()
            raise

    def poll(self, now):
        p = self.plan
        if not np.isfinite(now) or (self.last_at is not None and
                not 0 <= now-self.last_at <= p.input_timeout_s):
            raise ContractError('joystick input polling expired; restart required')
        self.last_at = now
        events = self.link.poll()
        axes = np.array([self.link.axes[a.index]*a.sign for a in p.axes])
        if not np.isfinite(axes).all() or np.any(np.abs(axes) > 1.001):
            raise ContractError('invalid joystick axes')
        centered = bool(np.all(np.abs(axes) <= p.deadzone))
        toggle = finish = False
        # Release/finish wins over all presses in a single poll batch.
        released_batch = any(e.button == p.enable_button and not e.pressed for e in events)
        for e in events:
            previous = self.buttons.get(e.button)
            self.buttons[e.button] = e.pressed
            if e.button == p.finish_button and e.pressed:
                finish = True
            if e.button == p.enable_button:
                if not e.pressed:
                    self.enabled, self.released = False, True
                elif previous is False and self.released and centered and not e.initial:
                    self.enabled, self.released = True, False
            if (e.button == p.arm_button and e.pressed and previous is False
                    and not e.initial and self.enabled and not released_batch):
                toggle = True
        if released_batch or self.buttons.get(p.enable_button) is not True or finish:
            self.enabled = toggle = False
        axes = np.clip(axes, -1, 1)
        mapped = np.sign(axes)*np.maximum(0, np.abs(axes)-p.deadzone)/(1-p.deadzone)
        velocity = mapped*np.array([a.maximum for a in p.axes]) if self.enabled else np.zeros(3)
        return JoystickSample(now, tuple(velocity), self.enabled, toggle, finish)

    def close(self):
        self.link.close()

    def reset_after_native_stand(self):
        """Discard the unconsumed power/stand queue; never resume an active link.

        Reopening gets a fresh kernel snapshot, not replayed button edges. Held
        enable/trigger INIT events cannot enable or toggle; a fresh release/press
        with centered axes is required. The physical device must stay identical.
        """
        identity = self.link.identity
        self.link.close()
        self.link = JoystickLink(self.plan.device)
        try:
            if self.link.identity != identity:
                raise ContractError('joystick changed during native stand; restart required')
            self.buttons = {}
            self.enabled = self.released = False
            self.last_at = None
            sample = self.poll(time.monotonic())
            if sample.enabled or sample.toggle or any(sample.velocity):
                raise ContractError('invalid joystick startup reset')
            self.last_at = None
        except BaseException:
            self.link.close()
            raise


class ArmPath:
    """Reverse along the SAME joint path, with bounded progress and zero startup jump.

    This path is a reviewed setpoint proposal, not a collision planner. Released
    enable freezes the reference. Firmware/native stow planners are not invoked.
    """
    def __init__(self, stow, plan):
        self.points = np.asarray([stow, *plan.arm_waypoints], dtype=float)
        if not np.isfinite(self.points).all() or np.any(self.points[:, 6] != stow[6]):
            raise ContractError('arm path must be finite and keep the gripper unchanged')
        self.durations = np.maximum(plan.arm_segment_min_s,
            1.875*np.max(np.abs(np.diff(self.points, axis=0)), axis=1)/plan.arm_speed_rad_s)
        self.ends = np.r_[0., np.cumsum(self.durations)]
        self.progress = 0.
        self.deployed = False

    @property
    def reference(self):
        i = min(len(self.durations)-1, max(0, np.searchsorted(self.ends, self.progress, side='right')-1))
        a = np.clip((self.progress-self.ends[i])/self.durations[i], 0, 1)
        a = a**3*(10+a*(-15+6*a))
        return self.points[i]+a*(self.points[i+1]-self.points[i])

    def advance(self, dt, enabled, toggle):
        if toggle:
            self.deployed = not self.deployed
        if enabled:
            self.progress = float(np.clip(self.progress+(dt if self.deployed else -dt),
                                          0, self.ends[-1]))
        return self.reference.copy()


class TeleopMotion:
    def __init__(self, manifest, source):
        self.plan, self.source = manifest.teleop, source
        self.arm = ArmPath(manifest.arm_stowed_positions, self.plan)
        self.velocity = np.zeros(3)
        self.active_at = self.last_at = None
        self.completed = False
        self.status = {}

    def update(self, now, active_at):
        sample = self.source.poll(now)
        if not 0 <= now-sample.at <= self.plan.input_timeout_s:
            raise ContractError('stale joystick sample')
        if self.last_at is not None and not 0 <= now-self.last_at <= self.plan.input_timeout_s:
            raise ContractError('teleop update gap expired')
        # No accumulated motion after a delayed tick.
        dt = 0 if self.last_at is None else min(.005, now-self.last_at)
        self.last_at = now
        ready = active_at is not None and now-active_at >= self.plan.initial_hold_s
        enabled = bool(ready and sample.enabled and not sample.finish)
        demand = np.asarray(sample.velocity, dtype=float)
        maximum = np.asarray([a.maximum for a in self.plan.axes])
        if demand.shape != (3,) or not np.isfinite(demand).all() or np.any(np.abs(demand)>maximum+1e-7):
            raise ContractError('joystick velocity outside the bound plan')
        if enabled:
            delta = np.asarray([a.acceleration for a in self.plan.axes])*dt
            self.velocity += np.clip(demand-self.velocity, -delta, delta)
        else:
            self.velocity[:] = 0  # Releasing enable requests zero immediately.
        self.arm.advance(dt, enabled, enabled and sample.toggle)
        self.completed = bool(sample.finish)
        self.status = dict(enabled=enabled, ready=ready, finish=sample.finish,
            requested_velocity=demand.tolist(), velocity_command=self.velocity.tolist(),
            arm_deployed_requested=self.arm.deployed, arm_path_progress_s=self.arm.progress,
            arm_command=self.arm.reference.tolist(), input_at=sample.at)
        return self.velocity.tolist(), self.arm.reference


class ScriptRow(StrictModel):
    time_s: float = Field(ge=0)
    velocity: list[float] = Field(min_length=3, max_length=3)
    enabled: bool
    toggle: bool = False
    finish: bool = False


class ScriptedJoystick:
    """Simulation-only deterministic input. Never selected by the hardware CLI."""
    def __init__(self, rows, start=10.):
        self.rows = [ScriptRow.model_validate(r) for r in rows]
        if (not self.rows or self.rows[0].time_s != 0 or
                any(a.time_s >= b.time_s for a, b in zip(self.rows, self.rows[1:]))):
            raise ContractError('script needs ordered times starting at zero')
        self.start, self.index = start, -1

    def poll(self, now):
        toggle = finish = False
        while self.index+1 < len(self.rows) and self.rows[self.index+1].time_s <= now-self.start+1e-9:
            self.index += 1
            toggle ^= self.rows[self.index].toggle
            finish |= self.rows[self.index].finish
        if self.index < 0:
            return JoystickSample(now, (0., 0., 0.), False, False, False)
        row = self.rows[self.index]
        return JoystickSample(now, tuple(row.velocity), row.enabled, toggle, finish)
