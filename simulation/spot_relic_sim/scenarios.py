"""Registered command sequence and paired perturbations; no hidden resets."""
from dataclasses import dataclass, asdict
import numpy as np
from .contract import Command, defaults, C


@dataclass
class Scenario:
    name: str = "nominal"
    mode: str = "four"
    seed: int = 101
    duration: float = 24.0
    friction: float = .8
    mass_delta: float = 0.0
    delay: int = 0
    push_time: float = -1.0
    push_dvy: float = 0.0
    smoke: bool = False

    def command(self, t):
        command = Command()
        if not self.smoke:
            command.velocity = np.array(
                [0, 0, 0] if t < 4 else [.3, 0, 0] if t < 8 else
                [0, .2, 0] if t < 12 else [0, 0, .4] if t < 16 else
                [-.2, 0, 0] if t < 20 else [0, 0, 0], dtype=np.float32)
        if self.mode != "four" and t >= 2:
            command.leg = self.mode
            start = defaults(C.LEG_JOINT_NAMES[self.mode])
            end = np.array([start[0], 1.3, -2.4], dtype=np.float32)
            command.leg_pose = start + min((t - 2) / 2, 1) * (end - start)
        return command

    def to_dict(self):
        return asdict(self)


def scenario(name, mode, seed, smoke=False):
    s = Scenario(name=name, mode=mode, seed=seed, smoke=smoke, duration=4 if smoke else 24)
    if name == "low_friction":
        s.friction = .4
    elif name == "payload":
        s.mass_delta = 2.5
    elif name == "delay":
        s.delay = 3
    elif name == "push":
        s.push_time, s.push_dvy = 10, .3
    elif name != "nominal":
        raise ValueError(name)
    return s
