"""Explicit moving-arm ReLIC contract using the byte-identical nine-input graph."""
from typing import Literal

import numpy as np
from pydantic import model_validator

from .contracts import ContractError
from .exploy_policy import ExployReLICManifest, ExployReLICPolicy
from .joystick_teleop import ArmPath, JoystickTeleopPlan
from .relic_policy import finite


class TeleopManifest(ExployReLICManifest):
    schema_version: Literal[5]
    # Same adapter, new task/morphology. The graph already accepts arm commands.
    task: Literal['joystick-arm']
    morphology: Literal['spot-arm-mobile']
    teleop: JoystickTeleopPlan

    @model_validator(mode='after')
    def teleop_contract(self):
        if self.walking is not None or self.relic.preparation.kind != 'direct':
            raise ValueError('joystick arm control requires direct startup and no distance plan')
        ArmPath(self.arm_stowed_positions, self.teleop)
        return self


class TeleopPolicy(ExployReLICPolicy):
    def reset(self):
        super().reset()
        self.arm_command = np.asarray(self.manifest.arm_stowed_positions, dtype=float)

    def velocity_input(self, velocity_command):
        command = finite([0., 0., 0.] if velocity_command is None else velocity_command, 3)
        if np.any(np.abs(command) > np.array([a.maximum for a in self.manifest.teleop.axes])+1e-7):
            raise ContractError('velocity command outside the explicit joystick plan')
        return command

    def inputs(self, state, previous, velocity_command=None):
        inputs = super().inputs(state, previous, velocity_command)
        inputs['arm_command'] = finite(self.arm_command, 7)[None]
        return inputs

    def output_arm_reference(self, inputs):
        # float64 reference retained in SDK output after graph float32 verification.
        return self.arm_command
