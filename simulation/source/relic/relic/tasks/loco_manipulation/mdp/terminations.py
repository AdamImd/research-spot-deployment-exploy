# Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

"""Functions specific to the interlimb loco-manipulation environments."""

import torch

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor


def illegal_ground_contact(
    env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Terminate when the contact force on the sensor exceeds the force threshold."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contact_forces_with_ground = contact_sensor.data.force_matrix_w[
        :, sensor_cfg.body_ids, ...
    ]
    # check if any contact force with the ground exceeds the threshold
    # Preserve the environment axis even for one robot. Flatten only the body
    # and ground-filter axes; an unrestricted squeeze removes the batch axis.
    return (
        torch.norm(contact_forces_with_ground, dim=-1).flatten(start_dim=1).amax(dim=1) > threshold
    )
