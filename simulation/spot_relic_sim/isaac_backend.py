"""Native Isaac Lab articulation with the released ReLIC actuator definitions."""
from pathlib import Path
import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from relic.assets.spot.spot import SPOT_ARM_CFG
from .contract import Contract, DT, LEGS


class IsaacBackend:
    def __init__(self, output, scenario, device="cpu", headless=True):
        self.headless = headless
        self.sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=device,
                                              render_interval=4, gravity=(0, 0, -9.81)))
        self.sim.set_camera_view([2.3, 2.3, 1.7], [0, 0, .4])
        floor = sim_utils.CuboidCfg(size=(200, 200, .1),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=scenario.friction,
                    dynamic_friction=scenario.friction, restitution=0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.18, .22, .28)))
        floor.func("/World/Floor", floor, translation=(0, 0, -.05))
        light = sim_utils.DomeLightCfg(intensity=1800)
        light.func("/World/Light", light)
        cfg = SPOT_ARM_CFG.copy()
        cfg.prim_path = "/World/Spot"
        cfg.spawn.usd_dir = str(Path(output).resolve() / "usd")
        cfg.spawn.force_usd_conversion = True
        # Isaac Lab 0.54 requires explicit importer gains. ReLIC computes PD
        # torques itself, so the simulator's implicit drives must be disabled.
        cfg.spawn.joint_drive.target_type = "none"
        cfg.spawn.joint_drive.gains.stiffness = 0.0
        cfg.spawn.joint_drive.gains.damping = 0.0
        for actuator in cfg.actuators.values():
            actuator.min_delay = scenario.delay
            actuator.max_delay = scenario.delay
        self.robot = Articulation(cfg)
        self.sim.reset()
        self.contract = Contract(self.robot.joint_names)
        self.feet = [self.robot.body_names.index(f"{leg}_foot") for leg in LEGS]
        self.base_mass = self.robot.root_physx_view.get_masses().clone()
        self.root = self.robot.body_names.index("body")
        self.reset(scenario)

    def reset(self, scenario):
        robot = self.robot
        ids = torch.tensor([0], dtype=torch.int, device="cpu")
        masses = self.base_mass.clone()
        masses[0, self.root] += scenario.mass_delta
        robot.root_physx_view.set_masses(masses, ids)
        materials = robot.root_physx_view.get_material_properties()
        materials[..., :2] = scenario.friction
        materials[..., 2] = 0
        robot.root_physx_view.set_material_properties(materials, ids)
        robot.write_root_pose_to_sim(robot.data.default_root_state[:, :7].clone())
        robot.write_root_velocity_to_sim(robot.data.default_root_state[:, 7:].clone())
        robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
        robot.reset()
        self.sim.forward()
        for actuator in robot.actuators.values():
            # DelayedPD buffers are allocated with this cell's maximum delay.
            if actuator.cfg.min_delay != scenario.delay or actuator.cfg.max_delay != scenario.delay:
                raise ValueError("Create a new process for a different actuator delay")
        robot.update(DT)
        self._episode_start_time = float(self.sim.current_time)

    @property
    def physics_time(self):
        return float(self.sim.current_time) - self._episode_start_time

    def state(self):
        data = self.robot.data
        values = dict(position=data.root_pos_w[0], linear_velocity=data.root_lin_vel_b[0],
                      angular_velocity=data.root_ang_vel_b[0], gravity=data.projected_gravity_b[0],
                      q=data.joint_pos[0], dq=data.joint_vel[0], feet=data.body_pos_w[0, self.feet])
        return {k: v.detach().cpu().numpy().astype(np.float32).copy() for k, v in values.items()}

    def step(self, target):
        self.robot.set_joint_position_target(torch.as_tensor(target, device=self.sim.device).unsqueeze(0))
        self.robot.write_data_to_sim()
        before = float(self.sim.current_time)
        # With a 20 ms render interval, step(render=True) advances four physics
        # ticks inside Isaac Sim. ReLIC needs updated PD torque every 5 ms.
        self.sim.step(render=False)
        advance = float(self.sim.current_time) - before
        if not np.isclose(advance, DT, rtol=0, atol=1e-7):
            raise RuntimeError(f"Unexpected physics advance: {advance:.9f} s; expected {DT} s")
        self.robot.update(DT)

    def push(self, dvy):
        velocity = self.robot.data.root_vel_w.clone()
        velocity[:, 1] += dvy
        self.robot.write_root_velocity_to_sim(velocity)

    def metadata(self):
        robot = self.robot
        return dict(joint_names=robot.joint_names, body_names=robot.body_names,
                    total_mass=float(robot.root_physx_view.get_masses().sum()),
                    body_mass=robot.root_physx_view.get_masses()[0].tolist(),
                    joint_limits=robot.data.joint_pos_limits[0].tolist(),
                    inertia=robot.root_physx_view.get_inertias()[0].tolist(),
                    center_of_mass=robot.root_physx_view.get_coms()[0].tolist(),
                    state={k: v.tolist() for k, v in self.state().items()})

    def render(self):
        if not self.headless:
            before = float(self.sim.current_time)
            # Isaac Lab render() temporarily disables physics while updating
            # the viewport. The shared runner calls this once per policy step.
            self.sim.render()
            if abs(float(self.sim.current_time) - before) > 1e-7:
                raise RuntimeError("Viewport rendering advanced physics outside the control loop")

    def close(self):
        # Prevent uncontrolled stepping if Kit takes time to shut down.
        self.sim.pause()
