"""Check the shared contract against native upstream observation/action code."""
import importlib.util
from types import SimpleNamespace
import numpy as np
import torch
from isaaclab.envs import mdp
from .contract import ROOT, ARM, LEGS, C, Command


def audit(backend):
    device = backend.sim.device
    contract, robot = backend.contract, backend.robot
    path = ROOT / "source/relic/relic/tasks/loco_manipulation/mdp/actions/spot_joint_actions.py"
    spec = importlib.util.spec_from_file_location("upstream_spot_joint_actions", path)
    source = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(source)
    def tensor(values, dtype=torch.float32):
        return torch.tensor(values, dtype=dtype, device=device)
    term = SimpleNamespace(
        arm_joint_sub_goal=tensor(np.zeros((1, 7))), leg_joint_sub_goal=tensor(np.zeros((1, 12))),
        command_leg=tensor([False], torch.bool), command_leg_idxs=tensor([0], torch.long),
        command_leg_joint_idxs=tensor(np.arange(12).reshape(4, 3), torch.long),
        leg_joint_idxs=tensor([[contract.names.index(n) for n in C.LEG_JOINT_NAMES[leg]]
                              for leg in LEGS], torch.long))
    commands = {}
    env = SimpleNamespace(scene={"robot": robot}, num_envs=1, device=device,
                          command_manager=SimpleNamespace(get_term=lambda name: term,
                                                          get_command=lambda name: commands[name]),
                          action_manager=SimpleNamespace(action=tensor(np.zeros((1, 12)))))
    cfg = SimpleNamespace(asset_name="robot", joint_names=["[fh].*"], preserve_order=False,
                          scale=.2, offset=0., clip=None, debug_vis=False, use_default_offset=True,
                          arm_joint_names=list(ARM), leg_joint_names=C.LEG_JOINT_NAMES,
                          command_name="arm_leg_joint_base_pose")
    action_term = source.MixedPDArmMultiLegJointPositionAction(cfg, env)
    rng = np.random.default_rng(101)
    obs_error, target_error = 0., 0.
    for i in range(25):
        leg = [None, *LEGS][i % 5]
        command = Command(velocity=rng.uniform(-.3, .3, 3).astype(np.float32), leg=leg,
                          leg_pose=np.array([.12, 1.3, -2.4], dtype=np.float32))
        # Exercise every joint/velocity channel, with actual articulation state.
        q = contract.q0 + rng.uniform(-.03, .03, 19).astype(np.float32)
        dq = rng.uniform(-.5, .5, 19).astype(np.float32)
        robot.write_joint_state_to_sim(tensor(q[None]), tensor(dq[None]))
        root_velocity = tensor(rng.uniform(-.4, .4, (1, 6)))
        robot.write_root_velocity_to_sim(root_velocity)
        backend.sim.forward()
        robot.update(.005)
        previous = rng.normal(0, 2, 12).astype(np.float32)
        raw = rng.normal(0, 2, 12).astype(np.float32)
        env.action_manager.action = tensor(previous[None])
        commands["base_velocity"] = tensor(command.velocity[None])
        commands["arm_leg_joint_base_pose"] = tensor(command.observation()[None])
        actual = contract.observation(backend.state(), command, previous)
        reference = torch.cat((mdp.base_lin_vel(env), mdp.base_ang_vel(env), mdp.projected_gravity(env),
            mdp.generated_commands(env, "base_velocity"), mdp.generated_commands(env, "arm_leg_joint_base_pose"),
            mdp.joint_pos_rel(env), mdp.joint_vel_rel(env), mdp.last_action(env)), dim=1)[0].cpu().numpy()
        np.testing.assert_allclose(actual, reference, atol=1e-6, rtol=0)
        obs_error = max(obs_error, float(np.max(abs(actual - reference))))
        term.arm_joint_sub_goal[:] = tensor(command.arm[None])
        term.leg_joint_sub_goal[:] = tensor(command.observation()[None, 7:19])
        term.command_leg[:] = leg is not None
        term.command_leg_idxs[:] = 0 if leg is None else LEGS.index(leg)
        action_term.process_actions(tensor(raw[None]))
        reference_target = contract.q0.copy()
        reference_target[action_term._joint_ids] = action_term.processed_actions[0].cpu().numpy()
        reference_target[action_term._arm_joint_ids] = action_term.arm_processed_actions[0].cpu().numpy()
        actual_target = contract.targets(raw, command)
        np.testing.assert_allclose(actual_target, reference_target, atol=1e-6, rtol=0)
        target_error = max(target_error, float(np.max(abs(actual_target - reference_target))))
    return dict(passed=True, cases=25, observation_max_error=obs_error, target_max_error=target_error,
                reference_action_class="upstream MixedPDArmMultiLegJointPositionAction",
                reference_observations="isaaclab.envs.mdp, released 84-D term order",
                modes=["four", *LEGS])


def audit_actuators(backend, output):
    """Compare NumPy port against native actuator classes, including delays."""
    from isaaclab.utils.types import ArticulationActions
    from .mujoco_backend import MujocoBackend
    from .scenarios import scenario
    rng = np.random.default_rng(102)
    port = MujocoBackend(backend.contract, output, scenario("nominal", "four", 101))
    reports = []
    cases = []
    for delay in (0, 3):
        s = scenario("delay" if delay else "nominal", "four", 101)
        port.reset(s)
        actuators = []
        for original in backend.robot.actuators.values():
            cfg = original.cfg.copy()
            cfg.min_delay = cfg.max_delay = delay
            ids = original.joint_indices
            actuator = type(original)(cfg=cfg, joint_names=original.joint_names, joint_ids=ids,
                num_envs=1, device="cpu", stiffness=original.stiffness.cpu(), damping=original.damping.cpu(),
                armature=original.armature.cpu(), friction=original.friction.cpu(),
                effort_limit=original.effort_limit.cpu(), velocity_limit=original.velocity_limit.cpu())
            actuator.reset(None)
            actuators.append((ids, actuator))
        error = 0.
        limits = port.model.jnt_range[port.joints]
        for _ in range(200):
            q = rng.uniform(limits[:, 0], limits[:, 1]).astype(np.float32)
            dq = rng.uniform(-40, 40, 19).astype(np.float32)
            target = (backend.contract.q0 + rng.normal(0, 2, 19)).astype(np.float32)
            port.data.qpos[port.qids] = q
            port.data.qvel[port.dids] = dq
            actual = port.torque(target)
            reference = np.zeros(19, dtype=np.float32)
            for ids, actuator in actuators:
                zeros = torch.zeros((1, len(actuator.joint_names)))
                control = ArticulationActions(joint_positions=torch.tensor(target[ids][None]),
                                              joint_velocities=zeros.clone(), joint_efforts=zeros.clone())
                answer = actuator.compute(control, torch.tensor(q[ids][None]), torch.tensor(dq[ids][None]))
                reference[ids] = answer.joint_efforts[0].cpu().numpy()
            np.testing.assert_allclose(actual, reference, atol=1e-4, rtol=1e-6)
            error = max(error, float(np.max(abs(actual - reference))))
            cases.append(np.concatenate(([delay], q, dq, target, actual, reference)))
        reports.append(dict(delay_steps=delay, cases=200, max_absolute_torque_error=error))
    from pathlib import Path
    np.savez_compressed(Path(output).parent / "actuator-cases.npz", cases=np.stack(cases))
    return dict(passed=True, cases=400, units="N m", results=reports,
                case_columns="delay, q[19], dq[19], target[19], mujoco_torque[19], native_torque[19]",
                seed=102,
                scope="All 19 joints; PD, knee angle/speed saturation, target delay including warmup")
