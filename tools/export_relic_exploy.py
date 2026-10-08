"""Export native ReLIC observations, released actor and all 19 joint targets with Exploy.

Simulation only. The resulting policy-step graph runs at 50 Hz inside the existing
200 Hz rollout. History is an explicit input owned by that rollout, not by Exploy's
automatic memory feedback or decimation controller.
"""

import argparse
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys

from spot_deploy.contracts import sha256
from spot_deploy.records import atomic_json, source_identity
from spot_deploy.relic_policy import CHECKPOINT_SHA256

EXPLOY_COMMIT = "05f9dc3b2e589abdae8de942e8c6749e7a123c88"
SCRIPT_SHA256 = "14ed0614f507e1330a5c66af1615f27558b58b6c999443b42d1bb6c3d45ae3b4"
META_KEY = "spot_deploy.relic_exploy"


def serial(value):
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [serial(v) for v in value]
    if callable(value):
        return f"{value.__module__}.{value.__qualname__}"
    if hasattr(value, "tolist"):
        return value.tolist()
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def configure(source, output):
    import isaaclab.sim as sim_utils
    from relic.tasks.loco_manipulation.config.spot.spot_env_cfg import SpotInterlimbEnvCfg_PLAY
    from spot_relic_sim.contract import ARM, defaults

    cfg = SpotInterlimbEnvCfg_PLAY()
    cfg.seed = 101
    cfg.scene.num_envs = 1
    cfg.sim.device = "cpu"
    cfg.scene.robot.spawn.usd_dir = str(output / "model")
    cfg.scene.robot.spawn.force_usd_conversion = True
    cfg.scene.robot.spawn.joint_drive.target_type = "none"
    cfg.scene.robot.spawn.joint_drive.gains.stiffness = 0.
    cfg.scene.robot.spawn.joint_drive.gains.damping = 0.
    cfg.scene.terrain.visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(.2, .2, .2))
    terrain = cfg.scene.terrain.terrain_generator
    terrain.num_rows = terrain.num_cols = 1
    terrain.sub_terrains = {"flat": terrain.sub_terrains["flat"].copy()}
    terrain.sub_terrains["flat"].proportion = 1.
    cfg.events.add_base_mass = cfg.events.add_arm_mass = None
    cfg.events.external_force_torque = cfg.events.push_robot = None
    cfg.events.reset_base.params["pose_range"] = {}
    cfg.events.reset_base.params["velocity_range"] = {}
    cfg.events.reset_robot_joints.params["position_range"] = (0., 0.)
    cfg.events.reset_robot_joints.params["velocity_range"] = (0., 0.)
    for actuator in cfg.scene.robot.actuators.values():
        actuator.min_delay = actuator.max_delay = 0
    cfg.observations.policy.enable_corruption = False
    for term in vars(cfg.observations.policy).values():
        if hasattr(term, "noise"):
            term.noise = None
    velocity = cfg.commands.base_velocity
    velocity.heading_command = False
    velocity.rel_standing_envs = 1.
    velocity.ranges.lin_vel_x = velocity.ranges.lin_vel_y = velocity.ranges.ang_vel_z = (0., 0.)
    command = cfg.commands.arm_leg_joint_base_pose
    command.command_which_leg = -1
    command.command_range_roll = command.command_range_pitch = (0., 0.)
    command.command_range_height = (.55, .55)
    arm = defaults(ARM).tolist()
    command.command_range_arm_joint = (arm, arm)
    return cfg


def export(args):
    import gymnasium as gym
    import numpy as np
    import onnx
    import torch
    import relic.tasks  # noqa: F401
    from exploy.exporter.core.components import Group, Input, Output
    from exploy.exporter.core.exporter import export_environment_as_onnx
    from exploy.exporter.frameworks.isaaclab.env import IsaacLabExportableEnvironment
    from exploy.exporter.frameworks.isaaclab.utils import get_articulation_actuator_gains

    source = args.simulation_source.resolve()
    asset = source / "source/relic/relic/assets/spot"
    script = asset / "pretrained/policy.pt"
    if sha256(script) != SCRIPT_SHA256 or sha256(asset / "pretrained/policy.onnx") != CHECKPOINT_SHA256:
        raise ValueError("Released checkpoint hashes do not match")
    direct = json.loads(importlib.metadata.distribution("exploy").read_text("direct_url.json"))
    if direct.get("vcs_info", {}).get("commit_id") != EXPLOY_COMMIT:
        raise ValueError("Install the pinned Exploy commit, not an untracked editable source")
    cfg = configure(source, args.output)
    atomic_json(args.output / "configuration.json", serial(cfg.to_dict()))
    env = gym.make("Isaac-Spot-Interlimb-Play-v0", cfg=cfg).unwrapped
    wrapped = None
    try:
        env.reset(seed=101)
        robot = env.scene["robot"]
        command = env.command_manager.get_term("arm_leg_joint_base_pose")
        if bool(command.command_leg.any()):
            raise ValueError("Export must use the four-foot standing specialization")
        if env.step_dt != .02 or cfg.sim.dt != .005:
            raise ValueError("Native ReLIC timing differs from 50/200 Hz")
        # ReLIC has extra arm outputs and a custom command. Register their stored
        # tensors explicitly so no height/arm command is frozen during tracing.
        class StandingEnvironment(IsaacLabExportableEnvironment):
            def prepare_export(self):
                pass  # Commands are explicit inputs; do not resample during tracing.

        wrapped = StandingEnvironment(env)
        context = wrapped.context_manager()
        getters = {
            "base_quaternion": lambda: robot.data.root_quat_w,
            "base_linear_velocity": lambda: robot.data.root_lin_vel_b,
            "base_angular_velocity": lambda: robot.data.root_ang_vel_b,
            "joint_positions": lambda: robot.data.joint_pos,
            "joint_velocities": lambda: robot.data.joint_vel,
            "arm_command": lambda: command.arm_joint_sub_goal,
            "torso_command": lambda: command.torso_roll_pitch_height_goal,
            "velocity_command": lambda: env.command_manager.get_term("base_velocity").vel_command_b,
            "previous_actions": lambda: env.action_manager.action,
        }
        for name, getter in getters.items():
            context.add_component(Input(name=name, get_from_env_cb=getter))
        gains = get_articulation_actuator_gains(robot)
        context.add_group(Group(
            name="output.joint_targets.robot.all",
            metadata={"type": "joint_targets", "names": robot.joint_names,
                      "stiffness": [gains[n]["stiffness"] for n in robot.joint_names],
                      "damping": [gains[n]["damping"] for n in robot.joint_names]},
            items=[Output(name="pos", get_from_env_cb=lambda: robot.data.joint_pos_target)],
        ))
        released = torch.jit.load(str(script), map_location="cpu").eval()
        # TorchScript modules cannot be nested in Exploy's legacy trace. Rebuild
        # the hash-pinned release's exact Linear/ELU architecture, copy every
        # parameter strictly, and compare both published formats before export.
        if released.normalizer.original_name != "Identity":
            raise ValueError("Released normalization changed")
        actor = torch.nn.Sequential(
            torch.nn.Linear(84, 512), torch.nn.ELU(),
            torch.nn.Linear(512, 256), torch.nn.ELU(),
            torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 12),
        ).eval()
        actor.load_state_dict({k.removeprefix("actor."): v for k, v in released.state_dict().items()},
                              strict=True)
        import onnxruntime as ort
        original = ort.InferenceSession(str(asset / "pretrained/policy.onnx"),
                                        providers=["CPUExecutionProvider"])
        samples = np.random.default_rng(101).normal(0, .3, (32, 84)).astype(np.float32)
        with torch.inference_mode():
            rebuilt = actor(torch.from_numpy(samples)).numpy()
            scripted = released(torch.from_numpy(samples)).numpy()
        published = np.concatenate([original.run(["actions"], {"obs": row[None]})[0]
                                    for row in samples])
        np.testing.assert_allclose(rebuilt, scripted, rtol=0, atol=1e-5)
        np.testing.assert_allclose(rebuilt, published, rtol=0, atol=1e-4)
        atomic_json(args.output / "actor-parity.json", {
            "seed": 101, "samples": 32,
            "torchscript_max_abs_error": float(np.max(np.abs(rebuilt - scripted))),
            "original_onnx_max_abs_error": float(np.max(np.abs(rebuilt - published))),
            "weights_changed": False})
        export_environment_as_onnx(wrapped, actor, str(args.output), "native.onnx",
                                   model_source={"checkpoint_sha256": SCRIPT_SHA256,
                                                 "exploy_commit": EXPLOY_COMMIT}, ir_version=10)
        # The host owns both scheduling and acknowledgement. Use Exploy's full
        # policy-step graph, not its auto-feedback/decimation wrapper.
        policy = args.output / "policy.onnx"
        shutil.copy2(args.output / "debug/native_default.onnx", policy)
        model = onnx.load(policy)
        model.ir_version = 10
        metadata = dict(contract="relic-exploy-v1", exploy_commit=EXPLOY_COMMIT,
            source_onnx_sha256=CHECKPOINT_SHA256, source_torchscript_sha256=SCRIPT_SHA256,
            joint_names=[n.replace("arm_", "arm0_") for n in robot.joint_names],
            action_joint_names=[n.replace("arm_", "arm0_") for n in
                                env.action_manager.get_term("joint_pos")._joint_names],
            gains={n.replace("arm_", "arm0_"): gains[n] for n in robot.joint_names},
            root_com_b=robot.data._com_root_pos_b[0].tolist(),
            policy_hz=50, stream_hz=200, graph="policy-step", mode="four-foot-standing",
            history="external-acknowledged-raw-actions; zero at activation",
            target_output="output.joint_targets.robot.all.pos")
        item = model.metadata_props.add()
        item.key, item.value = META_KEY, json.dumps(metadata, sort_keys=True)
        onnx.checker.check_model(model)
        actual = {v.name for v in model.graph.input}
        if actual != set(getters):
            raise ValueError(f"Export input mismatch: expected {sorted(getters)}, got {sorted(actual)}")
        onnx.save(model, policy)
        atomic_json(args.output / "graph-contract.json", metadata)
        atomic_json(args.output / "export.json", {
            "policy_sha256": sha256(policy), "source": source_identity(),
            "simulation_commit": subprocess.check_output(
                ["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip(),
            "simulation_dirty": bool(subprocess.check_output(
                ["git", "-C", str(source), "status", "--porcelain"], text=True).strip()),
            "packages": {n: importlib.metadata.version(n) for n in
                ("exploy", "torch", "onnx", "onnxruntime", "onnxscript", "numpy", "isaaclab")},
            "source_policy_sha256": CHECKPOINT_SHA256, "source_script_sha256": SCRIPT_SHA256,
            "configuration_sha256": sha256(args.output / "configuration.json"),
            "hardware_access": False,
        })
        np.testing.assert_array_equal(env.action_manager.action.shape, [1, 12])
        print(json.dumps({"exported": str(policy), "sha256": sha256(policy)}), flush=True)
    finally:
        if wrapped:
            wrapped.cleanup()
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path[:0] = [str(args.simulation_source), str(args.simulation_source / "source/relic")]
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, device="cpu",
                      kit_args="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0").app
    try:
        export(args)
        atomic_json(args.output / "COMPLETE.json", {"status": "completed", "artifacts": {
            str(p.relative_to(args.output)): sha256(p) for p in args.output.glob("*.json*")}})
    except BaseException:
        import os
        import traceback
        (args.output / "ERROR.txt").write_text(traceback.format_exc())
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        # Kit close can replace a pending exception with exit(0). Preserve the
        # failed status of this isolated exporter process for the supervisor.
        os._exit(1)
    finally:
        app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
