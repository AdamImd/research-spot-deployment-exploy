"""Offline closed-loop replay. The only time-varying input after reset is base velocity."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from spot_deploy.contracts import JOINTS, check_artifacts
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.sysid import extract, PAPER, validate_parameters, curves


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "simulation"


def dump(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False, default=str) + "\n")


def array(x):
    return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)


def configure(backend, params):
    """Use explicit Coulomb torque (Nm) for candidates in both engines, not PhysX dimensionless friction."""
    if params is not None:
        validate_parameters(params)
    isaac = hasattr(backend, "robot")
    if isaac:
        import torch
        from isaaclab.utils import LinearInterpolation

        robot = backend.robot
        knee = robot.actuators["spot_knee"]
        if params is not None:
            pos, neg = curves(params)
            for name, data in [("pos", pos), ("neg", neg)]:
                d = torch.tensor(data, device=backend.sim.device, dtype=torch.float32)
                setattr(knee, f"_{name}_torque_speed_data", d)
                setattr(
                    knee,
                    f"_{name}_torque_speed_limit",
                    LinearInterpolation(d[:, 0], d[:, 1], device=backend.sim.device),
                )
        r = dict(
            groups={
                k: dict(
                    names=list(v.joint_names),
                    kp=array(v.stiffness).tolist(),
                    kd=array(v.damping).tolist(),
                    effort_limit=[str(float(z)) for z in array(v.effort_limit).ravel()],
                    delay_steps=array(v.positions_delay_buffer.time_lags).tolist(),
                )
                for k, v in robot.actuators.items()
            },
            physx_friction=array(robot.data.joint_friction_coeff).tolist(),
            knee_speed_enabled=knee._enable_torque_speed_limit,
            pos_curve=array(knee._pos_torque_speed_data).tolist(),
            neg_curve=array(knee._neg_torque_speed_data).tolist(),
            knee_angle_table=knee.cfg.joint_parameter_lookup,
        )
    else:
        if params is not None:
            backend.pos_speed, backend.neg_speed = curves(params)
        r = dict(
            delay_steps=backend.delay,
            kp=backend.kp.tolist(),
            kd=backend.kd.tolist(),
            pos_curve=backend.pos_speed.tolist(),
            neg_curve=backend.neg_speed.tolist(),
            knee_angle_table=backend.table.tolist(),
            frictionloss=backend.model.dof_frictionloss[backend.dids].tolist(),
        )
    if params is not None:
        if isaac:
            for group, friction in (("spot_hip", params[0]), ("spot_knee", params[1])):
                act = robot.actuators[group]
                native = act.compute

                def damp(command, joint_pos, joint_vel, native=native, friction=friction):
                    cmd = native(command, joint_pos, joint_vel)
                    cmd.joint_efforts = cmd.joint_efforts - friction * torch.tanh(joint_vel / 0.05)
                    return cmd

                act.compute = damp
        else:
            coeff = np.array(
                [
                    params[1]
                    if n.endswith("_kn")
                    else params[0]
                    if n.endswith(("_hx", "_hy"))
                    else 0.0
                    for n in backend.contract.names
                ]
            )
            original = backend.torque
            backend.torque = lambda target: original(target) - coeff * np.tanh(
                backend.data.qvel[backend.dids] / 0.05
            )
    r.update(
        parameters=params,
        friction_implementation="candidate: explicit opposing torque applied to body after motor clipping; tanh(dq/0.05); baseline: native only",
    )
    return r


def run(args):
    sys.path[:0] = [str(SIM), str(SIM / "tools"), str(SIM / "source/relic")]
    from spot_relic_sim.contract import Contract, Policy, Command
    from spot_relic_sim.scenarios import Scenario
    from simulate_relic import sdk_state
    from debug_shadow import initialize_captured

    hw, init, meta = extract(args.trial, args.manifest)
    m = load_manifest(args.manifest)
    actor = make_policy(check_artifacts(args.manifest, m), m, args.manifest, device="cpu")
    actor.body_height = meta["height"]
    params = (
        None
        if args.parameters == "baseline"
        else PAPER
        if args.parameters == "paper"
        else json.loads(Path(args.parameters).read_text())
    )
    if params is not None:
        validate_parameters(params)
    scenario = Scenario(smoke=True, seed=101)
    if args.backend == "isaaclab":
        from spot_relic_sim.isaac_backend import IsaacBackend

        backend = IsaacBackend(args.output / "model", scenario, device="cpu", headless=True)
    else:
        from spot_relic_sim.mujoco_backend import MujocoBackend

        backend = MujocoBackend(
            Contract.load(SIM / "configs/articulation.json"), args.output / "model", scenario
        )
    try:
        if args.audit_actuators:
            if args.backend != "isaaclab":
                raise ValueError("native audit requires Isaac Lab")
            from spot_relic_sim.native_audit import audit_actuators

            dump(
                args.output / "native-audit.json",
                audit_actuators(backend, args.output / "audit-model"),
            )
        report = configure(backend, params)
        dump(args.output / "actuators.json", report)
        initialize_captured(backend, init, meta["height"], False)
        ctx = Command(
            arm=np.asarray(m.arm_stowed_positions), torso=np.array([0, 0, meta["height"]])
        )
        names = [s.replace("arm_", "arm0_") for s in backend.contract.names]
        ix = [JOINTS.index(n) for n in names]
        jx = [JOINTS.index(n) for n in m.actions.joint_order]
        previous = np.zeros(12, np.float32)
        ref = Policy()
        sim = []
        obs_err = act_err = target_err = 0.0
        fall = None
        for i, velocity in enumerate(hw["cmd"]):
            t = i * 0.02
            state, s = sdk_state(backend, t, 4 * i)
            inputs = actor.inputs(state, previous, np.zeros(3))
            inputs["velocity_command"] = velocity.astype(np.float32)[None]
            target, _, obs, raw = actor._evaluate_inputs(inputs, 0.0)
            ctx.velocity = velocity
            ref_obs = backend.contract.observation(s, ctx, previous)
            ref_raw = ref(ref_obs)
            obs_err = max(obs_err, float(np.max(np.abs(ref_obs - obs.ravel()))))
            act_err = max(act_err, float(np.max(np.abs(ref_raw - raw))))
            target_err = max(
                target_err, float(np.max(abs(backend.contract.targets(ref_raw, ctx) - target[ix])))
            )
            if obs_err > 1e-5 or act_err > 1e-4 or target_err > 3e-5:
                raise ValueError("independent native/Exploy parity failed")
            sim.append(
                (
                    np.asarray(state.positions)[jx],
                    np.asarray(state.velocities)[jx],
                    raw.copy(),
                    obs.ravel()[:3].copy(),
                )
            )
            for _ in range(4):
                backend.step(target[ix])
                measured = backend.state()
                if measured["position"][2] < 0.25 or -measured["gravity"][2] < np.cos(0.9):
                    fall = backend.physics_time
                    break
            if fall is not None:
                break
            previous = raw.copy()  # ideal receipt at next 5 ms packet, before next policy tick
        sim = {k: np.stack(x) for k, x in zip(("q", "dq", "a", "v"), zip(*sim))}
        np.savez_compressed(
            args.output / "paired.npz",
            **{"hw_" + k: v for k, v in hw.items()},
            **{"sim_" + k: v for k, v in sim.items()},
            sim_t=np.arange(len(sim["a"])) / 50,
        )
        dump(
            args.output / "result.json",
            dict(
                **meta,
                backend=args.backend,
                sim_samples=len(sim["a"]),
                fall_s=fall,
                max_parity_error=dict(obs=obs_err, action=act_err, target=target_err),
                hardware_access=False,
                physics_dt=0.005,
                policy_dt=0.02,
                seed=101,
                initial_velocity="zero; all joints and base",
                replay="nearest 50 Hz policy tick using verified hardware timestamp grid; ideal ACK",
                model_sha256=m.policy_sha256,
                action_names=m.actions.joint_order,
            ),
        )
    finally:
        backend.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ["trial", "manifest", "output"]:
        p.add_argument("--" + name, type=Path, required=True)
    p.add_argument(
        "--audit-actuators",
        action="store_true",
        help="compare native and ported actuators before configuring replay",
    )
    p.add_argument("--backend", choices=["isaaclab", "mujoco"], default="isaaclab")
    p.add_argument(
        "--parameters", default="baseline", help="baseline, paper, or path to eight parameters"
    )
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    app = None
    try:
        if args.backend == "isaaclab":
            from isaaclab.app import AppLauncher

            app = AppLauncher(
                headless=True,
                device="cpu",
                kit_args="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0 --/persistent/renderer/startupMessageDisplayed=true",
            ).app
        run(args)
    finally:
        if app:
            app.close(skip_cleanup=True)
