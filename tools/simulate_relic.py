"""Exercise the actual ReLIC rollout core against a simulator; never access Spot.

Use a frozen deployment source and frozen simulator source. Simulation limits
are explicitly tagged and cannot authorize physical execution.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

from spot_deploy.contracts import JOINTS, ContractError, Envelope, State, check_artifacts, load
from spot_deploy.policy import rotation
from spot_deploy.records import atomic_json, runtime_packages, verify_run
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_policy import ROOT_COM_B
from spot_deploy.relic_rollout import ReLICRollout
from spot_deploy.simulation_demo import ThreeLegPlan, SimulationThreeLegPolicy, demo_metrics


class Log:
    def __init__(self, directory):
        self.directory = directory
        self.stream = (directory / "events.jsonl").open("x")
        self.last_policy = None

    def event(self, kind, **fields):
        self.stream.write(json.dumps(dict(event=kind, time=datetime.now(timezone.utc).isoformat(),
                                         **fields), allow_nan=False) + "\n")
        if kind == "policy":
            self.last_policy = fields


def sdk_state(backend, t, key):
    s = backend.state()
    names = [n.replace("arm_", "arm0_") for n in backend.contract.names]
    ids = [names.index(n) for n in JOINTS]
    quat = (backend.data.qpos[3:7] if hasattr(backend, "data") else
            backend.robot.data.root_quat_w[0].cpu().numpy())
    state = State(robot_time_s=1000 + t, received_monotonic_s=10 + t,
                  positions=s["q"][ids].tolist(), velocities=s["dq"][ids].tolist(),
                  loads=[0.] * 19, odom_quaternion_wxyz=quat.tolist(),
                  linear_velocity_odom=[0.] * 3, angular_velocity_odom=[0.] * 3,
                  body_position_odom=s['position'].tolist(), body_pose_robot_time_s=1000+t,
                  last_command_key=key, last_command_received_robot_s=1000 + t)
    r = rotation(state)
    state.linear_velocity_odom = (r @ (s["linear_velocity"]
                                 - np.cross(s["angular_velocity"], ROOT_COM_B))).tolist()
    state.angular_velocity_odom = (r @ s["angular_velocity"]).tolist()
    torque = (backend.data.qfrc_applied[backend.dids] if hasattr(backend, "data") else
              backend.robot.data.applied_torque[0].cpu().numpy())
    state.loads = np.asarray(torque)[ids].tolist()
    return state, s


def run(args):
    if not verify_run(args.capture):
        raise ValueError("capture failed integrity verification")
    sys.path[:0] = [str(args.simulation_source), str(args.simulation_source / "tools")]
    from spot_relic_sim.contract import Command as SimCommand, Contract, Policy
    from spot_relic_sim.scenarios import Scenario
    from spot_relic_sim.settling import standing_window
    from debug_shadow import initialize_captured

    manifest, envelope = load_manifest(args.manifest), load(args.envelope, Envelope)
    if envelope.scope != "simulation":
        raise ValueError("simulator requires an explicitly simulation-scoped envelope")
    if not envelope.transition_s < args.duration <= envelope.max_duration_s:
        raise ValueError("duration must include preparation and remain within envelope")
    policy = make_policy(check_artifacts(args.manifest, manifest), manifest, args.manifest,
                         device=args.policy_device)
    demo = None
    if getattr(args, 'three_leg_plan', None):
        plan = load(args.three_leg_plan, ThreeLegPlan)
        if plan.duration_s > args.duration:
            raise ValueError('simulation duration cannot end before the three-leg sequence')
        demo = SimulationThreeLegPolicy(policy,
            args.simulation_source/'source/relic/relic/assets/spot/pretrained/policy.onnx',plan)
        policy = demo
    prewarm_times = []
    for _ in range(args.prewarm_inference):
        began = time.monotonic()
        policy.warmup()
        prewarm_times.append(time.monotonic() - began)
    height = json.loads((args.capture / "initial-height.json").read_text())
    initial = json.loads((args.capture / "initial-state.json").read_text())
    dt = 1 / manifest.stream_hz
    if args.backend != "mujoco" and args.mujoco_noslip_iterations:
        raise ValueError("MuJoCo friction-drift setting cannot be used with Isaac")
    if dt != .005:
        raise ValueError("this shared-core physics test requires 200 Hz commands")
    scenario = Scenario(name="shared-ReLIC-rollout-core", duration=args.duration, smoke=True, seed=101)
    log, backend = Log(args.output), None
    try:
        if args.backend == "mujoco":
            from spot_relic_sim.mujoco_backend import MujocoBackend
            backend = MujocoBackend(Contract.load(args.simulation_source / "configs/articulation.json"),
                                    args.output / "model", scenario)
            backend.model.opt.noslip_iterations = args.mujoco_noslip_iterations
        else:
            from spot_relic_sim.isaac_backend import IsaacBackend
            backend = IsaacBackend(args.output / "model", scenario,
                                   device=args.physics_device, headless=True)
        initialize_captured(backend, initial, height["height_m"], nominal_arm=False)
        core = ReLICRollout(policy, envelope, log)
        state, _ = sdk_state(backend, 0., 0)
        # The already-standing capture substitutes for stock stand. A supported
        # command is accepted before activation; no physics advances in this handshake.
        state.received_monotonic_s = 9.995
        state.robot_time_s = 999.995
        core.initialize(state, height, 9.995, 999.995)
        command = core.command(state, 9.995, 999.995)
        key = command.key
        core.activate(10.)
        inverse = [JOINTS.index(n.replace("arm_", "arm0_")) for n in backend.contract.names]
        reference_policy = Policy()
        ref_command = SimCommand(arm=np.asarray(manifest.arm_stowed_positions, dtype=np.float32),
                                torso=np.array([0., 0., policy.body_height], dtype=np.float32))
        rows, reason, failure = [], "completed", None
        max_obs_error = max_action_error = max_target_error = 0.
        compute_times = []
        config = dict(backend=args.backend, duration_s=args.duration, preparation_s=envelope.transition_s,
                      policy_device=args.policy_device, physics_device=args.physics_device,
                      providers=policy.runner.session.get_providers(),
                      provider_options=policy.runner.session.get_provider_options(),
                      prewarm_inference=args.prewarm_inference, prewarm_times_s=prewarm_times,
                      policy_hz=manifest.policy_hz, command_hz=manifest.stream_hz,
                      mujoco_noslip_iterations=args.mujoco_noslip_iterations,
                      simulation_joint_names=list(backend.contract.names),
                      seed=101, hardware_access=False, manifest=manifest.model_dump(),
                      envelope=envelope.model_dump(),
                      initial_state="captured physical standing state; zero simulated velocity",
                      demonstration=demo.sequence.plan.model_dump() if demo else None,
                      policy_execution=('released raw actor with native selected-leg override'
                                        if demo else 'bound deployment adapter'),
                      baseline_manifest_used_for_numeric_guards_only=bool(demo),
                      stock_stand_simulated=False, acknowledgements="ideal, one command tick later",
                      physics_actuators="released simulated effort saturation; hardware equivalence unverified")
        atomic_json(args.output / "configuration.json", config)
        atomic_json(args.output / "run.json", {"mode": "relic-simulation", "hardware_access": False})
        atomic_json(args.output / "source.json", {
            "deployment_source": str(Path(__file__).resolve().parents[1]),
            "simulation_source": str(args.simulation_source),
            "source_hashes": {
                str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                for base in (Path(__file__).resolve().parents[1] / "src/spot_deploy",
                             args.simulation_source / "spot_relic_sim")
                for p in sorted(base.glob("*.py"))},
            "packages": runtime_packages(),
            "capture_complete_sha256": hashlib.sha256((args.capture / "COMPLETE.json").read_bytes()).hexdigest(),
        })
        timing_rows = []
        loop_began = time.monotonic()
        with (args.output / "rollout.jsonl").open("x") as stream:
            for tick in range(round(args.duration / dt) + 1):
                t = tick * dt
                tick_began = time.monotonic()
                state, sim = sdk_state(backend, t, key)
                began = time.monotonic()
                count = core.policy_samples
                if demo:
                    previous_phase = demo.sequence.phase
                    demo.update(state,t)
                    if demo.sequence.phase != previous_phase:
                        log.event('demo_phase',phase=demo.sequence.phase,simulation_time_s=t)
                try:
                    command = core.command(state, 10 + t, 1000 + t)
                except ContractError as exc:
                    reason, failure = "guard_stop", str(exc)
                    log.event("first_failure", reason=failure, simulation_time_s=t,
                              state=state.model_dump())
                    break
                compute_times.append(time.monotonic() - began)
                core_finished = time.monotonic()
                if core.policy_samples != count:
                    recorded = log.last_policy
                    ref_command.velocity = np.asarray(recorded['velocity_command'], dtype=np.float32)
                    if demo:
                        ref_command.leg = demo.sequence.leg
                        ref_command.leg_pose = demo.sequence.pose.copy()
                        recorded['leg_command'] = demo.command.tolist()
                        log.event('demo_prediction',simulation_time_s=t,
                                  phase=demo.sequence.phase,leg_command=demo.command.tolist(),
                                  targets=recorded['targets'], raw_actions=recorded['raw_actions'],
                                  state=state.model_dump())
                    obs = backend.contract.observation(sim, ref_command,
                                                       np.asarray(recorded["observations"][-12:]))
                    action = reference_policy(obs)
                    target = backend.contract.targets(action, ref_command)
                    max_obs_error = max(max_obs_error,
                                        float(np.max(np.abs(obs - recorded["observations"]))))
                    max_action_error = max(max_action_error,
                                           float(np.max(np.abs(action - recorded["raw_actions"]))))
                    max_target_error = max(max_target_error,
                                           float(np.max(np.abs(target - np.asarray(recorded["targets"])[inverse]))))
                    if max_obs_error > 1e-5 or max_action_error > 1e-4 or max_target_error > 3e-5:
                        raise AssertionError("shared-core observation/action parity failed")
                audit_finished = time.monotonic()
                row = dict(time=max(0., t - envelope.transition_s), trial_time=t, phase=core.phase,
                           height=float(sim["position"][2]),
                           tilt=float(np.arccos(np.clip(-sim["gravity"][2], -1, 1))),
                           demo_phase=demo.sequence.phase if demo else None,
                           leg_command=demo.command.tolist() if demo else None,
                           root_quaternion_wxyz=state.odom_quaternion_wxyz,
                           preceding_applied_torque_Nm=state.loads,
                           pd_torque_Nm=(np.asarray(manifest.gains.kp)
                               * (np.asarray(command.positions) - state.positions)
                               - np.asarray(manifest.gains.kd) * state.velocities
                               + command.feedforward).tolist(),
                           **{k: v.tolist() for k, v in sim.items()},
                           positions=list(command.positions), feedforward=list(command.feedforward))
                rows.append(row)
                stream.write(json.dumps(row, allow_nan=False) + "\n")
                log.event("command", key=command.key, positions=list(command.positions),
                          feedforward=list(command.feedforward), phase=core.phase,
                          state=state.model_dump(), simulation_time_s=t,
                          end_robot_time_s=command.end_robot_time_s)
                recorded_at = time.monotonic()
                terminal = row["height"] < .25 or row["tilt"] > .9
                if terminal:
                    reason = "fall"
                # Equivalent PD+feedforward before the same simulated saturation;
                # the physical encoder transmits q and feedforward separately.
                effective = (np.asarray(command.positions)
                             + np.asarray(command.feedforward) / manifest.gains.kp)
                if not terminal and tick != round(args.duration / dt):
                    backend.step(effective[inverse])
                    if args.physics_device.startswith("cuda"):
                        # Include completion of GPU physics, not just kernel dispatch.
                        import torch
                        torch.cuda.synchronize()
                finished = time.monotonic()
                timing_rows.append(dict(tick=tick, simulation_time_s=t,
                    state_s=began - tick_began, core_s=core_finished - began,
                    parity_s=audit_finished - core_finished,
                    recording_s=recorded_at - audit_finished,
                    physics_s=finished - recorded_at, total_s=finished - tick_began))
                key = command.key
                if terminal:
                    break
        loop_wall_s = time.monotonic() - loop_began
        with (args.output / "timing.jsonl").open("x") as timing:
            for timing_row in timing_rows:
                timing.write(json.dumps(timing_row) + "\n")
        policy_rows = [r for r in rows if r["phase"] == "policy"]
        late = [r for r in policy_rows if r["time"] >= policy_rows[-1]["time"] - 10 - 1e-8]
        stable, metrics = standing_window(late) if late else (False, None)
        result = dict(reason=reason, failure=failure, trial_duration_s=t,
                      policy_duration_s=max(0., t - envelope.transition_s),
                      policy_samples=core.policy_samples, stable_final_window=bool(reason == "completed" and stable),
                      late_window_metrics=metrics, max_observation_error=max_obs_error,
                      max_raw_action_error=max_action_error, max_target_error_rad=max_target_error,
                      compute_p99_s=float(np.quantile(compute_times, .99)) if compute_times else None,
                      compute_max_s=max(compute_times) if compute_times else None,
                      loop_wall_s=loop_wall_s, real_time_factor=t / loop_wall_s,
                      timing={k: dict(p50_s=float(np.quantile([r[k] for r in timing_rows], .5)),
                                     p99_s=float(np.quantile([r[k] for r in timing_rows], .99)),
                                     max_s=max(r[k] for r in timing_rows),
                                     over_5ms=sum(r[k] > .005 for r in timing_rows))
                              for k in ("state_s", "core_s", "parity_s", "recording_s", "physics_s", "total_s")}
                             if timing_rows else {},
                      hardware_qualified=False, hardware_access=False,
                      three_leg_demo=demo_metrics(rows,demo.sequence.plan) if demo and rows else None,
                      walking_completed=core.motion.completed if core.motion else None,
                      walking=core.motion.progress if core.motion else None)
        if demo and result['three_leg_demo']:
            result['three_leg_demo']['passed'] = bool(reason == 'completed'
                and result['stable_final_window']
                and result['three_leg_demo']['pose_sequence_finished']
                and result['three_leg_demo']['clearance_proxy_passed'])
        atomic_json(args.output / "result.json", result)
        return result
    finally:
        log.stream.close()
        if backend:
            backend.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation-source", required=True, type=Path)
    parser.add_argument("--capture", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--envelope", required=True, type=Path)
    parser.add_argument("--backend", choices=["mujoco", "isaaclab"], required=True)
    parser.add_argument("--duration", type=float, default=60.)
    parser.add_argument("--policy-device", choices=["cpu", "cuda"], default="cpu")
    parser.add_argument("--physics-device", choices=["cpu", "cuda:0"], default="cpu")
    parser.add_argument("--prewarm-inference", type=int, choices=[0, 100], default=0,
                        help="Discard zero-input inferences before activation; history stays zero")
    parser.add_argument("--mujoco-noslip-iterations", type=int, choices=[0, 10], default=0,
                        help="Explicit friction-drift diagnostic; default preserves the baseline")
    parser.add_argument('--three-leg-plan',type=Path,
                        help='Offline-only raw actor front-left lift/hold/return; never a live manifest')
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.backend == "mujoco" and args.physics_device != "cpu":
        parser.error("this MuJoCo backend uses CPU physics")
    if args.policy_device == "cuda":
        # Use the isolated Isaac-compatible CUDA libraries, never modify the host.
        import torch  # noqa: F401
    args.output.mkdir(parents=True, exist_ok=False)
    app = None
    try:
        if args.backend == "isaaclab":
            from isaaclab.app import AppLauncher
            app = AppLauncher(headless=True, device=args.physics_device,
                              kit_args="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0").app
        result = run(args)
        atomic_json(args.output / "COMPLETE.json", {"status": "completed", "reason": result["reason"],
                    "artifacts": {str(p.relative_to(args.output)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in args.output.glob("*.json*")}})
        print(json.dumps(result), flush=True)
    except BaseException:
        import traceback
        (args.output / "ERROR.txt").write_text(traceback.format_exc())
        raise
    finally:
        if app:
            app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
