"""Real-time, stowed-arm ReLIC joystick trial in MuJoCo. No robot client."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from joystick_relic import prepare
from room_array import load_array
from simulate_relic import Log, sdk_state
from spot_deploy.contracts import JOINTS, Envelope, check_artifacts, load, sha256
from spot_deploy.joystick_teleop import DemoJoystick, JoystickTeleopPlan
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_rollout import ReLICRollout

ROOT = Path(__file__).resolve().parents[1]


def run(args):
    sys.path[:0] = [str(ROOT / "simulation"), str(ROOT / "simulation/tools")]
    from debug_shadow import initialize_captured
    from spot_relic_sim.contract import Contract
    from spot_relic_sim.mujoco_backend import MujocoBackend
    from spot_relic_sim.scenarios import Scenario

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    atomic_json(out / "room-array.json", load_array())
    print("Recording: " + str(out), flush=True)
    capture = ROOT / "fixtures/standing-capture"
    if not verify_run(capture):
        raise ValueError("standing fixture integrity check failed")
    if args.manifest:
        mp = args.manifest.resolve()
        m = load_manifest(mp)
        if m.task != "joystick-arm":
            raise ValueError("expected a joystick-arm manifest")
        plan = m.teleop
    else:
        plan = load(args.profile, JoystickTeleopPlan)
        m = prepare(ROOT / "policies/relic-exploy-standing/manifest.json", plan, out / "policy")
        mp = out / "policy/manifest.json"
    if plan.study_commands is not None:
        raise ValueError("manual simulator requires a manual joystick profile")
    env = load(ROOT / "fixtures/relic-direct-simulation/envelope.json", Envelope)
    if env.scope != "simulation" or not 0 < args.duration <= min(
        env.max_duration_s, plan.duration_s
    ):
        raise ValueError("duration exceeds simulation/profile limit")
    actor = make_policy(check_artifacts(mp, m), m, mp)
    actor.warmup()
    b = MujocoBackend(
        Contract.load(ROOT / "simulation/configs/articulation.json"), out / "model", Scenario()
    )
    b.model.opt.noslip_iterations = 10
    initial = json.loads((capture / "initial-state.json").read_text())
    height = json.loads((capture / "initial-height.json").read_text())
    initialize_captured(b, initial, height["height_m"], False)
    start_pose = b.data.qpos.tolist()
    order = [JOINTS.index(n.replace("arm_", "arm0_")) for n in b.contract.names]
    log = Log(out)
    render_log = (out / "renderer.log").open("w")
    renderer = None
    source = None
    tick = 0
    result = dict(
        success=False,
        robot_access=False,
        policy_hz=50,
        command_hz=200,
        policy_sha256=m.policy_sha256,
        calibration_sha256=sha256(out / "room-array.json"),
        startup="captured standing fixture, zero simulated velocity, ideal ACK",
        duration_s=args.duration,
    )
    try:
        renderer = subprocess.Popen(
            [sys.executable, str(ROOT / "tools/room_cameras.py"), str(out)],
            stdout=render_log,
            stderr=subprocess.STDOUT,
            env=dict(os.environ, MUJOCO_GL="egl"),
        )
        source = DemoJoystick(plan, args.finish_fd)
        result["usb_device"] = source.link.metadata
        core = ReLICRollout(actor, env, log, source)
        state, _ = sdk_state(b, -0.005, 0)
        core.initialize(state, height, 9.995, 999.995)
        key = core.command(state, 9.995, 999.995).key
        core.activate(10.0)
        start = previous = time.monotonic()
        deadline = start
        for tick in range(round(args.duration * 200)):
            now = time.monotonic()
            if now - previous > plan.input_timeout_s:
                raise ValueError("wall-clock USB polling gap exceeded")
            previous = now
            t = tick / 200
            state, s = sdk_state(b, t, key)
            command = core.command(state, 10 + t, 1000 + t)
            demand = np.asarray(command.positions) + np.asarray(command.feedforward) / m.gains.kp
            b.step(demand[order])
            key = command.key
            if (
                not np.isfinite(b.data.qpos).all()
                or s["position"][2] < 0.25
                or -s["gravity"][2] < np.cos(0.9)
            ):
                raise ValueError("invalid state or simulated fall")
            if tick % 20 == 0 or core.teleop.completed:
                atomic_json(
                    out / "view-state.json",
                    dict(
                        tick=tick,
                        seconds=t,
                        sampled_at=now,
                        qpos=b.data.qpos.tolist(),
                        start_qpos=start_pose,
                    ),
                )
                print("Velocity: " + json.dumps(core.velocity_command), flush=True)
                print("Monitor: " + json.dumps(source.monitor), flush=True)
                if renderer.poll() is not None:
                    raise RuntimeError("room renderer exited; see renderer.log")
            if core.teleop.completed:
                break
            deadline = max(now, deadline) + 0.005
            time.sleep(max(0, deadline - time.monotonic()))
        result.update(
            success=True,
            finish="operator" if core.teleop.completed else "duration",
            policy_samples=core.policy_samples,
            elapsed_wall_s=time.monotonic() - start,
        )
    except BaseException as exc:
        result.update(error=str(exc))
        raise
    finally:
        if source is not None:
            source.close()
        log.stream.close()
        b.close()
        (out / "view-done").touch()
        if renderer is not None:
            try:
                renderer.wait(timeout=15)
            except subprocess.TimeoutExpired:
                renderer.kill()
                renderer.wait()
            if renderer.returncode or not (out / "views/frame.json").is_file():
                result.update(success=False, renderer_error="see renderer.log")
        render_log.close()
        atomic_json(out / "result.json", dict(**result, ticks=tick + 1))
    if not result["success"]:
        raise RuntimeError("camera simulation did not complete; see result.json")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", type=Path, help="Optional frozen manual joystick actor/profile")
    p.add_argument("--finish-fd", required=True, type=int)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--profile", type=Path, default=ROOT / "examples/joystick-relic-arm.json")
    p.add_argument("--duration", type=float, default=60.0)
    run(p.parse_args())
