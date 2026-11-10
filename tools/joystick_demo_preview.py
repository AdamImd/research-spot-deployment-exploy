"""Capture a base-command trace only. No SDK, lease, or robot connection."""

import argparse
import json
from pathlib import Path
import time
from types import SimpleNamespace
from spot_deploy.contracts import load
from spot_deploy.joystick_teleop import DemoJoystick, JoystickTeleopPlan, TeleopMotion
from spot_deploy.records import atomic_json
from spot_deploy.relic_contract import load_manifest

ROOT = Path(__file__).resolve().parents[1]


def run(plan, stow, finish_fd, output):
    output.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    count = 0
    info = dict(
        kind="joystick command demonstration",
        hardware_access=False,
        contains_robot_states=False,
        arm="stowed",
        profile=plan.model_dump(mode="json"),
    )
    print("Recording: " + str(output), flush=True)
    source = None
    try:
        source = DemoJoystick(plan, finish_fd)
        motion = TeleopMotion(SimpleNamespace(teleop=plan, arm_stowed_positions=stow), source)
        start = time.monotonic()
        deadline = next_save = start
        with (output / "commands.jsonl").open("x") as stream:
            while True:
                now = time.monotonic()
                cmd, _ = motion.update(now, start)
                if now >= next_save or motion.completed:
                    row = dict(
                        time_s=now - start, velocity_command=cmd, enabled=motion.status["enabled"], monitor=source.monitor.copy()
                    )
                    stream.write(json.dumps(row) + "\n")
                    count += 1
                    next_save += 0.02
                    if next_save <= now:
                        next_save = now + 0.02
                    if count % 5 == 1:
                        print("Monitor: " + json.dumps(source.monitor), flush=True)
                        print("Velocity: " + json.dumps(cmd), flush=True)
                if motion.completed or now - start >= plan.duration_s:
                    break
                deadline = max(deadline + 0.005, time.monotonic())
                time.sleep(max(0, deadline - time.monotonic()))
        info["status"] = "completed"
    except BaseException as exc:
        info.update(status="failed", error=str(exc))
        raise
    finally:
        if source:
            source.close()
        atomic_json(
            output / "summary.json", dict(**info, samples=count, elapsed_s=time.monotonic() - start)
        )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--profile", type=Path, required=True)
    p.add_argument("--bundle", type=Path)
    p.add_argument("--finish-fd", type=int, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    m = load_manifest(ROOT / "policies/relic-exploy-standing/manifest.json")
    live = load_manifest(a.bundle / "policy/manifest.json") if a.bundle else None
    run(
        (live.teleop if live else load(a.profile, JoystickTeleopPlan)).model_copy(
            update={"study_family": None, "study_commands": None}
        ),
        (live or m).arm_stowed_positions,
        a.finish_fd,
        a.output,
    )
