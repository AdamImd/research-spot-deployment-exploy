"""Freeze a four-family study from one USB-only joystick demonstration. No robot I/O."""

import argparse
import hashlib
from spot_deploy.contracts import sha256
import json
import math
from pathlib import Path
import numpy as np
from spot_deploy.joystick_teleop import JoystickTeleopPlan

FAMILIES = ("forward", "six", "random", "joystick")
MAX = np.array([0.30, 0.20, 0.50])
ACC = np.array([0.10, 0.10, 0.40])


def emit(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sequences(joystick):
    rng = np.random.default_rng(101)
    sticks = []
    with joystick.open() as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("event") == "policy":
                sticks.append(
                    {"time_s": row["monotonic_s"], "velocity_command": row["velocity_command"]}
                )
            elif "event" not in row and "time_s" in row:
                sticks.append(row)
    if len(sticks) < 2:
        raise ValueError("record at least two command samples")
    t = np.array([s["time_s"] for s in sticks], dtype=float)
    t -= t[0]
    v = np.array([s["velocity_command"] for s in sticks])
    if (
        len(t) < 2
        or not np.isfinite(t).all()
        or not np.isfinite(v).all()
        or v.shape != (len(t), 3)
        or np.any(np.diff(t) <= 0)
        or np.max(np.diff(t)) > 0.1
        or np.any(abs(v) > MAX + 1e-6)
        or t[-1] > 300
        or not np.any(abs(v) > 0.01)
    ):
        raise ValueError(
            "need a valid, nonstationary USB recording within the reviewed joystick range"
        )
    tick = np.arange(0, t[-1] + 1e-9, 0.02)
    joy = v[np.searchsorted(t, tick, side="right") - 1]
    if np.any(joy[0]) or np.any(joy[-1]):
        raise ValueError("start and end the demonstration at zero")

    # Freeze a tick-aligned, slew-bounded version of the timestamped input.
    # A released enable produces an immediate zero, as for live teleoperation.
    previous = np.zeros(3)
    for i, demand in enumerate(joy):
        if i < 150 or not np.any(demand):
            previous[:] = 0
        else:
            previous += np.clip(demand - previous, -0.02 * ACC, 0.02 * ACC)
        joy[i] = previous
    if not np.any(abs(joy) > 0.01):
        raise ValueError("no motion remains after the initial three-second hold")

    def ramps(goals):
        v = np.zeros(3)
        rows = []
        for dur, goal in goals:
            for _ in range(round(dur * 50)):
                v += np.clip(goal - v, -ACC * 0.02, ACC * 0.02)
                rows.append(v.copy())
        return np.array(rows)

    z = np.zeros(3)
    directions = np.eye(3) * [0.18, 0.12, 0.25]
    six = [(4.0, sign * d) for d in directions for sign in [1, -1]]
    rand = [(3.0, rng.uniform(-1, 1, 3) * [0.18, 0.12, 0.25]) for _ in range(6)]
    return dict(
        forward=ramps([(3, z), (5, directions[0]), (3, z)]),
        six=ramps([(3, z), *six, (3, z)]),
        random=ramps([(3, z), *rand, (3, z)]),
        joystick=joy,
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--joystick", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    a = p.parse_args()
    traces = sequences(a.joystick)
    profile = json.loads(
        (
            Path(__file__).resolve().parents[1] / "examples/joystick-relic-arm-five-minute.json"
        ).read_text()
    )
    a.output.mkdir(parents=True, exist_ok=False)
    study = dict(
        schema=1,
        seed=101,
        policy_hz=50,
        hardware_authorized=False,
        training=list(FAMILIES[:3]),
        holdout="joystick",
        repeats=5,
        source_joystick_sha256=sha256(a.joystick),
        families={},
    )
    for family, vs in traces.items():
        path = a.output / (family + ".json")
        digest = emit(path, dict(hz=50, velocity_command=vs.tolist()))
        bound = JoystickTeleopPlan.model_validate(
            dict(
                profile,
                study_family=family,
                study_commands=vs.tolist(),
                duration_s=min(300, max(6, math.ceil(len(vs) / 50) + 1)),
            )
        )
        profile_path = a.output / (family + "-profile.json")
        emit(profile_path, bound.model_dump(mode="json"))
        study["families"][family] = dict(trace=path.name, sha256=digest, profile=profile_path.name)
    emit(a.output / "study.json", study)
    emit(a.output / "recordings.json", {k: [None] * 5 for k in FAMILIES})
    print(a.output / "study.json")


if __name__ == "__main__":
    main()
