"""Replay saved simulator states/targets through host guards; no dynamics or SDK I/O."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, Envelope, State, ContractError, load
from spot_deploy.policy import rotation
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest
from spot_deploy.relic_policy import ROOT_COM_B
from spot_deploy.safety import Guard


def run(source, manifest, envelope, output):
    if not verify_run(source, status="completed"):
        raise ValueError("source recording failed integrity verification")
    env = load(envelope, Envelope)
    if env.scope != "simulation":
        raise ValueError("saved-data audit requires simulation scope")
    policy = load_manifest(manifest)
    config = json.loads((source / "configuration.json").read_text())
    names = [n.replace("arm_", "arm0_") for n in config["simulation_joint_names"]]
    ids = [names.index(n) for n in JOINTS]
    output.mkdir(parents=True, exist_ok=False)
    events = []
    with (output / "events.jsonl").open("x") as stream:

        def event(name, **data):
            value = dict(event=name, **data)
            events.append(value)
            stream.write(json.dumps(value, allow_nan=False) + "\n")

        guard = Guard(policy, env, event=event)
        count, stopped = 0, None
        with (source / "rollout.jsonl").open() as rows:
            for line in rows:
                row = json.loads(line)
                t = row["trial_time"]
                state = State(
                    robot_time_s=1000 + t,
                    received_monotonic_s=10 + t,
                    positions=np.asarray(row["q"])[ids].tolist(),
                    velocities=np.asarray(row["dq"])[ids].tolist(),
                    loads=row["preceding_applied_torque_Nm"],
                    odom_quaternion_wxyz=row["root_quaternion_wxyz"],
                    linear_velocity_odom=[0.0] * 3,
                    angular_velocity_odom=[0.0] * 3,
                    body_position_odom=row["position"],
                    body_pose_robot_time_s=1000 + t,
                    last_command_key=guard.key,
                    last_command_received_robot_s=1000 + t,
                )
                r = rotation(state)
                state.linear_velocity_odom = (
                    r
                    @ (
                        np.asarray(row["linear_velocity"])
                        - np.cross(row["angular_velocity"], ROOT_COM_B)
                    )
                ).tolist()
                state.angular_velocity_odom = (r @ np.asarray(row["angular_velocity"])).tolist()
                try:
                    guard.check_state(state, 10 + t, 1000 + t)
                    guard.command(
                        row["positions"], state, 10 + t, 1000 + t, 10 + t, 0.0, row["feedforward"]
                    )
                except ContractError as exc:
                    stopped = dict(time_s=t, reason=str(exc))
                    break
                count += 1
    recovered = [e for e in events if e["event"] == "physical_limit_recovered"]
    result = dict(
        passed=stopped is None,
        stopped=stopped,
        samples_checked=count,
        through_time_s=t,
        hardware_access=False,
        new_dynamics=False,
        hardware_qualified=False,
        physical_limit_persistence_s=env.physical_limit_persistence_s,
        timing="ideal replay clock and zero inference duration; no live scheduling qualification",
        crossings_started=sum(e["event"] == "physical_limit_started" for e in events),
        crossings_recovered=len(recovered),
        max_recovered_duration_s=max((e["elapsed_s"] for e in recovered), default=0.0),
        pending_at_end=[
            dict(limit=k[0], index=k[1], since_s=v)
            for k, v in guard.physical_limits.pending.items()
        ],
        source_complete_sha256=hashlib.sha256((source / "COMPLETE.json").read_bytes()).hexdigest(),
        envelope=env.model_dump(),
    )
    atomic_json(output / "result.json", result)
    atomic_json(
        output / "COMPLETE.json",
        dict(
            status="completed",
            artifacts={
                p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.glob("*.json*")
            },
        ),
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--envelope", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.run, args.manifest, args.envelope, args.output)
    print(json.dumps({k: v for k, v in result.items() if k != "envelope"}))
