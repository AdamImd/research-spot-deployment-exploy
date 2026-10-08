"""Compare Exploy with the original ReLIC adapter on varied states and recorded policy frames."""

import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import State, check_artifacts
from spot_deploy.exploy_policy import ExployReLICManifest
from spot_deploy.records import RunRecord
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_policy import DEFAULT_Q


def run(args, record):
    manifest, reference = load_manifest(args.manifest), load_manifest(args.reference_manifest)
    if not isinstance(manifest, ExployReLICManifest) or reference.schema_version != 2:
        raise ValueError("Expected v3 Exploy and v2 original manifests")
    policy = make_policy(check_artifacts(args.manifest, manifest), manifest, args.manifest)
    original = make_policy(check_artifacts(args.reference_manifest, reference), reference,
                           args.reference_manifest)
    height = json.loads(args.height_record.read_text())
    policy.initialize_height(height)
    original.initialize_height(height)
    errors = np.zeros(3)
    count = 0

    def compare(state, previous, label):
        nonlocal errors, count
        target, elapsed, obs, action = policy.evaluate(state, previous)
        rt, _, ro, ra = original.evaluate(state, previous)
        diff = np.array([np.max(np.abs(obs - ro)), np.max(np.abs(action - ra)),
                         np.max(np.abs(target - rt))])
        errors = np.maximum(errors, diff)
        record.event("comparison", case=label, max_abs_errors=diff.tolist(), inference_s=elapsed)
        count += 1
        if np.any(diff > [1e-5, 1e-4, 3e-5]):
            raise AssertionError(f"Exploy numerical parity failed on {label}: {diff}")
        return target, obs, action

    rng = np.random.default_rng(101)
    for i in range(128):
        quat = rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        state = State(robot_time_s=1000 + i * .02, received_monotonic_s=10 + i * .02,
            positions=(DEFAULT_Q + rng.normal(0, .2, 19)).tolist(),
            velocities=rng.normal(0, .5, 19).tolist(), loads=[0.] * 19,
            odom_quaternion_wxyz=quat.tolist(), linear_velocity_odom=rng.normal(0, .2, 3).tolist(),
            angular_velocity_odom=rng.normal(0, .3, 3).tolist(), last_command_key=i,
            last_command_received_robot_s=1000 + i * .02)
        previous = np.zeros(12, dtype=np.float32) if i == 0 else rng.normal(0, .5, 12).astype(np.float32)
        # Exercise every command boundary independently of the imported standing pose.
        arm = (DEFAULT_Q[12:] + rng.normal(0, .1, 7)).tolist()
        policy.manifest.arm_stowed_positions = arm
        original.manifest.arm_stowed_positions = arm
        policy.body_height = original.body_height = float(rng.uniform(.35, .65))
        first = compare(state, previous, f"synthetic-{i}")
        policy.evaluate(state, -previous)
        again = policy.evaluate(state, previous)
        for before, after in zip(first, (again[0], again[2], again[3])):
            np.testing.assert_array_equal(before, after)
    # Restore candidate commands before replaying the independent recorded trace.
    arm = load_manifest(args.manifest).arm_stowed_positions
    policy.manifest.arm_stowed_positions = original.manifest.arm_stowed_positions = arm
    policy.initialize_height(height)
    original.initialize_height(height)
    recorded = 0
    for path in args.events:
        with path.open() as stream:
            for line in stream:
                row = json.loads(line)
                if row.get("event") != "policy":
                    continue
                compare(State.model_validate(row["state"]), np.asarray(row["observations"][-12:],
                        dtype=np.float32), f"recorded-{recorded}")
                recorded += 1
    if args.events and not recorded:
        raise ValueError("No policy frames found in requested trace")
    policy.reset()
    policy.warmup()
    np.testing.assert_array_equal(policy.previous_action, np.zeros(12))
    if policy.body_height != manifest.relic.body_height_m:
        raise AssertionError("Warmup changed height initialization")
    return dict(samples=count, synthetic_samples=128, recorded_samples=recorded, seed=101,
        max_observation_error=float(errors[0]), max_raw_action_error=float(errors[1]),
        max_joint_target_error_rad=float(errors[2]), tolerances=[1e-5, 1e-4, 3e-5],
        stateless_history=True, zero_history_after_warmup=True, hardware_access=False,
        hardware_qualified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--reference-manifest", required=True, type=Path)
    parser.add_argument("--height-record", required=True, type=Path)
    parser.add_argument("--events", nargs="*", type=Path, default=[])
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    record = RunRecord(args.output, "exploy-parity", [args.manifest, args.reference_manifest,
                                                    args.height_record, *args.events])
    try:
        result = run(args, record)
        record.finish("passed", result)
        print(json.dumps(result), flush=True)
    except BaseException as exc:
        record.finish("failed", {"error_type": type(exc).__name__, "error": str(exc)})
        raise


if __name__ == "__main__":
    main()
