"""Offline numerical audit of ReLIC observations/actions and independent gravity support."""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

from spot_deploy.contracts import JOINTS, State, check_artifacts, sha256
from spot_deploy.policy import rotation
from spot_deploy.records import atomic_json, source_identity, verify_run
from spot_deploy.relic_contract import ReLICDeploymentPolicy, load_manifest
from spot_deploy.relic_policy import ROOT_COM_B


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("simulation-source", "capture", "manifest", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    if not verify_run(args.capture):
        raise ValueError("invalid capture")
    sys.path.insert(0, str(args.simulation_source))
    from spot_relic_sim.contract import Command, Contract, Policy
    from spot_relic_sim.pose_start import StaticSupport
    from spot_relic_sim.record import provenance

    manifest = load_manifest(args.manifest)
    policy = ReLICDeploymentPolicy(check_artifacts(args.manifest, manifest), manifest, args.manifest)
    contract = Contract.load(args.simulation_source / "configs/articulation.json")
    reference_policy = Policy()
    reference_support = StaticSupport(contract, args.output / "reference-model")
    ids = [JOINTS.index(n.replace("arm_", "arm0_")) for n in contract.names]
    initial = State.model_validate_json((args.capture / "initial-state.json").read_text())
    rng = np.random.default_rng(101)
    rows, timing = [], []
    for case in range(32):
        state = initial.model_copy(deep=True)
        if case:
            state.positions = (np.asarray(initial.positions) + rng.normal(0, .015, 19)).tolist()
            state.velocities = rng.normal(0, .1, 19).tolist()
            state.linear_velocity_odom = rng.normal(0, .04, 3).tolist()
            state.angular_velocity_odom = rng.normal(0, .07, 3).tolist()
        previous = np.zeros(12, dtype=np.float32) if case == 0 else rng.normal(0, 1, 12).astype(np.float32)
        height = .52 if case == 0 else float(rng.uniform(.45, .6))
        policy.initialize_height({"height_m": height, "foot_contacts": [1]*4})
        target, elapsed, obs, action = policy.evaluate(state, previous)
        r = rotation(state).T
        w = r @ state.angular_velocity_odom
        sim_state = dict(q=np.asarray(state.positions)[ids], dq=np.asarray(state.velocities)[ids],
                         linear_velocity=r @ state.linear_velocity_odom + np.cross(w, ROOT_COM_B),
                         angular_velocity=w, gravity=r @ [0, 0, -1])
        command = Command(arm=np.asarray(manifest.arm_stowed_positions),
                          torso=np.array([0, 0, height]))
        expected_obs = contract.observation(sim_state, command, previous)
        expected_action = reference_policy(expected_obs)
        expected_target = contract.targets(expected_action, command)
        start = time.monotonic()
        support, _ = policy.support.compute(state)
        timing.append(time.monotonic()-start)
        expected_support, _ = reference_support.compute(sim_state["q"], state.odom_quaternion_wxyz)
        row = dict(case=case, state=state.model_dump(), previous=previous.tolist(), height_m=height,
                   observation_max_error=float(np.max(np.abs(obs[0]-expected_obs))),
                   action_max_error=float(np.max(np.abs(action-expected_action))),
                   target_max_error_rad=float(np.max(np.abs(target[ids]-expected_target))),
                   support_max_error_Nm=float(np.max(np.abs(support[ids]-expected_support))),
                   inference_s=elapsed)
        # Independent solver converts q to float32; micro-Nm differences are expected.
        assert row["support_max_error_Nm"] < 1e-4
        assert row["observation_max_error"] < 1e-5
        assert row["action_max_error"] < 1e-4
        assert row["target_max_error_rad"] < 3e-5
        rows.append(row)
    result = dict(passed=True, cases=len(rows), seed=101, hardware_access=False,
                  max_errors={key: max(r[key] for r in rows) for key in
                              ("observation_max_error", "action_max_error", "target_max_error_rad",
                               "support_max_error_Nm")},
                  support_compute_p99_s=float(np.quantile(timing,.99)),
                  source=source_identity(), manifest_sha256=sha256(args.manifest),
                  audit_script_sha256=sha256(Path(__file__)),
                  simulation_source=provenance("mujoco", contract),
                  capture_sha256=sha256(args.capture / "COMPLETE.json"), cases_detail=rows)
    atomic_json(args.output / "result.json", result)
    print(json.dumps({key: result[key] for key in
                      ("passed", "cases", "seed", "max_errors", "support_compute_p99_s")}))


if __name__ == "__main__":
    main()
