"""Offline parity with the previously audited ReLIC simulator contract and actual weights."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, State
from spot_deploy.records import RunRecord
from spot_deploy.relic_policy import DEFAULT_Q, OBS_IDS, ROOT_COM_B, ReLICPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    ref = args.reference.resolve()
    sys.path.insert(0, str(ref))
    from spot_relic_sim.contract import Command, Contract, Policy

    model = ref / "source/relic/relic/assets/spot/pretrained/policy.onnx"
    record = RunRecord(args.output, "relic-shadow-parity", [Path(__file__), model,
        ref / "spot_relic_sim/contract.py", ref / "configs/articulation.json",
        ref / "source/relic/relic/assets/spot/constants.py"])
    reference = Contract.load(ref / "configs/articulation.json")
    native_names = [n.replace("arm_", "arm0_") for n in reference.names]
    sdk_ids = [native_names.index(n) for n in JOINTS]
    policy, ref_policy = ReLICPolicy(model), Policy()
    rng = np.random.default_rng(20261005)
    previous = np.zeros(12, dtype=np.float32)
    maxima = dict(observation=0., action=0., target=0.)
    command = Command()
    for index in range(32):
        # Include variable requested heights; this input is an absolute metre
        # value, independent of action offsets and the arbitrary odom origin.
        policy.body_height = float(.30 + .01 * index)
        command.torso[2] = policy.body_height
        axis = np.eye(3)[index % 3]
        angle = (index % 7 - 3) * .17
        skew = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rot = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)
        linear, angular = rng.normal(0, .2, (2, 3))
        q = (DEFAULT_Q + rng.normal(0, .15, 19)).astype(np.float32)
        dq = rng.normal(0, .2, 19).astype(np.float32)
        state = State(robot_time_s=100 + index * .02, received_monotonic_s=100 + index * .02,
            positions=q.tolist(), velocities=dq.tolist(), loads=[0.] * 19,
            odom_quaternion_wxyz=[math.cos(angle / 2), *(axis * math.sin(angle / 2)).tolist()],
            linear_velocity_odom=(rot @ (linear - np.cross(angular, ROOT_COM_B))).tolist(),
            angular_velocity_odom=(rot @ angular).tolist(), last_command_key=0,
            last_command_received_robot_s=0.)
        if index == 0:
            command.arm = q[12:].copy()
        expected_obs = reference.observation({"q": q[OBS_IDS], "dq": dq[OBS_IDS],
            "linear_velocity": linear, "angular_velocity": angular,
            "gravity": rot.T @ [0, 0, -1]}, command, previous)
        expected_action = ref_policy(expected_obs)
        expected_target = reference.targets(expected_action, command)[sdk_ids]
        target, _, obs, action = policy.predict(state)
        for name, got, expected in (("observation", obs, expected_obs),
                                   ("action", action, expected_action),
                                   ("target", target, expected_target)):
            maxima[name] = max(maxima[name], float(np.max(np.abs(got - expected))))
            np.testing.assert_allclose(got, expected, atol=2e-5, rtol=0)
        record.event("comparison", case=index, state=state.model_dump(), observations=obs.tolist(),
                     raw_actions=action.tolist(), targets=target.tolist())
        previous = expected_action
    result = {"passed": True, "cases": 32, "seed": 20261005, "max_abs_error": maxima,
              "atol": 2e-5, "rtol": 0, "hardware_access": False}
    record.finish("passed", result)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
