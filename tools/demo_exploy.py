"""Predict bundled standing-pose targets offline; never construct a robot client."""

import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, State, check_artifacts
from spot_deploy.records import verify_run
from spot_deploy.relic_contract import load_manifest, make_policy

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path,
                        default=ROOT / "policies/relic-exploy-standing/manifest.json")
    parser.add_argument("--capture", type=Path, default=ROOT / "fixtures/standing-capture")
    args = parser.parse_args()
    if not verify_run(args.capture):
        raise ValueError("Offline pose fixture failed integrity verification")
    manifest = load_manifest(args.manifest)
    policy = make_policy(check_artifacts(args.manifest, manifest), manifest, args.manifest)
    policy.warmup()
    np.testing.assert_array_equal(policy.previous_action, np.zeros(12))
    policy.initialize_height(json.loads((args.capture / "initial-height.json").read_text()))
    state = State.model_validate_json((args.capture / "initial-state.json").read_text())
    targets, elapsed, observations, actions = policy.evaluate(state, np.zeros(12))
    print(json.dumps({
        "mode": "offline prediction only", "hardware_access": False,
        "manifest": str(args.manifest), "previous_actions": [0.] * 12,
        "body_height_m": policy.body_height, "observation_shape": list(observations.shape),
        "inference_s": elapsed, "raw_actions": actions.tolist(),
        "joint_targets_rad": dict(zip(JOINTS, targets.tolist(), strict=True)),
    }, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
