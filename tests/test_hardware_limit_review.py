import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.contracts import Envelope, JOINTS

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("limit_review", ROOT / "tools/review_hardware_limits.py")
review = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(review)


@pytest.fixture
def trace(tmp_path):
    # Reverse native articulation order deliberately: measured q must be remapped,
    # whereas targets, torques, and supported-hold events are already SDK ordered.
    q = np.arange(19) / 100
    target = q + .2
    config = {"hardware_access": False, "preparation_s": 0, "backend": "synthetic",
              "seed": 101, "policy_hz": 50., "command_hz": 200.,
              "envelope": {"joint_order": list(JOINTS)},
              "simulation_joint_names": [n.replace("arm0_", "arm_") for n in JOINTS[::-1]],
              "manifest": {"gains": {"kp": [10.] * 19, "kd": [1.] * 19},
                           "policy_sha256": "0" * 64, "arm_stowed_positions": q[12:].tolist()}}
    rows = [{"trial_time": t, "q": q[::-1].tolist(), "dq": [0.] * 19,
             "positions": target.tolist(), "pd_torque_Nm": [2.] * 19,
             "preceding_applied_torque_Nm": [0.] * 19, "feedforward": [0.] * 19,
             "height": .5, "tilt": 0., "linear_velocity": [0.] * 3,
             "angular_velocity": [0.] * 3} for t in (0., .005)]
    events = [{"event": "preparation", "monotonic_s": -.005,
               "reference": q.tolist(), "feedforward": [0.] * 19},
              {"event": "relic_handover", "monotonic_s": 0.,
               "previous_actions": [0.] * 12, "torque_step_Nm": [2.] * 19}]
    for name, value in (("configuration.json", config), ("source.json", {"packages": {}}),
                        ("result.json", {"stable_final_window": True}), ("run.json", {})):
        (tmp_path / name).write_text(json.dumps(value))
    for name, value in (("events.jsonl", events), ("rollout.jsonl", rows), ("timing.jsonl", [])):
        (tmp_path / name).write_text("".join(json.dumps(r) + "\n" for r in value))
    artifacts = {p.name: review.digest(p) for p in tmp_path.iterdir()}
    (tmp_path / "COMPLETE.json").write_text(json.dumps(
        {"status": "completed", "artifacts": artifacts}))
    return tmp_path


def test_native_order_and_first_handover_are_included(trace):
    result = review.summarize(trace, .005)
    joint = result["first_trial"]["joints"]["arm0_f1x"]
    assert joint["position_min_rad"] == pytest.approx(.18)
    assert joint["tracking_error_max_rad"] == pytest.approx(.2)
    assert joint["target_rate_max_rad_s"] == pytest.approx(40.)
    # The only target jump was BEFORE row zero, so diff(rollout_targets) misses it.
    assert result["handover_target_rate_rad_s"][18] == pytest.approx(40.)


def test_altered_raw_artifact_is_rejected(trace):
    (trace / "rollout.jsonl").write_text("{}\n")
    with pytest.raises(ValueError, match="hash mismatch: rollout.jsonl"):
        review.summarize(trace, .005)


def test_review_proposal_cannot_be_used_as_hardware_envelope():
    proposal = json.loads((ROOT / "records/hardware-limit-review-20261009/proposal.json").read_text())
    assert proposal["approval_status"] == "unreviewed"
    with pytest.raises(ValidationError):
        Envelope.model_validate(proposal)
