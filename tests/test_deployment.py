import builtins
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest
from pydantic import ValidationError

from spot_deploy import deployment
from spot_deploy.cli import main
from spot_deploy.contracts import Envelope, canonical_hash, sha256
from spot_deploy.readiness import GATES, HARDWARE_GATE, binding
from spot_deploy.records import RunRecord
from test_readiness_cli import robot_config

ROOT = Path(__file__).resolve().parents[1]
CANDIDATE = ROOT / "policies/relic-exploy-standing/manifest.json"


@pytest.fixture
def clean_source(monkeypatch):
    identity = deployment.source_identity() | {"dirty": False}
    monkeypatch.setattr(deployment, "source_identity", lambda: identity)
    return identity


@pytest.fixture
def bundle(tmp_path, clean_source):
    robot = robot_config(tmp_path)
    config = json.loads(robot.read_text())
    config["hardware_estop"] = dict(profile_sha256="1" * 64,
        status_file=str(tmp_path / "stop.json"), max_status_age_s=.25)
    robot.write_text(json.dumps(config))
    target = tmp_path / "deployment"
    assert main(["prepare-deployment", "--manifest", str(CANDIDATE),
                 "--robot", str(robot), "--output", str(target)]) == 0
    return target


def statuses(result):
    return {r["check"]: r["status"] for r in result["checks"]}


def test_prepare_and_check_never_import_sdk(monkeypatch, tmp_path, clean_source):
    original = builtins.__import__

    def blocked(name, *args, **kwargs):
        if name.startswith("bosdyn") or name.endswith(("sdk_read", "sdk_control", "sdk_estop")):
            raise AssertionError("preparation must not import a hardware adapter")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked)
    target = tmp_path / "bundle"
    assert main(["prepare-deployment", "--manifest", str(CANDIDATE),
                 "--output", str(target)]) == 0
    assert sha256(target / "policy/policy.onnx") == sha256(CANDIDATE.parent / "policy.onnx")
    assert (target / "policy/RELIC-LICENSE").is_file()
    command = json.loads((target / "commands.json").read_text())
    assert "--execute" not in command["stand_requires_explicit_execution"]
    assert "--envelope" not in command["preflight"]
    output = tmp_path / "check"
    assert main(["deployment-check", "--bundle", str(target), "--output", str(output)]) == 1
    result = json.loads((output / "result.json").read_text())
    assert statuses(result)["policy_bundle"] == "passed"
    assert not result["live_ready"] and result["motion_commands"] == 0
    assert (output / "deployment-check.md").is_file()


def test_review_requires_explicit_limits_and_no_evidence_promotion(bundle):
    template = json.loads((bundle / "envelope.review.json").read_text())
    assert template["transition_s"] == 0 and template["scope"] == "hardware"
    assert template["load_max"] == [None] * 19
    assert template["max_command_ack_s"] is None
    with pytest.raises(ValidationError):
        Envelope.model_validate(template)
    assert json.loads((bundle / "evidence.json").read_text())["evidence"] == []
    result = deployment.check(bundle)
    assert not result["passed"]
    assert statuses(result)[HARDWARE_GATE] == "unverified"


@pytest.mark.parametrize("change", ["graph", "missing_digest", "escape", "source", "packages"])
def test_prepared_bundle_rejects_drift(bundle, monkeypatch, clean_source, change):
    if change == "graph":
        with (bundle / "policy/policy.onnx").open("ab") as stream:
            stream.write(b"changed")
    elif change in ("missing_digest", "escape"):
        path = bundle / "bundle.json"
        data = json.loads(path.read_text())
        if change == "missing_digest":
            del data["frozen_files"]["policy/policy.onnx"]
        else:
            data["frozen_files"]["../outside"] = "0" * 64
        path.write_text(json.dumps(data))
    elif change == "source":
        clean_source["source_sha256"] = "0" * 64
    else:
        monkeypatch.setattr(deployment, "runtime_packages", lambda: {"onnxruntime": "changed"})
    check = statuses(deployment.check(bundle))
    assert check["runtime" if change in ("source", "packages") else "policy_bundle"] == "blocked"


def test_simulation_envelope_and_dirty_source_cannot_prepare(tmp_path, clean_source):
    assert main(["prepare-deployment", "--manifest", str(CANDIDATE), "--envelope",
                 str(ROOT / "fixtures/relic-direct-simulation/envelope.json"),
                 "--output", str(tmp_path / "sim")]) == 1
    clean_source["dirty"] = True
    assert main(["prepare-deployment", "--manifest", str(CANDIDATE),
                 "--output", str(tmp_path / "dirty")]) == 1


def complete_test_inputs(bundle, tmp_path):
    # Synthetic test limits and review records; never physical qualification.
    limits = json.loads((ROOT / "fixtures/relic-direct-simulation/envelope.json").read_text())
    limits["scope"] = "hardware"
    (bundle / "envelope.json").write_text(json.dumps(limits))
    bound = canonical_hash(binding(bundle / "policy/manifest.json", bundle / "envelope.json",
                                   bundle / "robot.json"))
    artifact = bundle / "test-review.txt"
    artifact.write_text("Synthetic unit-test evidence only")
    now = datetime.now(timezone.utc)
    records = [dict(gate=g, status="passed", reviewed_by="test reviewer",
        recorded_at=(now - timedelta(minutes=1)).isoformat(),
        expires_at=(now + timedelta(minutes=10)).isoformat(), binding_sha256=bound,
        artifact=artifact.name, artifact_sha256=sha256(artifact), note="Test only")
        for g in [*GATES, HARDWARE_GATE]]
    (bundle / "evidence.json").write_text(json.dumps(dict(schema_version=1, evidence=records)))
    preflight = tmp_path / "preflight"
    run = RunRecord(preflight, "preflight", [bundle / "robot.json", bundle / "policy/manifest.json"])
    run.finish("passed", {"passed": True, "checks": {"identity_firmware": True}})
    return preflight


def test_complete_offline_check_still_does_not_activate(bundle, tmp_path, monkeypatch):
    preflight = complete_test_inputs(bundle, tmp_path)
    monkeypatch.setattr("spot_deploy.estop_interlock.HardwareInterlock.check", lambda self: None)
    result = deployment.check(bundle, preflight, check_estop=True)
    assert result["passed"]
    assert not result["live_ready"] and result["motion_commands"] == 0
    assert result["binding_sha256"]
    assert not deployment.check(bundle, preflight)["passed"]  # Explicit local stop check needed.


@pytest.mark.parametrize("change", ["stale", "other_config", "tamper", "simulation"])
def test_preflight_must_be_fresh_bound_and_from_preflight(bundle, tmp_path, change):
    preflight = complete_test_inputs(bundle, tmp_path)
    if change == "stale":
        path = preflight / "COMPLETE.json"
        data = json.loads(path.read_text())
        data["finished_at"] = (datetime.now(timezone.utc) - timedelta(minutes=6)).isoformat()
        path.write_text(json.dumps(data))
    elif change == "other_config":
        path = bundle / "robot.json"
        data = json.loads(path.read_text())
        data["expected_serial"] = "different-test-robot"
        path.write_text(json.dumps(data))
    elif change == "tamper":
        (preflight / "result.json").write_text('{}')
    else:
        path = preflight / "run.json"
        data = json.loads(path.read_text())
        data["mode"] = "replay"
        path.write_text(json.dumps(data))
        complete = json.loads((preflight / "COMPLETE.json").read_text())
        complete["artifacts"]["run.json"] = sha256(path)
        (preflight / "COMPLETE.json").write_text(json.dumps(complete))
    assert statuses(deployment.check(bundle, preflight))["fresh_preflight"] == "blocked"
