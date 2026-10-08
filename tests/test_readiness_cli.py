import builtins
import json
from datetime import datetime, timedelta, timezone

import pytest

from spot_deploy.cli import main, safe_error
from spot_deploy.contracts import ContractError, canonical_hash, sha256
from spot_deploy.readiness import GATES, binding, report, require_live
from spot_deploy.records import verify_run


def robot_config(tmp_path):
    path = tmp_path / "robot.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "endpoint": "example.invalid",
                "expected_serial": "fixture",
                "expected_firmware": "5.1.1",
                "expected_host": "rpm",
                "wired_interface": "eth0",
                "rpc_timeout_s": 1,
                "joint_control_feature": "joint_control",
            }
        )
    )
    return path


def evidence_index(tmp_path, fixture_path):
    robot = robot_config(tmp_path)
    bound = canonical_hash(
        binding(fixture_path / "manifest.json", fixture_path / "envelope.json", robot)
    )
    artifact = tmp_path / "human-review.txt"
    artifact.write_text("Test-only review evidence")
    now = datetime.now(timezone.utc)
    data = {
        "schema_version": 1,
        "evidence": [
            {
                "gate": gate,
                "status": "passed",
                "reviewed_by": "test reviewer",
                "recorded_at": (now - timedelta(minutes=1)).isoformat(),
                "expires_at": (now + timedelta(hours=1)).isoformat(),
                "binding_sha256": bound,
                "artifact": artifact.name,
                "artifact_sha256": sha256(artifact),
                "note": "test fixture",
            }
            for gate in GATES
        ],
    }
    path = tmp_path / "evidence.json"
    path.write_text(json.dumps(data))
    return path, robot, data


def test_missing_evidence_is_unverified():
    result = report()
    assert not result["evidence_ready"]
    assert {c["status"] for c in result["checks"]} == {"unverified"}


@pytest.mark.parametrize(
    "change", ["binding", "expired", "future", "hash", "failed", "missing", "escape"]
)
def test_bad_evidence_never_promotes(tmp_path, fixture_path, change):
    path, robot, data = evidence_index(tmp_path, fixture_path)
    record = data["evidence"][0]
    if change == "binding":
        record["binding_sha256"] = "0" * 64
    elif change == "expired":
        record["expires_at"] = "2000-01-01T00:00:00+00:00"
    elif change == "future":
        record["recorded_at"] = "2200-01-01T00:00:00+00:00"
    elif change == "hash":
        record["artifact_sha256"] = "0" * 64
    elif change == "failed":
        record["status"] = "failed"
    elif change == "missing":
        data["evidence"].pop(0)
    else:
        record["artifact"] = "../escape"
    path.write_text(json.dumps(data))
    result = report(fixture_path / "manifest.json", fixture_path / "envelope.json", robot, path)
    assert not result["evidence_ready"]


def test_binding_valid_but_fixture_still_blocked(tmp_path, fixture_path, manifest, envelope):
    path, robot, data = evidence_index(tmp_path, fixture_path)
    assert report(fixture_path / "manifest.json", fixture_path / "envelope.json", robot, path)[
        "evidence_ready"
    ]
    with pytest.raises(ContractError, match="fixture"):
        require_live(
            manifest,
            envelope,
            None,
            fixture_path / "manifest.json",
            fixture_path / "envelope.json",
            robot,
            path,
            "Adam",
            "Operator",
            True,
            0.5,
        )


@pytest.mark.parametrize(
    "execute,operator,safety,error",
    [
        (False, "Adam", "Xun", "explicit"),
        (True, "Adam", "adam", "distinct"),
        (True, "", "Xun", "explicit"),
    ],
)
def test_live_operator_gates(manifest, envelope, execute, operator, safety, error):
    with pytest.raises(ContractError, match=error):
        require_live(
            manifest, envelope, None, None, None, None, None, operator, safety, execute, 0.5
        )


def test_offline_commands_cannot_import_sdk(monkeypatch, tmp_path, fixture_path):
    original = builtins.__import__

    def blocked(name, *args, **kwargs):
        if name.startswith("bosdyn") or name.endswith("sdk_read") or name.endswith("sdk_control"):
            raise AssertionError("offline command accessed hardware adapter")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked)
    for mode in ("inspect-policy", "replay", "readiness"):
        output = tmp_path / mode
        argv = [mode, "--manifest", str(fixture_path / "manifest.json"), "--output", str(output)]
        if mode == "replay":
            argv += [
                "--states",
                str(fixture_path / "states.jsonl"),
                "--envelope",
                str(fixture_path / "envelope.json"),
            ]
        assert main(argv) == 0
        assert verify_run(output)


@pytest.mark.parametrize("firmware", ["5.0.1", "5.1.1"])
def test_stand_fixture_rejected_before_any_network(monkeypatch, tmp_path, fixture_path, firmware):
    def forbidden(*args):
        raise AssertionError("network reached before readiness")

    monkeypatch.setattr("spot_deploy.network.wired_route", forbidden)
    robot = robot_config(tmp_path)
    config = json.loads(robot.read_text())
    config["expected_firmware"] = firmware
    robot.write_text(json.dumps(config))
    out = tmp_path / "stand"
    assert (
        main(
            [
                "stand",
                "--manifest",
                str(fixture_path / "manifest.json"),
                "--envelope",
                str(fixture_path / "envelope.json"),
                "--robot",
                str(robot),
                "--duration",
                ".5",
                "--execute",
                "--operator",
                "Adam",
                "--safety-operator",
                "Xun",
                "--output",
                str(out),
            ]
        )
        == 1
    )
    assert "fixture" in json.loads((out / "result.json").read_text())["error"]
    assert not verify_run(out)


def test_failed_replay_completion(tmp_path, fixture_path):
    bad = tmp_path / "bad.jsonl"
    first = (fixture_path / "states.jsonl").read_text().splitlines()[0]
    bad.write_text(first + "\n" + first + "\n")
    out = tmp_path / "run"
    assert (
        main(
            [
                "replay",
                "--manifest",
                str(fixture_path / "manifest.json"),
                "--states",
                str(bad),
                "--output",
                str(out),
            ]
        )
        == 1
    )
    assert (out / "COMPLETE.json").exists()
    assert not verify_run(out)


def test_errors_redact_input_and_sdk_secrets(capsys, tmp_path):
    robot = robot_config(tmp_path)
    data = json.loads(robot.read_text())
    data["password"] = "EXAMPLE_SECRET_SHOULD_NEVER_APPEAR"
    robot.write_text(json.dumps(data))
    out = tmp_path / "bad"
    assert main(["preflight", "--robot", str(robot), "--output", str(out)]) == 1
    assert data["password"] not in capsys.readouterr().err
    assert data["password"] not in (out / "result.json").read_text()
    assert data["password"] not in safe_error(RuntimeError(data["password"]))


def test_output_is_never_overwritten(tmp_path):
    out = tmp_path / "existing"
    out.mkdir()
    (out / "sentinel").write_text("preserve")
    assert main(["readiness", "--output", str(out)]) == 1
    assert list(out.iterdir()) == [out / "sentinel"]
