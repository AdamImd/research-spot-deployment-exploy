import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from bosdyn.api import estop_pb2, robot_state_pb2
from pydantic import ValidationError

from spot_deploy import deployment, estop_cli
from spot_deploy.contracts import ContractError, RobotConfig
from spot_deploy.readiness import HARDWARE_GATE, TABLET_DESCRIPTION, report
from spot_deploy.sdk_read import estop_observation
from test_readiness_cli import robot_config


def messages():
    state = robot_state_pb2.RobotState()
    for kind in (robot_state_pb2.EStopState.TYPE_HARDWARE,
                 robot_state_pb2.EStopState.TYPE_SOFTWARE):
        state.estop_states.add(type=kind, state=robot_state_pb2.EStopState.STATE_NOT_ESTOPPED)
    return state, estop_pb2.EstopSystemStatus(stop_level=estop_pb2.ESTOP_LEVEL_NONE)


def test_empty_endpoint_configuration_requires_explicit_tablet_mode():
    state, status = messages()
    assert not estop_observation(SimpleNamespace(estop_authority="sdk_endpoint"), state, status)["estop_ready"]
    result = estop_observation(SimpleNamespace(estop_authority="tablet"), state, status)
    assert result["estop_ready"] and result["estop_endpoint_count"] == 0
    assert len(result["estop_states"]) == 2


@pytest.mark.parametrize("failure", ["hardware_stop", "software_stop", "unknown", "missing",
                                    "service_cut", "service_settle", "service_unknown"])
def test_tablet_rejects_stopped_unknown_or_incomplete_status(failure):
    state, status = messages()
    cls = robot_state_pb2.EStopState
    if failure == "hardware_stop":
        state.estop_states[0].state = cls.STATE_ESTOPPED
    elif failure == "software_stop":
        state.estop_states[1].state = cls.STATE_ESTOPPED
    elif failure == "unknown":
        state.estop_states[1].state = cls.STATE_UNKNOWN
    elif failure == "missing":
        del state.estop_states[1]
    else:
        status.stop_level = {"service_cut": estop_pb2.ESTOP_LEVEL_CUT,
                             "service_settle": estop_pb2.ESTOP_LEVEL_SETTLE_THEN_CUT,
                             "service_unknown": estop_pb2.ESTOP_LEVEL_UNKNOWN}[failure]
    assert not estop_observation(SimpleNamespace(estop_authority="tablet"), state, status)["estop_ready"]


def test_tablet_and_local_bridge_are_mutually_exclusive(tmp_path):
    data = json.loads(robot_config(tmp_path).read_text())
    assert RobotConfig.model_validate(data).estop_authority == "sdk_endpoint"
    data.update(estop_authority="tablet", hardware_estop={
        "profile_sha256": "1" * 64, "status_file": "/tmp/stop.json", "max_status_age_s": .25})
    with pytest.raises(ValidationError, match="tablet authority"):
        RobotConfig.model_validate(data)


def test_tablet_keeps_stop_evidence_gate(tmp_path):
    path = robot_config(tmp_path)
    data = json.loads(path.read_text()) | {"estop_authority": "tablet"}
    path.write_text(json.dumps(data))
    checks = report(robot_path=path)["checks"]
    gate = next(c for c in checks if c["gate"] == HARDWARE_GATE)
    assert gate["description"] == TABLET_DESCRIPTION
    assert gate["status"] == "unverified"


@pytest.mark.parametrize("mode", ["configure", "bridge"])
def test_tablet_disables_local_estop_writes_before_device_or_network_access(tmp_path, monkeypatch, mode):
    path = robot_config(tmp_path)
    data = json.loads(path.read_text()) | {"estop_authority": "tablet"}
    config = RobotConfig.model_validate(data)
    monkeypatch.setattr(estop_cli, "load", lambda p, cls: config if cls is RobotConfig else object())
    monkeypatch.setattr(estop_cli, "sha256", lambda p: "1" * 64)
    args = SimpleNamespace(input_type="joystick", profile=path, robot=path, mode=mode, execute=True)
    with pytest.raises(ContractError, match="disabled for tablet authority"):
        estop_cli.execute(args, None)


def test_tablet_bundle_omits_local_bridge_check_but_keeps_other_gates(tmp_path, monkeypatch):
    from spot_deploy.cli import main

    monkeypatch.setattr(deployment, "source_identity",
                        lambda: {"dirty": False, "commit": "a" * 40, "source_sha256": "b" * 64})
    path = robot_config(tmp_path)
    path.write_text(json.dumps(json.loads(path.read_text()) | {"estop_authority": "tablet"}))
    root = Path(__file__).resolve().parents[1]
    bundle = tmp_path / "bundle"
    assert main(["prepare-deployment", "--manifest", str(root / "policies/relic-exploy-standing/manifest.json"),
                 "--robot", str(path), "--output", str(bundle)]) == 0
    result = deployment.check(bundle, check_estop=True)
    checks = {c["check"]: c for c in result["checks"]}
    assert checks["local_estop_status"]["status"] == "passed"
    assert checks[HARDWARE_GATE]["status"] == "unverified"
    assert checks["fresh_preflight"]["status"] == "unverified"
    assert not result["passed"] and not result["live_ready"]
