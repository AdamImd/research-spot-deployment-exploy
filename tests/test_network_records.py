import json
import socket
from pathlib import Path
from types import SimpleNamespace

import pytest

from spot_deploy.contracts import ContractError
from spot_deploy.network import wired_route
from spot_deploy.records import RunRecord, verify_run


@pytest.mark.parametrize(
    "route_interface,wireless,up,passes",
    [
        ("eth0", False, True, True),
        ("wlan0", False, True, False),
        ("eth0", True, True, False),
        ("eth0", False, False, False),
    ],
)
def test_network_guard_uses_observed_route(monkeypatch, route_interface, wireless, up, passes):
    monkeypatch.setattr(socket, "gethostname", lambda: "rpm")
    monkeypatch.setattr(socket, "getaddrinfo", lambda *a: [(0, 0, 0, "", ("10.0.0.2", 443))])
    observed = []

    def run(command, **kwargs):
        observed.append(command)
        return SimpleNamespace(
            stdout=json.dumps([{"dev": route_interface, "prefsrc": "10.0.0.20"}])
        )

    monkeypatch.setattr("spot_deploy.network.subprocess.run", run)
    monkeypatch.setattr(Path, "is_dir", lambda p: True)
    monkeypatch.setattr(Path, "exists", lambda p: wireless if p.name == "wireless" else True)
    monkeypatch.setattr(Path, "read_text", lambda p: "up" if up else "down")
    config = SimpleNamespace(expected_host="rpm", endpoint="robot", wired_interface="eth0")
    if passes:
        assert wired_route(config)["interface"] == "eth0"
    else:
        with pytest.raises(ContractError):
            wired_route(config)
    assert observed == [["ip", "-j", "route", "get", "10.0.0.2"]]


def test_wrong_host_rejected_before_resolving(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "unexpected")
    with pytest.raises(ContractError, match="host"):
        wired_route(SimpleNamespace(expected_host="rpm"))


def test_completion_hashes_detect_corruption(tmp_path):
    record = RunRecord(tmp_path / "run", "test")
    record.event("example", value=1)
    record.finish("passed", {"ok": True})
    assert verify_run(record.directory)
    (record.directory / "events.jsonl").write_text("corrupt")
    assert not verify_run(record.directory)


def test_recorder_failure_is_visible(tmp_path):
    record = RunRecord(tmp_path / "run", "test")
    record._writer_error = True
    with pytest.raises(ContractError, match="unavailable"):
        record.event("example")
    with pytest.raises(ContractError, match="shutdown"):
        record.finish("passed", {})
    assert (record.directory / "FAILED.json").exists()
    assert not (record.directory / "COMPLETE.json").exists()
