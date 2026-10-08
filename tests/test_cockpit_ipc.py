import socket
import threading
import time
from types import SimpleNamespace

import pytest

from spot_deploy.cockpit_ipc import CockpitIPC
from spot_deploy.contracts import load
from spot_deploy.estop_bridge import StatusWriter
from spot_deploy.joystick_estop import ButtonEvent, JoystickProfile, run_joystick_bridge


def test_ipc_owner_only_and_no_rearm_verb(tmp_path):
    tmp_path.chmod(0o700)
    ipc = CockpitIPC(tmp_path / "stop.sock")
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    client.bind(str(tmp_path / "client"))
    client.settimeout(0.1)
    try:
        client.sendto(b"ALLOW", str(ipc.path))
        assert not ipc.poll()
        assert not ipc.pending
        client.sendto(b"CUT", str(ipc.path))
        assert ipc.poll()
        status = SimpleNamespace(base={"session": "fixture"})
        ipc.publish(SimpleNamespace(axes={0: 0.2}, buttons={2: True}), status, "stopped", True)
        import json

        packet = json.loads(client.recv(8192))
        assert packet["state"] == "stopped" and packet["robot_confirmed"]
        assert packet["buttons"]["2"]
        # A disappeared subscriber cannot stall the E-stop loop.
        ipc.pending = [(str(tmp_path / "absent"), b"STATE")]
        ipc.publish(SimpleNamespace(), status, "stopped", True)
    finally:
        client.close()
        ipc.close()
    tmp_path.chmod(0o755)
    with pytest.raises(ValueError, match="owner-only"):
        CockpitIPC(tmp_path / "no.sock")


def test_cut_latches_before_checkin_and_does_not_rearm(tmp_path, monkeypatch):
    from pathlib import Path

    profile = load(Path(__file__).parents[1] / "examples/joystick-estop.json", JoystickProfile)
    tmp_path.chmod(0o700)
    monkeypatch.setenv("SPOT_JOYSTICK_IPC", str(tmp_path / "stop.sock"))
    stop = threading.Event()
    endpoint = SimpleNamespace(
        endpoint_id="fixture",
        levels=[],
        check_in=lambda allowed: levels.append(allowed),
        stop=lambda: levels.append(False),
    )
    levels = endpoint.levels
    start = time.monotonic()
    calls = 0

    def poll():
        nonlocal calls
        calls += 1
        if calls == 1:
            return [ButtonEvent(profile.stop_button, True, True), ButtonEvent(profile.rearm_button, False, True)]
        if calls == 2:
            return [ButtonEvent(profile.stop_button, False)]
        if 0.6 < time.monotonic() - start < 0.64:
            return [ButtonEvent(profile.rearm_button, True)]
        return []

    link = SimpleNamespace(metadata={}, poll=poll, axes={0: 0.4}, buttons={2: True})
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64, robot_hash="2" * 64)
    record = SimpleNamespace(event=lambda *a, **kw: None)
    errors = []

    def run():
        try:
            run_joystick_bridge(profile, link, status, record, endpoint, stop)
        except Exception as e:
            errors.append(e)

    thread = threading.Thread(target=run)
    thread.start()
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    client.bind(str(tmp_path / "client"))
    client.settimeout(0.5)
    try:
        deadline = time.monotonic() + 2
        while True not in levels and time.monotonic() < deadline:
            time.sleep(0.02)
        assert True in levels
        client.sendto(b"CUT", str(tmp_path / "stop.sock"))
        import json

        assert json.loads(client.recv(8192))["state"] == "stopped"
        index = len(levels)
        time.sleep(0.12)
        assert levels[index:] and not any(levels[index:])
    finally:
        stop.set()
        thread.join(2)
        client.close()
        status.close()
    assert not errors


def test_ipc_startup_failure_still_stops_registered_endpoint(tmp_path, monkeypatch):
    from pathlib import Path

    profile = load(Path(__file__).parents[1] / "examples/joystick-estop.json", JoystickProfile)
    tmp_path.chmod(0o755)
    monkeypatch.setenv("SPOT_JOYSTICK_IPC", str(tmp_path / "blocked.sock"))
    calls = []
    endpoint = SimpleNamespace(endpoint_id="fixture", stop=lambda: calls.append("stop"))
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64)
    try:
        with pytest.raises(ValueError, match="owner-only"):
            run_joystick_bridge(
                profile,
                SimpleNamespace(metadata={}),
                status,
                SimpleNamespace(event=lambda *a, **k: None),
                endpoint,
            )
        assert calls == ["stop"]
    finally:
        status.close()
