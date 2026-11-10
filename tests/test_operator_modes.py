import json
import os
from types import SimpleNamespace

from spot_deploy.operator_modes import OperatorModes


def test_only_click_for_current_phase_can_request_handover(tmp_path):
    rd, wr = os.pipe()
    try:
        modes = OperatorModes(rd, tmp_path, SimpleNamespace(event=lambda *args, **kwargs: None))
        modes.publish("native")
        old = json.loads(modes.path.read_text())["token"].encode() + b"\n"
        os.write(wr, old)
        assert modes.requested()
        modes.publish("relic")
        modes.publish("native")
        os.write(wr, old)
        assert not modes.requested()
        assert not modes.requested()
    finally:
        os.close(wr)
        os.close(rd)


def test_attached_camera_fleet_is_never_terminated(monkeypatch, tmp_path):
    import io
    from pathlib import Path

    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    import teleop_services

    monkeypatch.setattr(
        teleop_services,
        "urlopen",
        lambda *a, **kw: io.BytesIO(b'{"schema":1,"state":"running","cameras":{}}'),
    )

    def no_spawn(*args, **kwargs):
        raise AssertionError("unexpected second acquisition")

    monkeypatch.setattr(teleop_services.subprocess, "Popen", no_spawn)
    rd, wr = os.pipe()
    os.close(wr)
    try:
        teleop_services.fleet("http://example.invalid", tmp_path, rd)
    finally:
        os.close(rd)
