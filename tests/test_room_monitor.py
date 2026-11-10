"""Check the fleet snapshot contract and calibrated room render projection."""

import io
import json
import runpy
from pathlib import Path
from unittest.mock import patch
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"


@pytest.fixture(autouse=True)
def tools_path(monkeypatch):
    monkeypatch.syspath_prepend(str(TOOLS))


@pytest.mark.parametrize(
    "patch_meta",
    [
        {"live": False},
        {"camera": "other"},
        {"preview_age_s": 1.01},
        {"capture_age_s": -0.1},
        {"preview_age_s": float("nan")},
    ],
)
def test_live_camera_rejects_invalid_snapshot(patch_meta):
    fetch = runpy.run_path(str(TOOLS / "room_camera_panel.py"))["live_frame"]
    response = io.BytesIO(b"\xff\xd8example")
    response.headers = {
        "X-Fleet-Frame": json.dumps(
            dict(camera="flir-5mp", live=True, capture_age_s=0.02, preview_age_s=0.01) | patch_meta
        )
    }
    with patch.dict(fetch.__globals__, urlopen=lambda url, timeout: response):
        with pytest.raises(ValueError):
            fetch("http://localhost", "flir-5mp")


def test_room_projection_preserves_calibrated_pixels():
    np = pytest.importorskip("numpy")
    cv2 = pytest.importorskip("cv2")
    pytest.importorskip("mujoco")
    api = runpy.run_path(str(TOOLS / "room_cameras.py"))
    from room_array import DEFAULT, load_array

    if not DEFAULT.exists():
        pytest.skip("local room calibration not installed")
    cfg = load_array()
    for c in cfg["cameras"].values():
        inverse, fov = api["optical_map"](c)
        h, w = inverse.shape[:2]
        y, x = np.meshgrid(
            np.linspace(0, h - 1, 11, dtype=int), np.linspace(0, w - 1, 11, dtype=int)
        )
        unit = (inverse[y, x] - (api["SIDE"] - 1) / 2) * (
            2 * np.tan(np.deg2rad(fov / 2)) / api["SIDE"]
        )
        rays = np.concatenate([unit, np.ones((*unit.shape[:2], 1))], axis=-1).reshape(-1, 1, 3)
        project = cv2.fisheye.projectPoints if c["model"] == "fisheye" else cv2.projectPoints
        pixel, _ = project(
            rays, np.zeros(3), np.zeros(3), np.array(c["K"]), np.array(c["distortion"])
        )
        result = (pixel.reshape(*x.shape, 2) + 0.5) * np.array([w, h]) / c["image_size"] - 0.5
        np.testing.assert_allclose(result, np.stack([x, y], axis=-1), rtol=0, atol=0.06)


def test_snapshot_preserves_identity_rotation_and_roi():
    fetch = runpy.run_path(str(TOOLS / "room_camera_panel.py"))["live_frame"]
    meta = dict(
        camera="kinect-1",
        live=True,
        capture_age_s=0.12,
        preview_age_s=0.01,
        receiver_session="session-3",
        frame_key=["epoch", 42],
        rotation_deg=180,
        roi=[0, 0, 3840, 2160],
    )
    response = io.BytesIO(b"\xff\xd8example")
    response.headers = {"X-Fleet-Frame": json.dumps(meta)}
    with patch.dict(fetch.__globals__, urlopen=lambda url, timeout: response):
        token, expires, jpeg, rotation, roi = fetch("http://localhost", "kinect-1")
    assert token == ("session-3", ["epoch", 42])
    assert 0 < expires - __import__("time").monotonic() <= 0.88
    assert jpeg.startswith(b"\xff\xd8") and rotation == 180 and roi == meta["roi"]


@pytest.mark.parametrize("failures", [1, 3])
def test_readonly_connection_closes_and_rechecks_after_timeouts(
    tmp_path, monkeypatch, capsys, failures
):
    from types import SimpleNamespace
    from bosdyn.client.exceptions import TimedOutError
    from spot_deploy import contracts, network, sdk_read
    import teleop_services

    readers = []
    verified = []
    monkeypatch.setattr(contracts, "load", lambda *a: SimpleNamespace(wired_interface="test"))
    monkeypatch.setattr(network, "wired_route", lambda cfg: None)

    class Reader:
        connection_stage = "authentication"
        closed = False

        def __init__(self, config):
            readers.append(self)

        def connect(self, streaming):
            assert streaming is False
            if len(readers) <= failures:
                raise TimedOutError(None, "timed out")

        def snapshot(self):
            verified.append(True)
            return dict(identity_matches=True, joint_layout_matches=True, has_arm=True)

        def health(self):
            assert verified
            return dict(estop_ready=True)

        def close(self):
            self.closed = True

    monkeypatch.setattr(sdk_read, "ReadOnlySpot", Reader)
    monkeypatch.setattr(teleop_services.select, "select", lambda *a: (verified, [], []))
    if failures == 3:
        with pytest.raises(RuntimeError, match="authentication.*three read-only attempts"):
            teleop_services.connection(tmp_path, 0)
        assert not verified
    else:
        teleop_services.connection(tmp_path, 0)
        assert len(readers) == 2 and verified
    assert all(r.closed for r in readers)
    assert "reconnecting" in capsys.readouterr().out
