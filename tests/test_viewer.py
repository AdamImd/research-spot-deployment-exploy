import http.client
import json
import threading

import pytest

from spot_deploy.contracts import ContractError
from spot_deploy.kinematics import from_urdf, schematic
from spot_deploy.viewer import DemoFeed, RunFeed, make_server


def run_dir(tmp_path):
    (tmp_path / "run.json").write_text('{"mode":"shadow"}')
    return tmp_path


def test_partial_lines_and_prediction_provenance(tmp_path, state):
    feed = RunFeed(run_dir(tmp_path))
    path = tmp_path / "events.jsonl"
    event = {
        "event": "shadow_sample",
        "state": state.model_dump(),
        "targets": [1] * 19,
        "raw_actions": [2] * 12,
    }
    raw = json.dumps(event)
    path.write_text(raw[:40])
    assert not feed.response()["frames"]
    with path.open("a") as stream:
        stream.write(raw[40:] + "\n")
        stream.write(
            json.dumps(
                {"event": "command", "state": state.model_dump(), "positions": [3] * 19, "key": 42}
            )
            + "\n"
        )
    values = feed.response()["frames"]
    assert len(values) == 2
    assert values[0]["command"] is None
    assert values[1]["target"]["positions"] == [1] * 19
    assert values[1]["command"]["positions"] == [3] * 19
    assert len(feed.response(after=1)["frames"]) == 1
    path.write_text("")
    with pytest.raises(ContractError, match="truncated"):
        feed.response()


def test_malformed_telemetry_never_shown(tmp_path, state):
    feed = RunFeed(run_dir(tmp_path))
    path = tmp_path / "events.jsonl"
    event = {
        "event": "policy",
        "state": state.model_dump(),
        "targets": [float("nan")] * 19,
        "raw_actions": [0] * 12,
    }
    path.write_text("junk\n" + json.dumps(event) + "\n" + "x" * 270000 + "\n")
    response = feed.response()
    assert not response["frames"] and response["skipped"] == 3
    assert not feed.line


@pytest.mark.parametrize("after", [0, 10000])
def test_log_backlog_requests_more_even_at_end_of_loaded_frames(tmp_path, state, after):
    feed = RunFeed(run_dir(tmp_path))
    path = tmp_path / "events.jsonl"
    with path.open("w") as stream:
        for index in range(60):
            sample = state.model_copy(update={"robot_time_s": state.robot_time_s + index / 50})
            stream.write(json.dumps({"event": "state", "state": sample.model_dump(),
                                     "extra_diagnostic": "x" * 60000}) + "\n")
    first = feed.response(after=after)
    assert first["more"] and first["pending_bytes"] > 0
    assert 0 < first["latest_id"] < 60
    cursor = first["frames"][-1]["id"] if first["frames"] else after
    last = feed.response(after=cursor)
    assert last["latest_id"] == 60
    assert last["pending_bytes"] == 0 and not last["more"]
    assert feed.frames[-1]["state"]["robot_time_s"] == pytest.approx(state.robot_time_s + 59 / 50)
    assert feed.skipped == 0


def test_http_readonly_and_loopback_origin():
    server = make_server(DemoFeed(), 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
    try:
        for path in ("/", "/app.js", "/style.css", "/api/model", "/api/frames"):
            client.request("GET", path)
            response = client.getresponse()
            assert response.status == 200
            assert "frame-ancestors 'none'" in response.headers["Content-Security-Policy"]
            body = response.read()
            if path == "/api/frames":
                assert json.loads(body)["demo"] is True
        for headers in ({"Origin": "https://evil.example"}, {"Host": "evil.example"}):
            client.request("GET", "/api/frames", headers=headers)
            response = client.getresponse()
            assert response.status == 403
            response.read()
        client.request("POST", "/arm")
        response = client.getresponse()
        assert response.status == 501
        response.read()
        client.request("GET", "/../pyproject.toml")
        response = client.getresponse()
        assert response.status == 404
        response.read()
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        thread.join(2)


def urdf_text():
    model = schematic()
    joints = []
    for joint in model["joints"]:
        xyz = " ".join(map(str, joint["xyz"]))
        axis = " ".join(map(str, joint["axis"]))
        joints.append(
            f'<joint name="{joint["name"]}" type="revolute"><parent link="{joint["parent"]}"/><child link="{joint["child"]}"/><origin xyz="{xyz}"/><axis xyz="{axis}"/></joint>'
        )
    return '<robot name="test">' + "".join(reversed(joints)) + "</robot>"


def test_urdf_tree_is_sorted_and_meshlocations_not_served(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text(urdf_text())
    model = from_urdf(path)
    seen = {model["root"]}
    for joint in model["joints"]:
        assert joint["parent"] in seen
        seen.add(joint["child"])
    assert len(model["joints"]) == 19 and len(model["sha256"]) == 64


@pytest.mark.parametrize(
    "text",
    [
        "<bad",
        '<!DOCTYPE robot [<!ENTITY x "bad">]><robot/>',
        "<robot/>",
        urdf_text().replace('axis xyz="1 0 0"', 'axis xyz="0 0 0"'),
    ],
)
def test_bad_urdf_is_rejected(tmp_path, text):
    path = tmp_path / "bad.urdf"
    path.write_text(text)
    with pytest.raises(ContractError):
        from_urdf(path)
