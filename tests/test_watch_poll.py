import json
from types import SimpleNamespace

import pytest
from pydantic import ValidationError

from spot_deploy.contracts import ContractError, JOINTS, RobotConfig, WatchRobotConfig
from spot_deploy.sdk_read import decode_full_state


def robot_config():
    return dict(schema_version=1, endpoint="10.0.0.3", expected_serial="test-robot",
                expected_firmware="5.0.1", expected_host="test-host", wired_interface="eth0",
                rpc_timeout_s=3, joint_control_feature="joint_control")


def full_state():
    from bosdyn.api.robot_state_pb2 import RobotState

    message = RobotState()
    kin = message.kinematic_state
    kin.acquisition_timestamp.seconds = 1000
    kin.acquisition_timestamp.nanos = 250000000
    kin.transforms_snapshot.child_to_parent_edge_map["odom"].parent_frame_name = ""
    body = kin.transforms_snapshot.child_to_parent_edge_map["body"]
    body.parent_frame_name = "odom"
    for axis in ("w", "x", "y", "z"):
        setattr(body.parent_tform_child.rotation, axis, .5)
    kin.velocity_of_body_in_odom.linear.x = .25
    for i, name in reversed(list(enumerate(JOINTS))):
        joint = kin.joint_states.add(name=name.replace("_", "."))
        joint.position.value = i / 10
        joint.velocity.value = -i / 10
        joint.load.value = i
    return message


def test_unary_state_maps_names_and_uses_frame_snapshot():
    state = decode_full_state(full_state(), received=5.)
    assert state.positions == [i / 10 for i in range(19)]
    assert state.velocities == [-i / 10 for i in range(19)]
    assert state.loads == list(range(19))
    assert state.odom_quaternion_wxyz == [.5] * 4
    assert state.linear_velocity_odom == [.25, 0, 0]
    assert state.robot_time_s == 1000.25 and state.received_monotonic_s == 5.


@pytest.mark.parametrize("name", ["unknown", "fl.hx"])
def test_unary_state_rejects_unknown_or_duplicate_joints(name):
    message = full_state()
    message.kinematic_state.joint_states[0].name = name
    with pytest.raises(ContractError, match="19 canonical"):
        decode_full_state(message)


def test_reviewed_firmware_contract_accepts_5_0_1_but_not_unknown_versions():
    data = robot_config()
    assert WatchRobotConfig.model_validate(data).expected_firmware == "5.0.1"
    assert RobotConfig.model_validate(data).expected_firmware == "5.0.1"
    assert RobotConfig.model_validate(data).joint_control_feature == "joint_level_control"
    assert RobotConfig.model_validate(data | {"joint_control_feature": "joint_level_control"}).joint_control_feature == "joint_level_control"
    with pytest.raises(ValidationError):
        RobotConfig.model_validate(data | {"joint_control_feature": "arbitrary_feature"})
    for firmware in ("4.1.0", "5.0.2", "6.0.0"):
        for cls in (RobotConfig, WatchRobotConfig):
            with pytest.raises(ValidationError):
                cls.model_validate(data | {"expected_firmware": firmware})


def test_watch_poll_cli_selects_read_only_profile_and_disables_streaming(monkeypatch, tmp_path):
    from spot_deploy.cli import main

    calls = []
    class Reader:
        def __init__(self, config):
            assert type(config) is WatchRobotConfig
        def connect(self, *, streaming):
            assert streaming is False
            calls.append("connect")
        def snapshot(self):
            return dict(identity_matches=True, joint_layout_matches=True, has_arm=True,
                        joint_control_licensed=False, arm_stowed=False, active_fault_count=1,
                        estop_ready=False)
        def close(self):
            calls.append("close")
    def watch(reader, duration, record, source):
        assert duration == 3600 and source == "poll"
        calls.append("poll")
        return {"motion_commands": 0}
    monkeypatch.setattr("spot_deploy.network.wired_route", lambda config: {})
    monkeypatch.setattr("spot_deploy.sdk_read.ReadOnlySpot", Reader)
    monkeypatch.setattr("spot_deploy.runtime.watch", watch)
    path = tmp_path / "robot.json"
    path.write_text(json.dumps(robot_config()))
    assert main(["watch", "--robot", str(path), "--state-source", "poll", "--duration", "3600",
                 "--output", str(tmp_path / "watch")]) == 0
    assert calls == ["connect", "poll", "close"]


@pytest.mark.parametrize("stale", [False, True])
def test_watch_poll_updates_status_and_stops_on_stale_robot_time(monkeypatch, stale):
    from spot_deploy.runtime import watch

    clock = [0.]
    monkeypatch.setattr("spot_deploy.runtime.time.monotonic", lambda: clock[0])
    monkeypatch.setattr("spot_deploy.runtime.time.sleep", lambda delay: clock.__setitem__(0, clock[0] + delay))
    events = []
    def read_state():
        state = decode_full_state(full_state(), received=clock[0])
        state.robot_time_s = 1000 + clock[0]
        return state
    reader = SimpleNamespace(read_state=read_state,
        robot_now=lambda: 1000 + clock[0] + (1 if stale else 0),
        health=lambda: {"battery_percent": 93, "estop_ready": False, "fault_count": 0})
    record = SimpleNamespace(event=lambda kind, **data: events.append((kind, data)))
    if stale:
        with pytest.raises(ContractError, match="timestamp stale"):
            watch(reader, .12, record, "poll")
    else:
        result = watch(reader, .12, record, "poll")
        assert result["samples"] == 3 and result["motion_commands"] == 0
        assert result["stream"] is None and result["state_source"] == "poll"
        assert [kind for kind, _ in events] == ["state", "health", "state", "state"]
