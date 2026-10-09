from types import SimpleNamespace

import pytest

from spot_deploy.contracts import ContractError, JOINTS
from spot_deploy.sdk_control import command_proto
from spot_deploy.sdk_read import ReadOnlySpot, decode_state
from spot_deploy.safety import Command


def test_sdk_joint_order_contract():
    from bosdyn.api.spot import spot_constants_pb2

    values = spot_constants_pb2.DESCRIPTOR.enum_types_by_name["JointIndex"].values
    names = [x.name.replace("JOINT_INDEX_", "").lower().replace("a0_", "arm0_") for x in values]
    assert tuple(names) == JOINTS
    assert [x.number for x in values] == list(range(19))


def test_full_command_proto(manifest, envelope):
    command = Command(tuple(range(19)), 1000.05, 4)
    proto = command_proto(command, manifest, envelope)
    joint = proto.joint_command
    assert list(joint.position) == list(range(19))
    assert (
        len(joint.velocity)
        == len(joint.load)
        == len(joint.gains.k_q_p)
        == len(joint.gains.k_qd_p)
        == 19
    )
    assert joint.user_command_key == 4
    assert joint.end_time.seconds == 1000
    assert joint.end_time.nanos == pytest.approx(50000000, abs=1000)
    assert joint.velocity_safety_limit.value == min(envelope.velocity_max)
    assert joint.extrapolation_duration.nanos == 0


@pytest.mark.parametrize("omit_header", [False, True])
def test_decode_real_proto_and_reject_errors(omit_header):
    from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
    from bosdyn.api.header_pb2 import CommonError

    message = RobotStateStreamResponse()
    if not omit_header:
        message.header.error.code = CommonError.CODE_OK
    message.joint_states.position.extend([0] * 19)
    message.joint_states.velocity.extend([0] * 19)
    message.joint_states.load.extend([0] * 19)
    message.joint_states.acquisition_timestamp.seconds = 1000
    message.kinematic_state.odom_tform_body.rotation.w = 1
    state = decode_state(message, received=100)
    assert state.robot_time_s == 1000
    assert len(state.positions) == 19
    message.header.error.code = CommonError.CODE_INTERNAL_SERVER_ERROR
    with pytest.raises(ContractError, match="header"):
        decode_state(message)
    message.header.error.code = CommonError.CODE_UNSPECIFIED
    with pytest.raises(ContractError, match="header"):
        decode_state(message)
    message.ClearField("header")
    message.joint_states.ClearField("position")
    with pytest.raises(ValueError):
        decode_state(message)


def test_read_capability_has_no_motion_methods():
    assert not any(
        hasattr(ReadOnlySpot, name)
        for name in (
            "power_on",
            "power_off",
            "acquire",
            "activate",
            "start_command_stream",
            "stand",
        )
    )


@pytest.mark.parametrize("pitch", [0, .3])
def test_initial_height_is_vertical_ground_relative_with_arbitrary_odom_origin(pitch):
    import math
    from bosdyn.api.robot_state_pb2 import RobotState
    from bosdyn.client.math_helpers import Quat
    from spot_deploy.sdk_read import body_height_from_state

    message = RobotState()
    edges = message.kinematic_state.transforms_snapshot.child_to_parent_edge_map
    edges["odom"].parent_tform_child.rotation.w = 1
    body, ground = edges["body"], edges["gpe"]
    body.parent_frame_name = ground.parent_frame_name = "odom"
    ground.parent_tform_child.position.x = 3
    ground.parent_tform_child.position.z = 7
    ground.parent_tform_child.rotation.CopyFrom(Quat.from_pitch(pitch).to_proto())
    body.parent_tform_child.rotation.w = 1
    body.parent_tform_child.position.x = 4
    # GPE's tilted plane at x=4 has z=7-tan(pitch).
    body.parent_tform_child.position.z = 7 - math.tan(pitch) + .473
    assert body_height_from_state(message)["height_m"] == pytest.approx(.473)
    del edges["gpe"]
    with pytest.raises(ContractError, match="requires"):
        body_height_from_state(message)


def test_initial_height_rejects_stale_full_state(monkeypatch):
    monkeypatch.setattr("spot_deploy.sdk_read.body_height_from_state",
                        lambda message: {"height_m": .48, "robot_time_s": 100})
    reader = ReadOnlySpot(SimpleNamespace(rpc_timeout_s=.2))
    reader.state_client = SimpleNamespace(get_robot_state=lambda **kwargs: object())
    reader.robot_now = lambda: 100.3
    with pytest.raises(ContractError, match="older than 250"):
        reader.read_body_height()


def test_state_stream_cancelled_on_close():
    reader = ReadOnlySpot(SimpleNamespace(rpc_timeout_s=0.2))
    calls = []
    reader._stream = SimpleNamespace(cancel=lambda: calls.append("cancel"))
    reader.close()
    assert calls == ["cancel"]


@pytest.mark.parametrize("failure", ["authentication", "time_sync"])
def test_failed_connection_cleanup_never_starts_time_sync(monkeypatch, failure):
    calls = []

    def fail():
        raise TimeoutError("fixture timeout")

    class Robot:
        def authenticate(self, *args, **kwargs):
            calls.append("authentication")
            if failure == "authentication":
                fail()

        @property
        def time_sync(self):
            calls.append("time_sync_started")
            return SimpleNamespace(wait_for_sync=lambda **_: fail(),
                                   stop=lambda: calls.append("time_sync_stopped"))

    monkeypatch.setenv("BOSDYN_CLIENT_USERNAME", "fixture")
    monkeypatch.setenv("BOSDYN_CLIENT_PASSWORD", "fixture")
    monkeypatch.setattr("bosdyn.client.create_standard_sdk", lambda _: SimpleNamespace(
        register_service_client=lambda _: None, create_robot=lambda _: Robot()))
    reader = ReadOnlySpot(SimpleNamespace(endpoint="example.invalid", rpc_timeout_s=.2))
    with pytest.raises(TimeoutError):
        reader.connect(streaming=False)
    assert reader.connection_stage == failure
    reader.close()
    assert calls == (["authentication"] if failure == "authentication" else
                     ["authentication", "time_sync_started", "time_sync_stopped"])


@pytest.mark.parametrize("overridden", [False, True])
def test_native_stand_requires_current_processing_feedback(overridden):
    import threading
    from bosdyn.api import basic_command_pb2, robot_command_pb2
    from spot_deploy.sdk_control import ControlSpot

    response = robot_command_pb2.RobotCommandFeedbackResponse()
    mobility = response.feedback.synchronized_feedback.mobility_command_feedback
    status = basic_command_pb2.RobotCommandFeedbackStatus
    mobility.status = status.STATUS_COMMAND_OVERRIDDEN if overridden else status.STATUS_PROCESSING
    mobility.stand_feedback.status = basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING
    calls = []
    control = ControlSpot.__new__(ControlSpot)
    control.timeout = 0.1
    control.robot = SimpleNamespace(power_on=lambda **kwargs: calls.append("power_on"))
    control.command = SimpleNamespace(
        robot_command=lambda *args, **kwargs: 7,
        robot_command_feedback=lambda *args, **kwargs: response,
    )
    control.heartbeat = lambda: calls.append("heartbeat")
    if overridden:
        with pytest.raises(ContractError, match="interrupted"):
            control.native_stand(threading.Event())
    else:
        control.native_stand(threading.Event())
    assert control.power_owned
    assert calls == ["power_on", "heartbeat"]


@pytest.mark.parametrize(
    "parent_status,joint_status,expected",
    [(1, 1, True), (1, 0, False), (1, 2, "feedback error"), (2, 1, "no longer processing")],
)
def test_joint_activation_with_actual_sdk_feedback(parent_status, joint_status, expected):
    from bosdyn.api import basic_command_pb2, robot_command_pb2
    from spot_deploy.sdk_control import ControlSpot

    # Exercise the actual 5.1.1 schema, not a namespace accepting arbitrary names.
    response = robot_command_pb2.RobotCommandFeedbackResponse()
    response.feedback.full_body_feedback.status = parent_status
    response.feedback.full_body_feedback.joint_feedback.status = joint_status
    assert basic_command_pb2.JointCommand.Feedback.STATUS_ACTIVE == 1
    assert basic_command_pb2.JointCommand.Feedback.STATUS_ERROR == 2
    control = ControlSpot.__new__(ControlSpot)
    control.command_id, control.timeout = 42, .1
    calls = []

    def feedback(command_id, **kwargs):
        calls.append((command_id, kwargs))
        return response

    control.command = SimpleNamespace(robot_command_feedback=feedback)
    if isinstance(expected, str):
        with pytest.raises(ContractError, match=expected):
            control.active()
    else:
        assert control.active() is expected
    assert calls == [(42, {"timeout": .1})]


@pytest.mark.parametrize("firmware,licensed", [("5.1.1", True), ("5.0.1", True), ("5.0.1", False)])
def test_preflight_cli_only_uses_read_capability(monkeypatch, tmp_path, firmware, licensed):
    import json
    from spot_deploy.cli import main

    calls = []
    snapshot = {
        "identity_matches": True,
        "joint_layout_matches": True,
        "has_arm": True,
        "joint_control_licensed": licensed,
        "arm_stowed": True,
        "active_fault_count": 0,
        "estop_ready": True,
    }

    class ReadSpy:
        def __init__(self, config):
            calls.append("construct")

        def connect(self, *, streaming):
            assert streaming is False
            calls.append("connect")

        def snapshot(self):
            calls.append("snapshot")
            return snapshot

        def close(self):
            calls.append("close")

    monkeypatch.setattr("spot_deploy.sdk_read.ReadOnlySpot", ReadSpy)
    monkeypatch.setattr("spot_deploy.network.wired_route", lambda c: {"interface": "eth0"})
    path = tmp_path / "robot.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "endpoint": "example.invalid",
                "expected_serial": "fixture",
                "expected_firmware": firmware,
                "expected_host": "rpm",
                "wired_interface": "eth0",
                "rpc_timeout_s": 1,
                "joint_control_feature": "joint_control",
            }
        )
    )
    assert main(["preflight", "--robot", str(path), "--output", str(tmp_path / "run")]) == (0 if licensed else 1)
    assert calls == ["construct", "connect", "snapshot", "close"]
    result = json.loads((tmp_path / "run" / "result.json").read_text())
    assert result["checks"]["joint_control_license"] is licensed


def test_snapshot_with_actual_sdk_messages():
    from bosdyn.api import estop_pb2, payload_pb2, robot_id_pb2, robot_state_pb2
    from bosdyn.client.estop import EstopClient
    from bosdyn.client.license import LicenseClient
    from bosdyn.client.payload import PayloadClient
    from bosdyn.client.robot_id import RobotIdClient
    from spot_deploy.sdk_read import validate_snapshot

    robot_id = robot_id_pb2.RobotId(serial_number="test-serial")
    version = robot_id.software_release.version
    version.major_version, version.minor_version, version.patch_level = 5, 1, 1
    state = robot_state_pb2.RobotState()
    for name in JOINTS:
        state.kinematic_state.joint_states.add(name=name.replace("_", "."))
    state.manipulator_state.stow_state = robot_state_pb2.ManipulatorState.STOWSTATE_STOWED
    state.power_state.motor_power_state = robot_state_pb2.PowerState.STATE_OFF
    state.battery_states.add().charge_percentage.value = 90
    hardware = robot_state_pb2.HardwareConfiguration()
    hardware.skeleton.urdf = '<robot name="test"/>'
    estop = estop_pb2.EstopSystemStatus(stop_level=estop_pb2.ESTOP_LEVEL_NONE)
    estop.endpoints.add()
    payload = payload_pb2.Payload(GUID="test", name="arm", is_authorized=True, is_enabled=True)
    payload.mass_volume_properties.total_mass = 8
    config = SimpleNamespace(
        rpc_timeout_s=1,
        joint_control_feature="joint_level_control",
        estop_authority="sdk_endpoint",
        expected_serial="test-serial",
        expected_firmware="5.1.1",
    )
    reader = ReadOnlySpot(config)
    def feature_enabled(codes, **kwargs):
        assert codes == ["joint_level_control"]
        return {"joint_level_control": True}

    clients = {
        RobotIdClient.default_service_name: SimpleNamespace(get_id=lambda **k: robot_id),
        EstopClient.default_service_name: SimpleNamespace(get_status=lambda **k: estop),
        PayloadClient.default_service_name: SimpleNamespace(list_payloads=lambda **k: [payload]),
        LicenseClient.default_service_name: SimpleNamespace(
            get_feature_enabled=feature_enabled
        ),
    }
    reader.robot = SimpleNamespace(ensure_client=clients.__getitem__, has_arm=lambda **k: True)
    reader.state_client = SimpleNamespace(
        get_robot_state=lambda **k: state, get_robot_hardware_configuration=lambda **k: hardware
    )
    result = reader.snapshot()
    assert all(validate_snapshot(result, config, for_control=True).values())
    assert len(result["robot_model_sha256"]) == len(result["payload_config_sha256"]) == 64
    payload.mass_volume_properties.com_pos_rt_payload.x = 0.1
    assert reader.snapshot()["payload_config_sha256"] != result["payload_config_sha256"]
    config.estop_authority = "tablet"
    del estop.endpoints[:]
    for kind in (robot_state_pb2.EStopState.TYPE_HARDWARE,
                 robot_state_pb2.EStopState.TYPE_SOFTWARE):
        state.estop_states.add(type=kind, state=robot_state_pb2.EStopState.STATE_NOT_ESTOPPED)
    assert reader.snapshot()["estop_ready"] and reader.health()["estop_ready"]
    state.estop_states[1].state = robot_state_pb2.EStopState.STATE_ESTOPPED
    assert not reader.snapshot()["estop_ready"] and not reader.health()["estop_ready"]


def test_read_only_shadow_never_imports_control(
    monkeypatch, tmp_path, fixture_path, manifest, envelope, state
):
    import builtins
    import time
    from spot_deploy.policy import OnnxPolicy
    from spot_deploy.records import RunRecord
    from spot_deploy.runtime import shadow

    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name.endswith("sdk_control") or "robot_command" in name or "lease" in name:
            raise AssertionError("shadow attempted control import")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)

    def get():
        now = time.monotonic()
        return state.model_copy(update={"received_monotonic_s": now, "robot_time_s": now + 1000})

    reader = SimpleNamespace(
        start_stream=lambda d: get(),
        mailbox=SimpleNamespace(get=get),
        robot_now=lambda: time.monotonic() + 1000,
        stream_statistics=lambda: {
            "received_count": 3,
            "acquisition_gap_s": None,
            "receive_gap_s": None,
        },
    )
    record = RunRecord(tmp_path / "shadow", "test")
    result = shadow(
        reader, OnnxPolicy(fixture_path / "fixture.onnx", manifest), envelope, 0.06, record
    )
    record.finish("passed", result)
    assert result["samples"] >= 2
    assert result["motion_commands"] == 0
    assert result["command_network_latency_s"] is None
