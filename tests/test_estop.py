import json
import socket
import threading
import time
import zlib
from pathlib import Path
from types import SimpleNamespace

import pytest
from bosdyn.api import estop_pb2

from spot_deploy.contracts import ContractError, RobotConfig, load, sha256
from spot_deploy.estop_bridge import StatusWriter, run_bridge
from spot_deploy.estop_interlock import HardwareInterlock, boot_id, robot_binding
from spot_deploy.estop_protocol import EstopProfile, PhysicalLatch, parse, request
from spot_deploy.sdk_estop import SpotEstop, assert_slot, endpoint_for

BASE = Path(__file__).resolve().parents[1]
NONCE = "0123456789abcdef"
DEVICE = "aabbccddeeff"


@pytest.fixture
def profile():
    return load(BASE / "examples/esp32-estop.json", EstopProfile).model_copy(
        update={"device_id": DEVICE}
    )


def frame(nonce=NONCE, device=DEVICE, boot="abcdef01", seq=1, uptime=100, stopped=True):
    body = f"E1,{device},{boot},{seq},{uptime},{nonce},{int(stopped)},{int(not stopped)}".encode()
    return body + f",{zlib.crc32(body):08x}\n".encode()


def test_challenge_and_crc():
    assert request(NONCE) == b"Q,0123456789abcdef\n"
    assert parse(frame(), NONCE, DEVICE).stopped
    with pytest.raises(ContractError):
        request("short")


@pytest.mark.parametrize(
    "value",
    [
        b"",
        b"x" * 161 + b"\n",
        frame()[:-1],
        frame()[:-10] + b"0,00000000\n",
        frame(nonce="0" * 16),
        frame(device="0" * 12),
        frame(boot="ABCDEF00"),
        frame(seq=-1),
        frame(seq=2**32),
        frame(uptime=2**32),
    ],
)
def test_malformed_or_stale_frame_rejected(value):
    with pytest.raises(ContractError):
        parse(value, NONCE, DEVICE)


def test_startup_requires_stop_then_physical_arm():
    latch = PhysicalLatch()
    assert not latch.accept(parse(frame(seq=1, stopped=False), NONCE, DEVICE))
    assert not latch.accept(parse(frame(seq=2), NONCE, DEVICE))
    assert latch.accept(parse(frame(seq=3, stopped=False), NONCE, DEVICE))
    assert not latch.accept(parse(frame(seq=4), NONCE, DEVICE))


@pytest.mark.parametrize(
    "replacement", [frame(), frame(seq=2, boot="12345678"), frame(seq=2, uptime=99)]
)
def test_reboot_or_replay_fault(replacement):
    latch = PhysicalLatch()
    latch.accept(parse(frame(), NONCE, DEVICE))
    with pytest.raises(ContractError):
        latch.accept(parse(replacement, NONCE, DEVICE))


def test_profile_bounds(profile):
    for update in (
        {"endpoint_role": "PDB_rooted"},
        {"cut_power_timeout_s": 0.2},
        {"response_timeout_s": 0.08, "rpc_timeout_s": 0.1, "poll_interval_s": 0.08},
        {"serial_device": "/dev/ttyUSB0"},
    ):
        with pytest.raises(ValueError):
            EstopProfile.model_validate(profile.model_dump() | update)


class Endpoint:
    endpoint_id = "reserved-hardware-endpoint"

    def __init__(self):
        self.levels = []

    def check_in(self, allowed):
        self.levels.append(allowed)

    def stop(self):
        self.levels.append(False)


class Record:
    def event(self, *args, **kwargs):
        pass


def test_bridge_never_allows_after_invalid_packet(profile, tmp_path):
    class Link:
        count = 0

        def exchange(self, nonce):
            self.count += 1
            if self.count == 3:
                return b"corrupt\n"
            return frame(nonce=nonce, seq=self.count, stopped=self.count == 1)

    endpoint = Endpoint()
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64, "2" * 64)
    try:
        with pytest.raises(ContractError):
            run_bridge(profile, Link(), status, Record(), endpoint)
        assert endpoint.levels == [False, True, False]
        assert json.loads(status.path.read_text())["state"] != "armed"
    finally:
        status.close()


def test_primary_fault_survives_failed_stop_rpc(profile, tmp_path):
    class FailedEndpoint(Endpoint):
        def stop(self):
            raise RuntimeError("SDK stop also failed")

    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64)
    try:
        with pytest.raises(ContractError, match="invalid E-stop"):
            run_bridge(
                profile,
                SimpleNamespace(exchange=lambda _: b"corrupt\n"),
                status,
                Record(),
                FailedEndpoint(),
            )
        saved = json.loads(status.path.read_text())
        assert saved["state"] == "fault" and not saved["robot_confirmed"]
    finally:
        status.close()


def test_bridge_delayed_safe_frame_never_allows(profile, tmp_path):
    class Link:
        def exchange(self, nonce):
            time.sleep(profile.response_timeout_s + 0.01)
            return frame(nonce=nonce, stopped=False)

    endpoint = Endpoint()
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64)
    try:
        with pytest.raises(ContractError, match="deadline"):
            run_bridge(profile, Link(), status, Record(), endpoint)
        assert endpoint.levels == [False]
    finally:
        status.close()


def test_status_single_owner(profile, tmp_path):
    path = tmp_path / "status.json"
    status = StatusWriter(path, profile, "1" * 64)
    try:
        with pytest.raises(ContractError, match="owner"):
            StatusWriter(path, profile, "1" * 64)
    finally:
        status.close()


def hardware_config(tmp_path):
    return RobotConfig(
        schema_version=1,
        endpoint="example.invalid",
        expected_serial="fixture",
        expected_firmware="5.1.1",
        expected_host=socket.gethostname(),
        wired_interface="eth0",
        rpc_timeout_s=0.2,
        joint_control_feature="joint_control",
        hardware_estop={
            "profile_sha256": "1" * 64,
            "status_file": str(tmp_path / "status.json"),
            "max_status_age_s": 0.3,
        },
    )


def armed_status(config):
    return dict(
        schema_version=1,
        host=socket.gethostname(),
        host_boot_id=boot_id(),
        mode="robot",
        session="one",
        device_id=DEVICE,
        profile_sha256="1" * 64,
        robot_binding_sha256=robot_binding(config),
        state="armed",
        robot_confirmed=True,
        updated_monotonic_s=time.monotonic(),
        endpoint_id="endpoint",
    )


def test_interlock_binds_session_and_robot_status(tmp_path):
    config = hardware_config(tmp_path)
    path = Path(config.hardware_estop.status_file)
    data = armed_status(config)
    path.write_text(json.dumps(data))
    gate = HardwareInterlock(config)
    gate.check({"estop_endpoints": [{"id": "endpoint", "allowed": True}]})
    with pytest.raises(ContractError):
        gate.check({"estop_endpoints": [{"id": "different", "allowed": True}]})
    path.write_text(json.dumps(data | {"session": "restart"}))
    with pytest.raises(ContractError):
        gate.check()


@pytest.mark.parametrize(
    "change",
    [
        {"updated_monotonic_s": 0},
        {"updated_monotonic_s": float("nan")},
        {"updated_monotonic_s": 1e30},
        {"mode": "bench"},
        {"state": "stopped"},
        {"robot_confirmed": False},
        {"host_boot_id": "oldboot"},
        {"profile_sha256": "0" * 64},
        {"robot_binding_sha256": "0" * 64},
    ],
)
def test_interlock_fails_closed(tmp_path, change):
    config = hardware_config(tmp_path)
    path = Path(config.hardware_estop.status_file)
    path.write_text(json.dumps(armed_status(config) | change))
    with pytest.raises(ContractError):
        HardwareInterlock(config).check()


def config_proto(profile):
    config = estop_pb2.EstopConfig(unique_id="config")
    independent = config.endpoints.add(role="PDB_rooted", name="tablet", unique_id="independent")
    independent.timeout.seconds = 1
    config.endpoints.add().CopyFrom(endpoint_for(None, profile).to_proto())
    config.endpoints[-1].unique_id = "slot"
    return config


def test_actual_sdk_slot_contract(profile):
    config = config_proto(profile)
    assert assert_slot(config, profile, "config", "slot").role == profile.endpoint_role
    with pytest.raises(ContractError):
        assert_slot(config, profile, "changed", "slot")
    with pytest.raises(ContractError):
        assert_slot(config, profile, "config", "other")
    config.endpoints[-1].cut_power_timeout.seconds = 2
    with pytest.raises(ContractError):
        assert_slot(config, profile, "config", "slot")


def make_adapter(client, profile):
    adapter = SpotEstop.__new__(SpotEstop)
    adapter.client, adapter.profile = client, profile
    adapter.endpoint, adapter.config_id = None, None
    adapter.motors_off = lambda: None
    return adapter


def test_additive_configuration_preserves_independent_endpoints(profile):
    current = config_proto(profile)
    del current.endpoints[-1]
    original = current.endpoints[0].SerializeToString()

    class Client:
        def get_config(self, **kwargs):
            return current

        def set_config(self, proposed, expected, **kwargs):
            assert expected == "config" and kwargs["timeout"] == profile.rpc_timeout_s
            assert len(proposed.endpoints) == 2
            assert proposed.endpoints[0].SerializeToString() == original
            proposed.unique_id = "new-config"
            proposed.endpoints[-1].unique_id = "new-slot"
            return proposed

    assert make_adapter(Client(), profile).configure("config")["endpoint_id"] == "new-slot"


def test_registration_refuses_active_owner(profile):
    config = config_proto(profile)
    status = estop_pb2.EstopSystemStatus()
    status.endpoints.add(endpoint=config.endpoints[-1], stop_level=estop_pb2.ESTOP_LEVEL_NONE)
    client = SimpleNamespace(get_config=lambda **_: config, get_status=lambda **_: status)
    with pytest.raises(ContractError, match="active owner"):
        make_adapter(client, profile).register("config", "slot")


@pytest.mark.parametrize("expired_registration", [False, True])
def test_registration_and_manual_checkins_are_bounded_and_initially_stopped(
    profile, expired_registration
):
    config = config_proto(profile)
    status = estop_pb2.EstopSystemStatus()
    if expired_registration:
        old = status.endpoints.add(
            endpoint=config.endpoints[-1], stop_level=estop_pb2.ESTOP_LEVEL_NONE
        )
        old.time_since_valid_response.seconds = 2

    class Client:
        levels = []
        _stub = SimpleNamespace(RegisterEstopEndpoint=object())

        def get_config(self, **kwargs):
            return config

        def get_status(self, **kwargs):
            return status

        def call(self, rpc, request, **kwargs):
            assert kwargs["timeout"] == profile.rpc_timeout_s
            assert request.target_config_id == "config"
            assert request.target_endpoint.unique_id == "slot"
            assert request.target_endpoint.role == profile.endpoint_role
            response = estop_pb2.RegisterEstopEndpointResponse(
                status=estop_pb2.RegisterEstopEndpointResponse.STATUS_SUCCESS,
                new_endpoint=request.new_endpoint,
            )
            response.new_endpoint.unique_id = "registered"
            return kwargs["value_from_response"](response)

        def check_in(self, level, endpoint, challenge, response, **kwargs):
            assert kwargs["timeout"] == profile.rpc_timeout_s
            self.levels.append(level)
            return 123

    client = Client()
    adapter = make_adapter(client, profile)
    adapter.register("config", "slot")
    assert adapter.endpoint_id == "registered"
    assert client.levels == [estop_pb2.ESTOP_LEVEL_CUT]
    adapter.check_in(True)
    adapter.stop()
    assert client.levels == [
        estop_pb2.ESTOP_LEVEL_CUT,
        estop_pb2.ESTOP_LEVEL_NONE,
        estop_pb2.ESTOP_LEVEL_CUT,
    ]


@pytest.mark.parametrize("motors_on,wrong_robot", [(True, False), (False, True)])
def test_registration_rejects_powered_or_wrong_robot(profile, tmp_path, motors_on, wrong_robot):
    from bosdyn.api import robot_id_pb2, robot_state_pb2

    identity = robot_id_pb2.RobotId(serial_number="other" if wrong_robot else "fixture")
    identity.software_release.version.major_version = 5
    identity.software_release.version.minor_version = 1
    identity.software_release.version.patch_level = 1
    state = robot_state_pb2.RobotState()
    state.power_state.motor_power_state = (
        robot_state_pb2.PowerState.STATE_ON if motors_on else robot_state_pb2.PowerState.STATE_OFF
    )
    adapter = SpotEstop.__new__(SpotEstop)
    adapter.profile = profile
    adapter.reader = SimpleNamespace(
        config=hardware_config(tmp_path),
        robot=SimpleNamespace(ensure_client=lambda _: SimpleNamespace(get_id=lambda **_: identity)),
        state_client=SimpleNamespace(get_robot_state=lambda **_: state),
    )
    with pytest.raises(ContractError):
        adapter.motors_off()


def test_bench_cli_cannot_import_spot_sdk(profile, tmp_path, monkeypatch):
    import builtins
    from spot_deploy.estop_cli import main

    original = builtins.__import__

    def imports(name, *args, **kwargs):
        if name.startswith("bosdyn") or name.endswith("sdk_estop") or name.endswith("sdk_read"):
            pytest.fail("bench imported a hardware SDK adapter")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", imports)
    monkeypatch.setattr(
        "spot_deploy.estop_cli.SerialLink",
        lambda _: SimpleNamespace(exchange=lambda nonce: frame(nonce=nonce), close=lambda: None),
    )
    path = tmp_path / "profile.json"
    path.write_text(profile.model_dump_json())
    status = tmp_path / "status.json"
    assert (
        main(
            [
                "bench",
                "--profile",
                str(path),
                "--status",
                str(status),
                "--duration",
                ".01",
                "--output",
                str(tmp_path / "run"),
            ]
        )
        == 0
    )
    saved = json.loads(status.read_text())
    assert saved["mode"] == "bench" and not saved["robot_confirmed"]


def test_live_estop_requires_execute_before_network(tmp_path, monkeypatch):
    from spot_deploy.estop_cli import main

    config = hardware_config(tmp_path)
    path = tmp_path / "robot.json"
    path.write_text(config.model_dump_json())
    monkeypatch.setattr("spot_deploy.network.wired_route", lambda _: pytest.fail("network touched"))
    assert (
        main(
            [
                "configure",
                "--profile",
                str(BASE / "examples/esp32-estop.json"),
                "--robot",
                str(path),
                "--expected-config-id",
                "config",
                "--output",
                str(tmp_path / "run"),
            ]
        )
        == 1
    )


def test_hardware_readiness_gate_is_required_when_configured(tmp_path):
    from spot_deploy.readiness import report

    path = tmp_path / "robot.json"
    path.write_text(hardware_config(tmp_path).model_dump_json())
    result = report(robot_path=path)
    assert any(
        c["gate"] == "hardware_estop" and c["status"] == "unverified" for c in result["checks"]
    )
    assert not result["evidence_ready"]


def test_real_serial_pty_disconnect_is_stopped(profile, tmp_path, monkeypatch):
    import os
    import pty
    from spot_deploy.estop_bridge import SerialLink

    master, slave = pty.openpty()
    serial_profile = profile.model_copy(update={"serial_device": os.ttyname(slave)})
    # Skip only the real board reset delay; real pyserial read/write deadlines are exercised.
    original_sleep = time.sleep
    monkeypatch.setattr(
        "spot_deploy.estop_bridge.time.sleep", lambda s: original_sleep(s) if s < 1 else None
    )
    link = SerialLink(serial_profile)
    endpoint = Endpoint()
    status = StatusWriter(
        tmp_path / "status.json", profile, sha256(BASE / "examples/esp32-estop.json")
    )

    def device():
        buffer = b""
        for sequence in range(1, 4):
            while b"\n" not in buffer:
                buffer += os.read(master, 128)
            query, buffer = buffer.split(b"\n", 1)
            nonce = query.decode().split(",")[1]
            os.write(
                master,
                frame(nonce=nonce, seq=sequence, stopped=sequence == 1)
                if sequence < 3
                else b"corrupt\n",
            )

    thread = threading.Thread(target=device, daemon=True)
    thread.start()
    try:
        with pytest.raises(ContractError):
            run_bridge(profile, link, status, Record(), endpoint)
        assert endpoint.levels == [False, True, False]
        thread.join(1)
        assert not thread.is_alive()
    finally:
        link.close()
        status.close()
        os.close(master)
        os.close(slave)
