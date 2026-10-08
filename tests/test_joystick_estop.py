import json
import select
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from bosdyn.api import estop_pb2

from spot_deploy.contracts import ContractError, load, sha256
from spot_deploy.estop_bridge import StatusWriter
from spot_deploy.joystick_estop import (
    AXIS, BUTTON, EVENT, INIT, ButtonEvent, JoystickLatch, JoystickLink, JoystickProfile,
    run_joystick_bridge,
)
from spot_deploy.sdk_estop import SpotEstop, assert_slot, endpoint_for

BASE = Path(__file__).resolve().parents[1]


@pytest.fixture
def profile():
    return load(BASE / "examples/joystick-estop.json", JoystickProfile).model_copy(
        update={"stop_button": 0, "rearm_button": 1})


def initialize(latch, stop=False, rearm=False):
    return latch.accept([ButtonEvent(0, stop, True), ButtonEvent(1, rearm, True)], 0)


def test_no_auto_arm_and_stop_release_does_not_rearm(profile):
    latch = JoystickLatch(profile)
    assert not initialize(latch)
    assert not latch.accept([ButtonEvent(1, True)], 1)  # No observed STOP yet.
    assert not latch.accept([ButtonEvent(1, False), ButtonEvent(0, True)], 2)
    assert not latch.accept([ButtonEvent(0, False)], 3)
    assert not latch.accept([], 4)
    assert latch.accept([ButtonEvent(1, True)], 4)
    assert not latch.accept([ButtonEvent(0, True)], 5)
    assert not latch.accept([ButtonEvent(0, False)], 6)
    assert not latch.accept([], 10)  # Still holding rearm through the stop.
    assert not latch.accept([ButtonEvent(1, False)], 11)
    assert latch.accept([ButtonEvent(1, True)], 12)


def test_held_buttons_at_start_cannot_arm(profile):
    latch = JoystickLatch(profile)
    assert not initialize(latch, stop=True, rearm=True)
    assert not latch.accept([ButtonEvent(0, False)], 1)
    assert not latch.accept([], 2)
    assert not latch.accept([ButtonEvent(1, False)], 3)
    assert latch.accept([ButtonEvent(1, True)], 4)


def test_rearm_press_before_release_settles_is_not_deferred(profile):
    latch = JoystickLatch(profile)
    initialize(latch, stop=True)
    assert not latch.accept([ButtonEvent(0, False)], 1)
    assert not latch.accept([ButtonEvent(1, True)], 1.1)
    assert not latch.accept([], 2)
    assert not latch.accept([ButtonEvent(1, False)], 2.1)
    assert latch.accept([ButtonEvent(1, True)], 2.2)


@pytest.mark.parametrize("events", [
    [ButtonEvent(0, True), ButtonEvent(0, False), ButtonEvent(1, True)],
    [ButtonEvent(1, True), ButtonEvent(0, True), ButtonEvent(0, False)],
])
def test_stop_press_wins_even_when_release_and_rearm_share_one_poll(profile, events):
    latch = JoystickLatch(profile)
    initialize(latch, stop=True)
    latch.accept([ButtonEvent(0, False)], 1)
    assert not latch.accept(events, 2)
    assert not latch.accept([], 3)


@pytest.mark.parametrize("update", [
    {"stop_button": 1}, {"rearm_button": 0}, {"joystick_device": "/dev/input/js0"},
    {"joystick_device": "/dev/input/by-id/usb-Logitech-event-joystick"},
    {"release_hold_s": .1}, {"max_input_gap_s": .1}, {"endpoint_role": "anything"},
    {"cut_power_timeout_s": .2},
])
def test_profile_rejects_ambiguous_identity_or_unsafe_timing(profile, update):
    with pytest.raises(ValueError):
        JoystickProfile.model_validate(profile.model_dump() | update)


def decoder():
    link = JoystickLink.__new__(JoystickLink)
    link.ready, link.buttons = False, {}
    link.initial_buttons, link.initial_axes = set(), set()
    link.axes = {}
    link.metadata = dict(button_count=2, axis_count=1)
    return link


def test_driver_decode_and_overflow_resynchronization_fault():
    link = decoder()
    got = link._decode(EVENT.pack(0, 0, INIT | BUTTON, 0)
                       + EVENT.pack(0, 0, INIT | AXIS, 0))
    assert got == [ButtonEvent(0, False, True)]
    link.ready = True
    with pytest.raises(ContractError, match="resynchronization"):
        link._decode(EVENT.pack(1, 0, INIT | BUTTON, 0))


@pytest.mark.parametrize("data", [b"", b"short", EVENT.pack(0, 2, BUTTON, 0),
                                    EVENT.pack(0, 1, BUTTON, 2), EVENT.pack(0, 1, 4, 0)])
def test_driver_rejects_malformed_input(data):
    with pytest.raises(ContractError):
        decoder()._decode(data)


def test_device_disconnect_and_replacement_fault_before_read():
    link = decoder()
    info = SimpleNamespace(st_dev=1, st_ino=2, st_rdev=3)
    link.path, link.identity = SimpleNamespace(stat=lambda: info), (1, 2, 3)
    link.poller = SimpleNamespace(poll=lambda _: [(12, select.POLLHUP)])
    with pytest.raises(ContractError, match="disconnected"):
        link.poll()
    info.st_ino = 99
    with pytest.raises(ContractError, match="replaced"):
        link.poll()


class Endpoint:
    endpoint_id = "joystick-endpoint"

    def __init__(self):
        self.levels = []

    def check_in(self, allowed):
        self.levels.append(allowed)

    def stop(self):
        self.levels.append(False)


class Record:
    def event(self, *args, **kwargs):
        pass


def test_usb_failure_stops_and_faults_instead_of_reconnecting(profile, tmp_path):
    calls = []

    def poll():
        calls.append(1)
        if len(calls) == 1:
            return [ButtonEvent(0, True, True), ButtonEvent(1, False, True)]
        raise OSError("device removed")

    endpoint = Endpoint()
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64, "2" * 64)
    try:
        with pytest.raises(OSError, match="removed"):
            run_joystick_bridge(profile, SimpleNamespace(metadata={}, poll=poll), status,
                                Record(), endpoint)
        assert endpoint.levels == [False, False]
        saved = json.loads(status.path.read_text())
        assert saved["state"] == "fault" and saved["input_type"] == "linux_joystick"
        assert saved["stop_button"] == 0
        assert len(calls) == 2
    finally:
        status.close()


def test_slow_input_never_sends_allow(profile, tmp_path, monkeypatch):
    import spot_deploy.joystick_estop as module
    clock = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])

    def poll():
        clock[0] += .2
        return [ButtonEvent(0, False, True), ButtonEvent(1, True)]

    endpoint = Endpoint()
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64, "2" * 64)
    try:
        with pytest.raises(ContractError, match="stale"):
            run_joystick_bridge(profile, SimpleNamespace(metadata={}, poll=poll), status,
                                Record(), endpoint)
        assert endpoint.levels == [False]
    finally:
        status.close()


def test_normal_stop_event_sends_stop_on_exit(profile, tmp_path):
    endpoint, stop = Endpoint(), threading.Event()
    stop.set()
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64)
    try:
        run_joystick_bridge(profile, SimpleNamespace(metadata={}), status, Record(), endpoint, stop)
        assert endpoint.levels == [False]
    finally:
        status.close()


def adapter(client, profile):
    obj = SpotEstop.__new__(SpotEstop)
    obj.client, obj.profile, obj.endpoint = client, profile, None
    obj.motors_off = lambda: None
    return obj


def test_joystick_can_create_primary_only_from_empty_configuration(profile):
    class Client:
        def get_config(self, **kwargs):
            return estop_pb2.EstopConfig()

        def set_config(self, proposed, expected, **kwargs):
            assert expected == "" and len(proposed.endpoints) == 1
            assert proposed.endpoints[0].role == "PDB_rooted"
            proposed.unique_id, proposed.endpoints[0].unique_id = "new", "slot"
            return proposed

    result = adapter(Client(), profile).configure("")
    assert result["endpoint_id"] == "slot"


def test_joystick_primary_never_replaces_an_existing_endpoint(profile):
    config = estop_pb2.EstopConfig(unique_id="old")
    config.endpoints.add(role="PDB_rooted", name="tablet", unique_id="tablet")
    client = SimpleNamespace(get_config=lambda **_: config)
    with pytest.raises(ContractError, match="already exists"):
        adapter(client, profile).configure("old")


def test_slot_rejects_server_changed_watchdogs(profile):
    profile = profile.model_copy(update={"endpoint_timeout_s": .4, "cut_power_timeout_s": .8})
    config = estop_pb2.EstopConfig(unique_id="new")
    config.endpoints.add().CopyFrom(endpoint_for(None, profile).to_proto())
    config.endpoints[0].timeout.seconds, config.endpoints[0].timeout.nanos = 1, 0
    config.endpoints[0].cut_power_timeout.seconds = 1
    config.endpoints[0].cut_power_timeout.nanos = 0
    with pytest.raises(ContractError, match="watchdogs"):
        assert_slot(config, profile, "new", "")


def test_secondary_joystick_preserves_independent_endpoint(profile):
    profile = profile.model_copy(update={"endpoint_role": "spot_joystick_rpm"})
    config = estop_pb2.EstopConfig(unique_id="old")
    config.endpoints.add(role="PDB_rooted", name="tablet", unique_id="tablet")
    independent = config.endpoints[0].SerializeToString()

    class Client:
        def get_config(self, **kwargs):
            return config

        def set_config(self, proposed, expected, **kwargs):
            assert proposed.endpoints[0].SerializeToString() == independent
            proposed.unique_id, proposed.endpoints[-1].unique_id = "new", "slot"
            return proposed

    assert adapter(Client(), profile).configure("old")["endpoint_id"] == "slot"
    slot = estop_pb2.EstopConfig(unique_id="new")
    slot.endpoints.add().CopyFrom(endpoint_for(None, profile).to_proto())
    with pytest.raises(ContractError, match="independent"):
        assert_slot(slot, profile, "new")


def test_joystick_bench_never_imports_sdk(profile, tmp_path, monkeypatch):
    import builtins
    from spot_deploy.estop_cli import main

    original = builtins.__import__

    def imports(name, *args, **kwargs):
        if name.startswith("bosdyn") or name.endswith(("sdk_read", "sdk_estop")):
            pytest.fail("bench accessed a Spot adapter")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", imports)
    monkeypatch.setattr("spot_deploy.estop_cli.JoystickLink", lambda _: SimpleNamespace(
        metadata={}, poll=lambda: [ButtonEvent(0, False, True), ButtonEvent(1, False, True)],
        close=lambda: None))
    assert main(["bench", "--input", "joystick", "--profile",
                 str(BASE / "examples/joystick-estop.json"), "--status", str(tmp_path / "status.json"),
                 "--duration", ".04", "--output", str(tmp_path / "run")]) == 0
    status = json.loads((tmp_path / "status.json").read_text())
    assert status["mode"] == "bench" and status["robot_confirmed"] is False


def test_connection_failure_is_unconfirmed_and_records_stage(tmp_path, monkeypatch):
    from spot_deploy.estop_cli import main

    profile_path = BASE / "examples/joystick-estop.json"
    status_path = tmp_path / "status.json"
    robot_path = tmp_path / "robot.json"
    robot_path.write_text(json.dumps(dict(
        schema_version=1, endpoint="example.invalid", expected_serial="fixture",
        expected_firmware="5.0.1", expected_host="rpm", wired_interface="eth0",
        rpc_timeout_s=1, joint_control_feature="joint_level_control",
        hardware_estop=dict(profile_sha256=sha256(profile_path),
                            status_file=str(status_path), max_status_age_s=.25),
    )))
    calls = []

    class Reader:
        connection_stage = "authentication"

        def __init__(self, config):
            pass

        def connect(self, *, streaming):
            assert streaming is False
            raise TimeoutError("fixture")

        def close(self):
            calls.append("reader_closed")

    monkeypatch.setattr("spot_deploy.sdk_read.ReadOnlySpot", Reader)
    monkeypatch.setattr("spot_deploy.sdk_estop.SpotEstop",
                        lambda *args: pytest.fail("failed connection reached E-stop service"))
    monkeypatch.setattr("spot_deploy.network.wired_route", lambda _: {"interface": "eth0"})
    monkeypatch.setattr("spot_deploy.estop_cli.JoystickLink", lambda _: SimpleNamespace(
        close=lambda: calls.append("input_closed")))
    output = tmp_path / "run"
    assert main(["bridge", "--input", "joystick", "--execute",
                 "--profile", str(profile_path), "--robot", str(robot_path),
                 "--status", str(status_path), "--output", str(output),
                 "--expected-config-id", "0", "--expected-endpoint-id", ""]) == 1
    status = json.loads(status_path.read_text())
    assert status["state"] == "fault" and not status["robot_confirmed"]
    assert status["endpoint_id"] is None and "authentication" in status["reason"]
    events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
    assert events[-1]["event"] == "estop_failure"
    assert events[-1]["phase"] == "authentication"
    assert calls == ["input_closed", "reader_closed"]


def test_button_two_cuts_and_button_four_rearms():
    profile = load(BASE / "examples/joystick-estop.json", JoystickProfile)
    assert (profile.stop_button, profile.rearm_button) == (1, 3)
    latch = JoystickLatch(profile)
    assert not latch.accept([ButtonEvent(i, False, True) for i in range(12)], 0)
    assert not latch.accept([ButtonEvent(3, True)], 1)
    assert not latch.accept([ButtonEvent(3, False), ButtonEvent(1, True)], 2)
    assert not latch.accept([ButtonEvent(1, False)], 3)
    assert not latch.accept([ButtonEvent(0, True), ButtonEvent(2, True)], 4)
    assert latch.accept([ButtonEvent(3, True)], 5)
    assert not latch.accept([ButtonEvent(1, True)], 6)
    assert not latch.accept([ButtonEvent(1, False)], 7)
    assert not latch.accept([], 8)
    assert not latch.accept([ButtonEvent(3, False)], 9)
    assert latch.accept([ButtonEvent(3, True)], 10)
    assert not latch.accept([ButtonEvent(3, False), ButtonEvent(3, True),
                             ButtonEvent(1, True)], 11)


def test_blocked_status_disk_does_not_block_joystick_checkins(profile, tmp_path, monkeypatch):
    import time
    import spot_deploy.estop_bridge as module

    entered, release, stop = threading.Event(), threading.Event(), threading.Event()
    original = module.atomic_json
    writes = []

    def blocked(path, value):
        writes.append(value)
        if len(writes) == 1:
            entered.set()
            assert release.wait(2)
        original(path, value)

    monkeypatch.setattr(module, "atomic_json", blocked)
    endpoint = Endpoint()
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64, "2" * 64)
    link = SimpleNamespace(metadata={}, poll=lambda: [ButtonEvent(0, True)])
    errors = []

    def run():
        try:
            run_joystick_bridge(profile, link, status, Record(), endpoint, stop)
        except BaseException as error:
            errors.append(error)

    thread = threading.Thread(target=run)
    thread.start()
    try:
        assert entered.wait(1)
        initial = len(endpoint.levels)
        end = time.monotonic() + 1
        while len(endpoint.levels) < initial + 5 and time.monotonic() < end:
            time.sleep(.01)
        assert len(endpoint.levels) >= initial + 5
        # Disk remains blocked, but input polling and acknowledged STOP continue.
        assert not errors
        stop.set()
        release.set()
        thread.join(2)
        assert not thread.is_alive() and not errors
        saved = json.loads(status.path.read_text())
        assert saved["state"] == "stopped" and saved["robot_confirmed"]
        assert writes[0]["updated_monotonic_s"] < saved["updated_monotonic_s"]
    finally:
        stop.set()
        release.set()
        thread.join(2)
        status.close()


def test_status_disk_failure_stops_bridge(profile, tmp_path, monkeypatch):
    import spot_deploy.estop_bridge as module

    def failed(*args):
        raise OSError("fixture disk failure")

    monkeypatch.setattr(module, "atomic_json", failed)
    endpoint = Endpoint()
    status = StatusWriter(tmp_path / "status.json", profile, "1" * 64, "2" * 64)
    try:
        with pytest.raises(ContractError, match="status writer failed"):
            run_joystick_bridge(profile,
                                SimpleNamespace(metadata={}, poll=lambda: [ButtonEvent(0, True)]),
                                status, Record(), endpoint, duration=1)
        assert endpoint.levels[-1] is False
    finally:
        status.close()
