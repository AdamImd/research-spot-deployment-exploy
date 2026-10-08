import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from spot_deploy.contracts import ContractError
from spot_deploy.control_loop import stand


class Log:
    def __init__(self):
        self.events = []

    def event(self, kind, **kwargs):
        self.events.append((kind, kwargs))


class FakeReader:
    def __init__(self, state):
        self.state = state
        self.key = 0
        self.bad_health = False
        self.disconnected = False
        self.config = SimpleNamespace(rpc_timeout_s=0.3)
        self.mailbox = self

    def robot_now(self):
        return time.monotonic() + 1000

    def get(self):
        if self.disconnected:
            raise ContractError("test disconnect")
        now = time.monotonic()
        return self.state.model_copy(
            update={
                "robot_time_s": now + 1000,
                "received_monotonic_s": now,
                "last_command_key": self.key,
            }
        )

    def start_stream(self, duration):
        return self.get()

    def health(self):
        return {
            "time": time.monotonic(),
            "estop_ready": not self.bad_health,
            "fault_count": 0,
            "battery_percent": 100,
        }


class FakeControl:
    def __init__(self, reader, mode=None):
        self.reader, self.mode = reader, mode
        self.calls = []
        self.commands = []
        self.stop = threading.Event()
        self.activated = False
        self.worker = None

    def acquire(self):
        self.calls.append("acquire")

    def native_stand(self, cancelled):
        self.calls.append("native_stand")

    def heartbeat(self):
        if self.mode == "lease" and self.activated:
            raise ContractError("test lease lost")

    def start(self, iterator, duration):
        self.calls.append("start")

        def consume():
            for command in iterator:
                if self.stop.is_set():
                    break
                self.commands.append(command)
                if self.mode != "ack":
                    self.reader.key = command.key
                if self.mode == "estop":
                    self.reader.bad_health = True
                if self.mode == "disconnect" and len(self.commands) >= 3:
                    self.reader.disconnected = True

        self.worker = threading.Thread(target=consume, daemon=True)
        self.worker.start()

    def activate(self):
        self.calls.append("activate")
        self.activated = True

    def active(self):
        return self.activated

    def check_stream(self):
        pass

    def close(self):
        self.calls.append("close")
        self.stop.set()
        if self.worker:
            self.worker.join(0.5)


class FakePolicy:
    def __init__(self, manifest, mode=None):
        self.manifest, self.mode = manifest, mode
        self.count = 0

    def reset(self):
        self.count = 0

    def predict(self, state):
        self.count += 1
        if self.mode == "nan":
            return np.full(19, np.nan), 0.001, np.zeros((1, 48)), np.zeros(12)
        if self.mode == "stall" and self.count > 2:
            time.sleep(0.4)
        return np.zeros(19), 0.001, np.zeros((1, 48)), np.zeros(12)


def test_standing_independent_schedules_and_cleanup(manifest, envelope, state):
    reader = FakeReader(state)
    control = FakeControl(reader)
    policy = FakePolicy(manifest)
    log = Log()
    result = stand(reader, control, policy, envelope, 0.12, log, lambda c, *a: c)
    assert control.calls == ["acquire", "native_stand", "start", "activate", "close"]
    assert result["commands"] > policy.count * 2
    assert all(len(c.positions) == 19 for c in control.commands)
    assert all(c.end_robot_time_s > 1000 for c in control.commands)
    assert not control.worker.is_alive()


def test_measured_hold_precedes_active_feedback_then_blends(manifest, envelope, state):
    state.positions = [0.05 + i * 0.001 for i in range(12)] + [0] * 7
    reader = FakeReader(state)

    class DelayedActivation(FakeControl):
        polls = 0
        acknowledged = False

        def active(self):
            self.polls += 1
            self.acknowledged = self.activated and self.polls >= 3
            return self.acknowledged

    control = DelayedActivation(reader)
    policy = FakePolicy(manifest)
    samples = []

    def encode(command, *args):
        samples.append((control.acknowledged, np.array(command.positions)))
        return command

    envelope.transition_s = 0.04
    stand(reader, control, policy, envelope, 0.12, Log(), encode)
    held = [target for active, target in samples if not active]
    assert held
    for target in held:
        np.testing.assert_allclose(target, state.positions)
    assert any(0 < target[0] < state.positions[0] for active, target in samples if active)
    np.testing.assert_array_equal(samples[-1][1], [0] * 19)
    assert control.calls == ["acquire", "native_stand", "start", "activate", "close"]


@pytest.mark.parametrize("mode", ["nan", "stall", "lease", "estop", "disconnect", "ack"])
def test_fault_injection_stops_control(manifest, envelope, state, mode):
    reader = FakeReader(state)
    control = FakeControl(reader, mode)
    policy = FakePolicy(manifest, mode)
    began = time.monotonic()
    with pytest.raises(ContractError):
        stand(reader, control, policy, envelope, 0.6, Log(), lambda c, *a: c)
    assert time.monotonic() - began < 1.5
    assert control.calls[-1] == "close"
    assert not control.worker or not control.worker.is_alive()
    count = len(control.commands)
    time.sleep(0.01)
    assert len(control.commands) == count


def test_power_on_failure_still_closes(manifest, envelope, state):
    reader = FakeReader(state)
    control = FakeControl(reader)

    def bad_stand(cancelled):
        raise ContractError("power response lost")

    control.native_stand = bad_stand
    with pytest.raises(ContractError):
        stand(reader, control, FakePolicy(manifest), envelope, 0.1, Log(), lambda c, *a: c)
    assert control.calls[-1] == "close"


@pytest.mark.parametrize("fail_at", [1, 5])
def test_hardware_interlock_blocks_acquire_or_stops_commands(manifest, envelope, state, fail_at):
    reader = FakeReader(state)
    control = FakeControl(reader)

    class Interlock:
        calls = 0

        def check(self, health=None):
            self.calls += 1
            if self.calls >= fail_at:
                raise ContractError("hardware E-stop unavailable")

    with pytest.raises(ContractError, match="hardware E-stop"):
        stand(
            reader,
            control,
            FakePolicy(manifest),
            envelope,
            0.5,
            Log(),
            lambda c, *a: c,
            interlock=Interlock(),
        )
    assert control.calls[-1] == "close"
    if fail_at == 1:
        assert control.calls == ["close"]
    else:
        assert not control.worker.is_alive()
        assert len(control.commands) < 10
