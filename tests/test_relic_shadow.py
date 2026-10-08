import builtins
import time
from types import SimpleNamespace

import numpy as np
import pytest

from spot_deploy.contracts import ContractError, JOINTS
from spot_deploy.relic_policy import (
    ACTION_IDS, ACTION_JOINTS, DEFAULT_Q, OBS_IDS, ROOT_COM_B, joint_targets, observation,
)
from spot_deploy.relic_shadow import run
from spot_deploy.viewer import RunFeed


def test_relic_observation_order_frames_and_previous_raw_action(state):
    state.positions = (DEFAULT_Q + np.arange(19, dtype=np.float32) / 10).tolist()
    state.velocities = list(range(19))
    state.odom_quaternion_wxyz = [2 ** -.5, 0, 0, 2 ** -.5]
    state.linear_velocity_odom = [0, 2, 0]
    state.angular_velocity_odom = [-3, 0, 0]
    arm = np.arange(7, dtype=np.float32)
    previous = np.arange(12, dtype=np.float32) + 10
    obs = observation(state, arm, previous)
    np.testing.assert_allclose(obs[:3], [2 + 3 * ROOT_COM_B[2], 0, 0], atol=1e-6)
    np.testing.assert_allclose(obs[3:9], [0, 3, 0, 0, 0, -1], atol=1e-6)
    np.testing.assert_array_equal(obs[9:12], 0)
    np.testing.assert_array_equal(obs[12:19], arm)
    np.testing.assert_array_equal(obs[19:31], 0)
    np.testing.assert_allclose(obs[31:34], [0, 0, .55])
    np.testing.assert_allclose(obs[34:53], np.arange(19)[OBS_IDS] / 10, atol=1e-6)
    np.testing.assert_array_equal(obs[53:72], np.arange(19)[OBS_IDS])
    np.testing.assert_array_equal(obs[72:], previous)
    assert obs.dtype == np.float32


def test_relic_targets_permute_by_name_without_clipping():
    action = np.arange(12, dtype=np.float32) - 6
    arm = np.arange(7, dtype=np.float32)
    target = joint_targets(action, arm)
    for idx, name in enumerate(ACTION_JOINTS):
        assert target[JOINTS.index(name)] == pytest.approx(DEFAULT_Q[JOINTS.index(name)] + .2 * action[idx])
    np.testing.assert_array_equal(target[12:], arm)
    assert len(set(ACTION_IDS)) == 12
    with pytest.raises(ContractError):
        joint_targets([float("nan")] * 12, arm)


def test_requested_height_changes_only_the_height_observation(state):
    arm, previous = np.zeros(7), np.zeros(12)
    nominal = observation(state, arm, previous)
    initial = observation(state, arm, previous, body_height=.473)
    assert initial[33] == pytest.approx(.473)
    np.testing.assert_array_equal(initial[:33], nominal[:33])
    np.testing.assert_array_equal(initial[34:], nominal[34:])
    np.testing.assert_array_equal(initial[72:], 0)


@pytest.mark.parametrize("stale", [False, True])
def test_shadow_only_reads_and_logs(monkeypatch, tmp_path, state, stale):
    original = builtins.__import__

    def blocked(name, *args, **kwargs):
        if "sdk_control" in name or "robot_command" in name or "control_loop" in name:
            raise AssertionError("shadow imported robot control")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked)
    events = []

    def sample():
        now = time.monotonic()
        return state.model_copy(update={"robot_time_s": now - (1 if stale else 0),
                                        "received_monotonic_s": now})

    reader = SimpleNamespace(start_stream=lambda duration: sample(),
        mailbox=SimpleNamespace(get=sample), robot_now=time.monotonic,
        health=lambda: {"estop_ready": False, "fault_count": 0, "battery_percent": 100},
        config=SimpleNamespace(rpc_timeout_s=.1), received_count=20,
        stream_statistics=lambda: {"received_count": 20})
    policy = SimpleNamespace(predict=lambda s: (DEFAULT_Q, .001, np.zeros(84), np.zeros(12)))
    record = SimpleNamespace(directory=tmp_path,
                             event=lambda kind, **fields: events.append((kind, fields)))
    if stale:
        with pytest.raises(ContractError, match="older"):
            run(reader, policy, .045, record)
    else:
        result = run(reader, policy, .045, record)
        assert result["samples"] >= 2 and result["motion_commands"] == 0
        assert set(kind for kind, _ in events) == {"health", "shadow_sample"}
        assert not result["control_lease_acquired"]


def test_relic_viewer_metadata_is_filtered(tmp_path):
    import json

    (tmp_path / "run.json").write_text('{"mode":"relic-shadow"}')
    (tmp_path / "policy.json").write_text(json.dumps({"name": "ReLIC", "arbitrary": "omit"}))
    (tmp_path / "shadow-status.json").write_text(json.dumps({"state_hz": 333, "policy_hz": 50,
                                                           "arbitrary": "omit"}))
    result = RunFeed(tmp_path).response()
    assert result["policy"]["name"] == "ReLIC"
    assert result["rates"]["policy_hz"] == 50
    assert "arbitrary" not in result["policy"] and "arbitrary" not in result["rates"]


@pytest.fixture
def shadow_clock(monkeypatch, tmp_path, state):
    """Advance the consumer clock independently of delivered robot samples."""
    clock = SimpleNamespace(now=100.0)

    def sleep(seconds):
        assert seconds > 0
        clock.now += seconds

    monkeypatch.setattr("spot_deploy.relic_shadow.time", SimpleNamespace(
        monotonic=lambda: clock.now, time=lambda: 1000 + clock.now, sleep=sleep))
    first = state.model_copy(update={"robot_time_s": clock.now,
                                     "received_monotonic_s": clock.now})
    events, predictions = [], []

    def predict(sample):
        predictions.append(sample.robot_time_s)
        return DEFAULT_Q, .001, np.zeros(84), np.zeros(12)

    reader = SimpleNamespace(start_stream=lambda duration: first,
        mailbox=SimpleNamespace(get=lambda: first), robot_now=lambda: clock.now,
        health=lambda: {"estop_ready": False}, config=SimpleNamespace(rpc_timeout_s=.1),
        received_count=20, stream_statistics=lambda: {"received_count": 20})
    record = SimpleNamespace(directory=tmp_path,
                             event=lambda kind, **fields: events.append((kind, fields)))
    return SimpleNamespace(clock=clock, first=first, reader=reader,
                           policy=SimpleNamespace(predict=predict), record=record,
                           events=events, predictions=predictions)


def test_shadow_waits_for_fresh_delivery_without_repredicting(shadow_clock):
    h = shadow_clock
    delivery = iter([h.first, h.first, h.first.model_copy(update={
        "robot_time_s": 100.04, "received_monotonic_s": 100.04})])
    h.reader.mailbox.get = lambda: next(delivery)
    result = run(h.reader, h.policy, .055, h.record)
    assert h.predictions == [100.0, 100.04]
    assert result["samples"] == 2
    assert result["skipped_state_ticks"] == 1
    assert result["motion_commands"] == 0
    wait = [fields for kind, fields in h.events if kind == "shadow_wait"]
    assert len(wait) == 1
    assert wait[0]["state_age_s"] == pytest.approx(.02)
    assert result["policy_loop_s"]["min"] == pytest.approx(.04)


@pytest.mark.parametrize("stale_field", ["robot_time_s", "received_monotonic_s"])
def test_shadow_delivery_wait_preserves_both_freshness_limits(shadow_clock, stale_field):
    h = shadow_clock

    def sample():
        times = dict(robot_time_s=h.clock.now, received_monotonic_s=h.clock.now)
        times[stale_field] = h.first.robot_time_s
        return h.first.model_copy(update=times)

    h.reader.mailbox.get = sample
    with pytest.raises(ContractError, match="older than 100 ms"):
        run(h.reader, h.policy, .5, h.record)
    assert .1 < h.clock.now - 100 <= .12 + 1e-9
    if stale_field == "robot_time_s":
        assert h.predictions == [100.0]


def test_shadow_still_rejects_timestamp_regression(shadow_clock):
    h = shadow_clock
    delivery = iter([h.first, h.first.model_copy(update={"robot_time_s": 99.999})])
    h.reader.mailbox.get = lambda: next(delivery)
    with pytest.raises(ContractError, match="moved backwards"):
        run(h.reader, h.policy, .055, h.record)
    assert h.predictions == [100.0]
