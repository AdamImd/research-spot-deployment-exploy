import threading
import time

import numpy as np
import pytest

from pydantic import ValidationError

from spot_deploy.contracts import ContractError, Envelope
from spot_deploy.safety import Guard, LatestState, Schedule, bounded_join


def issue(guard, state, targets=None, now=100.0, robot_now=1000.0, policy_at=100.0, elapsed=0.001):
    return guard.command(
        np.zeros(19) if targets is None else targets, state, now, robot_now, policy_at, elapsed
    )


@pytest.mark.parametrize(
    "updates,kwargs,error",
    [
        ({}, {"now": 100.2}, "stale local"),
        ({}, {"robot_now": 1000.2}, "timestamp"),
        ({}, {"robot_now": 999.9}, "timestamp"),
        ({}, {"policy_at": 99.0}, "policy"),
        ({}, {"elapsed": 0.1}, "inference"),
        ({"positions": [3.0] * 19}, {}, "position"),
        ({"velocities": [3.0] * 19}, {}, "velocity"),
        ({"loads": [3.0] * 19}, {}, "load"),
        ({"positions": [0.0] * 12 + [0.1] * 7}, {}, "stowed"),
        ({"linear_velocity_odom": [1.0, 0.0, 0.0]}, {}, "speed"),
        ({"odom_quaternion_wxyz": [2**-0.5, 2**-0.5, 0.0, 0.0]}, {}, "attitude"),
    ],
)
def test_stops_bad_states(manifest, envelope, state, updates, kwargs, error):
    value = state.model_copy(update=updates)
    with pytest.raises(ContractError, match=error):
        issue(Guard(manifest, envelope), value, **kwargs)


@pytest.mark.parametrize(
    "target,error",
    [
        ([0.0] * 12, "nineteen"),
        ([float("nan")] * 19, "finite"),
        ([3.0] * 19, "position"),
        ([0.6] * 19, "tracking"),
        ([0] * 12 + [0.01] * 7, "arm movement"),
    ],
)
def test_stops_bad_targets(manifest, envelope, state, target, error):
    with pytest.raises(ContractError, match=error):
        issue(Guard(manifest, envelope), state, targets=target)


def test_command_expiry_and_key(manifest, envelope, state):
    command = issue(Guard(manifest, envelope), state)
    assert command.key == 1
    assert command.end_robot_time_s == pytest.approx(1000.08)


def test_command_gap_slew_tracking(manifest, envelope, state):
    guard = Guard(manifest, envelope)
    issue(guard, state)
    with pytest.raises(ContractError, match="slew"):
        issue(guard, state, targets=[0.1] * 12 + [0] * 7, now=100.01)
    guard = Guard(manifest, envelope)
    issue(guard, state)
    with pytest.raises(ContractError, match="scheduling"):
        issue(guard, state, now=100.06)


def test_predicted_load(manifest, envelope, state):
    manifest.gains.kp = [100] * 19
    with pytest.raises(ContractError, match="predicted"):
        issue(Guard(manifest, envelope), state, targets=[0.1] * 12 + [0] * 7)


def separated_positions(envelope):
    data = envelope.model_dump()
    data['position_min'][0], data['position_max'][0] = -.1, .1
    data['command_position_min'] = data['position_min'].copy()
    data['command_position_max'] = data['position_max'].copy()
    data['command_position_min'][0], data['command_position_max'][0] = -3., 3.
    return Envelope.model_validate(data)


def test_virtual_target_can_exceed_measured_bound_without_clipping(manifest, envelope, state):
    env = separated_positions(envelope)
    target = [0.] * 19
    target[0] = .2
    command = issue(Guard(manifest, env), state, targets=target)
    assert command.positions == tuple(target)
    # The same angle remains outside the measured-position envelope.
    measured = state.model_copy(deep=True)
    measured.positions[0] = .2
    with pytest.raises(ContractError, match='measured joint position'):
        Guard(manifest, env).check_state(measured, 100., 1000.)


def test_legacy_shared_bounds_still_reject_outside_target(manifest, envelope, state):
    env = separated_positions(envelope).model_dump()
    env['command_position_min'] = env['command_position_max'] = None
    target = [0.] * 19
    target[0] = .2
    with pytest.raises(ContractError, match='command position'):
        issue(Guard(manifest, Envelope.model_validate(env)), state, targets=target)


def test_wider_virtual_bounds_keep_tracking_torque_and_slew_checks(manifest, envelope, state):
    env = separated_positions(envelope)
    target = [0.] * 19
    target[0] = .6
    with pytest.raises(ContractError, match='tracking'):
        issue(Guard(manifest, env), state, targets=target)
    target[0] = .2
    high_gain = manifest.model_copy(deep=True)
    high_gain.gains.kp = [100.] * 19
    with pytest.raises(ContractError, match='predicted'):
        issue(Guard(high_gain, env), state, targets=target)
    guard = Guard(manifest, env)
    issue(guard, state)
    with pytest.raises(ContractError, match='slew'):
        issue(guard, state, targets=target, now=100.005)


@pytest.mark.parametrize('change', [
    {'command_position_min': [-3.] * 19},
    {'command_position_min': [-3.] * 18, 'command_position_max': [3.] * 19},
    {'command_position_min': [3.] * 19, 'command_position_max': [3.] * 19},
    {'command_position_min': [float('nan')] * 19, 'command_position_max': [3.] * 19},
])
def test_command_position_bounds_are_paired_ordered_finite_vectors(envelope, change):
    with pytest.raises(ValidationError):
        Envelope.model_validate(envelope.model_dump() | change)


def test_state_mailbox_latest_duplicate_error_and_close(state):
    box = LatestState()
    box.publish(state)
    assert box.get() is state
    with pytest.raises(ContractError, match="duplicate"):
        box.publish(state)
    with pytest.raises(ContractError, match="stream failed"):
        box.get()
    empty = LatestState()
    with pytest.raises(ContractError, match="startup"):
        empty.get(0.001)
    empty.close()
    with pytest.raises(ContractError, match="closed"):
        empty.get()


def test_bounded_join():
    thread = threading.Thread(target=lambda: time.sleep(0.05), daemon=True)
    thread.start()
    with pytest.raises(ContractError, match="deadline"):
        bounded_join(thread, 0.001)
    thread.join()


def test_schedule_does_not_catch_up():
    clock = Schedule(50, start=1)
    assert clock.advance(1) == pytest.approx(0.02)
    assert clock.advance(2) == pytest.approx(0.02)
    assert clock.next == pytest.approx(2.02)
