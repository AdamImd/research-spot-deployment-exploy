import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from spot_deploy.contracts import ContractError, Envelope, check_artifacts, load
from spot_deploy.joystick_estop import ButtonEvent
from spot_deploy.joystick_teleop import (
    ArmPath, JoystickInput, JoystickSample, JoystickTeleopPlan, TeleopMotion,
)
from spot_deploy.relic_contract import make_policy
from spot_deploy.relic_policy import DEFAULT_Q, ReLICPolicy, joint_targets, observation
from spot_deploy.teleop_policy import TeleopManifest
from spot_deploy.safety import Guard

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def plan():
    return load(ROOT/'examples/joystick-relic-arm.json', JoystickTeleopPlan)


@pytest.fixture
def mobile(plan):
    data = json.loads((ROOT/'policies/relic-exploy-standing/manifest.json').read_text())
    return TeleopManifest.model_validate(data | dict(schema_version=5, task='joystick-arm',
        morphology='spot-arm-mobile', teleop=plan.model_dump()))


class FakeLink:
    metadata = dict(axis_count=4, button_count=12)
    def __init__(self):
        self.axes = {i: 0. for i in range(4)}
        self.buttons = {}
        self.events = []
        self.closed = False

    def poll(self):
        events, self.events = self.events, []
        return events

    def close(self):
        self.closed = True


def test_center_enable_three_axes_release_and_trigger(plan):
    link = FakeLink()
    link.events = [ButtonEvent(i, False, True) for i in range(12)]
    source = JoystickInput(plan, link)
    assert not source.poll(0).enabled
    # Held startup/INIT never enables or toggles.
    link.events = [ButtonEvent(1, True, True), ButtonEvent(0, True, True)]
    assert not source.poll(.01).enabled
    link.events = [ButtonEvent(1, False), ButtonEvent(0, False)]
    source.poll(.02)
    link.axes[0] = .8
    link.events = [ButtonEvent(1, True)]
    assert not source.poll(.03).enabled  # No enabling with off-center stick.
    link.axes[0] = 0
    link.events = [ButtonEvent(1, False)]
    source.poll(.04)
    link.events = [ButtonEvent(1, True), ButtonEvent(0, True)]
    assert source.poll(.05).toggle
    link.axes.update({0: -1., 1: -1., 2: -1.})
    sample = source.poll(.06)
    assert sample.velocity == (.3, .2, .5)
    assert not sample.toggle
    link.events = [ButtonEvent(0, False), ButtonEvent(0, True), ButtonEvent(1, False)]
    sample = source.poll(.07)
    assert not sample.enabled and not sample.toggle and sample.velocity == (0, 0, 0)
    with pytest.raises(ContractError, match='expired'):
        source.poll(.5)


def test_finish_and_disconnect(plan):
    link = FakeLink()
    source = JoystickInput(plan, link)
    link.events = [ButtonEvent(3, True)]
    assert source.poll(0).finish
    def lost():
        raise ContractError('disconnected')
    link.poll = lost
    with pytest.raises(ContractError, match='disconnected'):
        source.poll(.01)


def test_arm_path_pause_reverse_and_speed(mobile):
    arm = ArmPath(mobile.arm_stowed_positions, mobile.teleop)
    original = arm.reference.copy()
    arm.advance(0, True, True)
    previous = original
    for _ in range(4000):
        q = arm.advance(.005, True, False)
        assert np.max(np.abs(q-previous)) <= .5*.005+1e-10
        previous = q
    np.testing.assert_allclose(q, mobile.teleop.arm_waypoints[-1])
    np.testing.assert_array_equal(arm.advance(1, False, False), q)
    arm.advance(0, True, True)
    for _ in range(4000):
        q = arm.advance(.005, True, False)
    np.testing.assert_array_equal(q, original)


def test_motion_hold_release_no_catchup(mobile):
    current = JoystickSample(0, (.3, 0, 0), True, True, False)
    source = SimpleNamespace(poll=lambda now: current)
    motion = TeleopMotion(mobile, source)
    assert motion.update(0, 0)[0] == [0, 0, 0]
    for i in range(1, 701):
        now = i*.005
        current = JoystickSample(now, (.3, 0, 0), True, i == 610, False)
        motion.update(now, 0)
    assert motion.velocity[0] <= .051
    assert motion.arm.progress > 0
    before = motion.arm.reference.copy()
    current = JoystickSample(3.505, (0, 0, 0), False, False, False)
    assert motion.update(3.505, 0)[0] == [0, 0, 0]
    np.testing.assert_array_equal(motion.arm.reference, before)
    current = JoystickSample(3.505, (0, 0, 0), False, False, False)
    with pytest.raises(ContractError, match='stale'):
        motion.update(4, 0)


def test_mobile_guard_does_not_relax_arm_or_legacy_contract(mobile, state):
    env = load(ROOT/'fixtures/relic-direct-simulation/envelope.json', Envelope)
    env = env.model_copy(update=dict(arm_motion_guard='observe', arm_motion_reason='harness'))
    with pytest.raises(ContractError, match='enforced arm'):
        Guard(mobile, env)
    env = env.model_copy(update=dict(arm_motion_guard='enforce', arm_motion_reason=None))
    env.position_min[13] = -3.2
    env.position_max[13] = -3.0
    with pytest.raises(ContractError, match='arm path'):
        Guard(mobile, env)


def compliant_guard_inputs(mobile, state):
    env = load(ROOT/'fixtures/relic-direct-simulation/envelope.json', Envelope)
    values = env.model_dump()
    values.update(arm_tracking_guard='observe',
                  arm_tracking_reason='Operator: compliant manipulation')
    env = Envelope.model_validate(values)
    env.tracking_error_max[12:] = [.15]*7
    target = np.asarray(state.positions).copy()
    target[12:] = mobile.teleop.arm_waypoints[-1]
    current = state.model_copy(deep=True)
    current.positions = target.tolist()
    current.positions[14] += .16
    current.velocities = [0.]*19
    current.loads = [0.]*19
    return env, target, current


def test_compliant_arm_tracking_continues_but_default_stops(mobile, state):
    env, target, current = compliant_guard_inputs(mobile, state)
    enforced = env.model_copy(update={'arm_tracking_guard': 'enforce', 'arm_tracking_reason': None})
    ordinary = Guard(mobile, enforced)
    ordinary.set_arm_reference(target[12:])
    with pytest.raises(ContractError, match='outside commanded pose'):
        ordinary.check_state(current, current.received_monotonic_s, current.robot_time_s)
    guard = Guard(mobile, env)
    guard.set_arm_reference(target[12:])
    for i in range(121):
        sample = current.model_copy(update={
            'received_monotonic_s': current.received_monotonic_s+i*.005,
            'robot_time_s': current.robot_time_s+i*.005})
        guard.command(target, sample, sample.received_monotonic_s,
                      sample.robot_time_s, sample.received_monotonic_s, 0)
    assert guard.motion_joint_count == 19 and guard.tracking_joint_count == 12
    assert guard.arm_tracking_observation['reference_indices'] == [2]
    assert guard.arm_tracking_observation['measured_tracking_indices'] == [2]
    assert guard.arm_tracking_observation['command_tracking_indices'] == [2]
    values = env.model_dump()
    values['arm_tracking_reason'] = None
    with pytest.raises(ValueError, match='operator reason'):
        Envelope.model_validate(values)


@pytest.mark.parametrize('violation,match', [
    ('position', 'measured joint position'), ('speed', 'measured joint velocity'),
    ('load', 'measured joint velocity or load'),
    ('pd', 'predicted PD-plus-feedforward'), ('leg_tracking', 'command tracking error'),
])
def test_compliant_arm_keeps_other_stops(mobile, state, violation, match):
    env, target, current = compliant_guard_inputs(mobile, state)
    if violation == 'position':
        current.positions[14] = env.position_max[14]+.01
    elif violation == 'speed':
        current.velocities[16] = env.velocity_max[16]+.01
    elif violation == 'load':
        current.loads[14] = env.load_max[14]+1
    elif violation == 'pd':
        env.load_max[14] = 1
    else:
        env.tracking_error_max[0] = .01
        current.positions[0] += .02
    guard = Guard(mobile, env)
    guard.set_arm_reference(target[12:])
    with pytest.raises(ContractError, match=match):
        guard.command(target, current, current.received_monotonic_s,
                      current.robot_time_s, current.received_monotonic_s, 0)


def test_real_graph_three_axes_moving_arm_parity(mobile, state):
    path = ROOT/'policies/relic-exploy-standing/manifest.json'
    policy = make_policy(check_artifacts(path, mobile), mobile, path)
    actor = ReLICPolicy(ROOT/'simulation/source/relic/relic/assets/spot/pretrained/policy.onnx', .52)
    policy.initialize_height(dict(height_m=.52, foot_contacts=[1]*4))
    rng = np.random.default_rng(101)
    for _ in range(32):
        arm = np.asarray(mobile.arm_stowed_positions) + rng.uniform(0, 1)*(
            np.asarray(mobile.teleop.arm_waypoints[-1])-mobile.arm_stowed_positions)
        policy.arm_command = arm
        velocity = rng.uniform(-1, 1, 3)*[.3, .2, .5]
        previous = rng.normal(0, .1, 12).astype(np.float32)
        measured = state.model_copy(update=dict(positions=(DEFAULT_Q+rng.normal(0, .02, 19)).tolist()))
        target, _, obs, raw = policy.evaluate(measured, previous, velocity)
        reference = observation(measured, arm, previous, .52, velocity)
        expected = actor.session.run(['actions'], {'obs': reference[None]})[0][0]
        np.testing.assert_allclose(obs[0], reference, rtol=0, atol=1e-5)
        np.testing.assert_allclose(raw, expected, rtol=0, atol=1e-4)
        np.testing.assert_allclose(target, joint_targets(expected, arm), rtol=0, atol=3e-5)
        np.testing.assert_array_equal(policy.previous_action, np.zeros(12))
    policy.reset()
    policy.warmup()
    np.testing.assert_array_equal(policy.arm_command, mobile.arm_stowed_positions)
    np.testing.assert_array_equal(policy.previous_action, np.zeros(12))
    assert policy.body_height is None


def test_shared_rollout_200hz_arm_and_ack_history(mobile, state):
    from spot_deploy.relic_rollout import ReLICRollout
    from test_control_loop import Log
    from test_relic_rollout import FakeReLIC

    class Actor(FakeReLIC):
        def evaluate(self, state, previous, velocity_command=None):
            self.velocity = velocity_command
            target, elapsed, obs, raw = super().evaluate(state, previous)
            target[12:] = self.arm_command
            return target, elapsed, obs, raw

    policy = Actor(mobile)
    env = load(ROOT/'fixtures/relic-direct-simulation/envelope.json', Envelope)
    # Inference is fake; test scheduling/history and exact arm references, not dynamics.
    env.max_state_gap_s = .02
    source = SimpleNamespace(poll=lambda now: JoystickSample(now, (.1, 0, .1), True,
                                                             13.099 < now < 13.101, False))
    log = Log()
    core = ReLICRollout(policy, env, log, source)
    current = state.model_copy(update=dict(positions=DEFAULT_Q.astype(float).tolist(),
                                          velocities=[0.]*19, loads=[0.]*19))
    current.positions[12:] = mobile.arm_stowed_positions
    current.robot_time_s, current.received_monotonic_s = 999.995, 9.995
    core.initialize(current, dict(height_m=.52, foot_contacts=[1]*4), 9.995, 999.995)
    command = core.command(current, 9.995, 999.995)
    core.activate(10.)
    commands = []
    for i in range(720):
        now = 10+i*.005
        current = current.model_copy(update=dict(positions=list(command.positions),
            robot_time_s=now+990, received_monotonic_s=now, last_command_key=command.key))
        command = core.command(current, now, now+990)
        commands.append(command)
    np.testing.assert_array_equal(policy.calls[0], np.zeros(12))
    np.testing.assert_allclose(policy.calls[1], .01)
    assert len(policy.calls) == 180
    assert policy.velocity[0] > 0 and policy.velocity[2] > 0
    arm = np.asarray([c.positions[12:] for c in commands])
    assert np.any(np.diff(arm[-60:], axis=0))  # Changes between 50 Hz actor ticks too.
    assert np.max(np.abs(np.diff(arm, axis=0))) <= .005*.5+1e-9
    assert np.all(arm[:, -1] == mobile.arm_stowed_positions[-1])


def test_new_task_evidence_required_before_robot_or_usb(mobile, tmp_path, monkeypatch):
    from spot_deploy.readiness import report
    path = tmp_path/'manifest.json'
    path.write_text(mobile.model_dump_json())
    gates = {r['gate'] for r in report(manifest_path=path)['checks']}
    assert {'joystick_control', 'arm_deployment', 'teleop_clearance'} <= gates
    from spot_deploy.cli import execute, parser
    args = parser().parse_args(['joystick', '--manifest', str(path), '--envelope', 'unused',
        '--robot', 'unused', '--operator', 'Adam', '--safety-operator', 'Minghao',
        '--duration', '60', '--output', str(tmp_path/'run')])
    assert not args.execute
    # The existing require_live gate rejects before reader construction; direct
    # protocol check keeps these tests free of local robot configs/credentials.
    from spot_deploy.readiness import require_live
    with pytest.raises(ContractError, match='explicit execution'):
        require_live(mobile, None, None, path, None, None, None, 'Adam', 'Minghao', False, 60)
    assert callable(execute)


def test_live_wrapper_rejects_unbound_profile_before_io():
    import runpy
    main = runpy.run_path(str(ROOT/'tools/joystick_relic.py'))['main']
    with pytest.raises(SystemExit) as exc:
        main(['run', '--profile', 'unbound.json'])
    assert exc.value.code == 2


@pytest.mark.parametrize('mode,duration,expected', [
    ('joystick', 300, 'readiness sentinel'),
    ('joystick', 301, 'duration must be finite'),
    ('shadow', 300, 'duration must be finite'),
])
def test_five_minute_cli_reaches_gates_without_robot_io(mobile, tmp_path, monkeypatch,
                                                       mode, duration, expected):
    from spot_deploy import cli

    plan = mobile.teleop.model_copy(update={'duration_s': 300})
    candidate = mobile.model_copy(update={'teleop': plan})
    env = load(ROOT/'fixtures/relic-direct-simulation/envelope.json', Envelope)
    monkeypatch.setattr(cli, 'load_manifest', lambda _: candidate)
    monkeypatch.setattr(cli, 'load', lambda _, cls: env if cls is Envelope else object())
    monkeypatch.setattr(cli, 'check_artifacts', lambda *_: Path('unused.onnx'))
    monkeypatch.setattr(cli, 'make_policy', lambda *_: SimpleNamespace(manifest=candidate))

    def readiness(*args):
        assert args[-1] == 300
        raise ContractError('readiness sentinel')

    monkeypatch.setattr(cli, 'require_live', readiness)
    values = [mode, '--manifest', 'unused', '--envelope', 'unused', '--robot', 'unused',
              '--duration', str(duration), '--output', str(tmp_path/'run')]
    if mode == 'joystick':
        values += ['--operator', 'Adam', '--safety-operator', 'Minghao']
    with pytest.raises(ContractError, match=expected):
        cli.execute(cli.parser().parse_args(values), SimpleNamespace())
