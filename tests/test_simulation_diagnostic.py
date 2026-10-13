# ruff: noqa: F811
# Imported pytest fixtures intentionally share test argument names.
import numpy as np
import pytest

from spot_deploy.contracts import JOINTS, ContractError
from spot_deploy.safety import Guard
from spot_deploy.simulation_diagnostic import (
    DiagnosticCommands, DiagnosticTravel, SimulationDiagnosticRollout, demand_summary,
)
from test_control_loop import Log
from test_relic_rollout import FakeReLIC, harness_configuration, relic_envelope, relic_manifest, sample  # noqa: F401
from test_walking import plan, posed  # noqa: F401


def configuration(manifest, envelope):
    manifest, envelope = harness_configuration(manifest, envelope)
    return manifest, envelope.model_copy(update=dict(scope='simulation'))


def test_no_hardware_envelope_can_construct_unguarded_core(relic_manifest, relic_envelope):
    manifest, envelope = configuration(relic_manifest, relic_envelope)
    for scope in ['hardware', 'simulation']:
        env = envelope.model_copy(update=dict(scope=scope))
        if scope == 'hardware':
            with pytest.raises(ContractError, match='simulation scope'):
                SimulationDiagnosticRollout(FakeReLIC(manifest), env, Log())
        else:
            assert SimulationDiagnosticRollout(FakeReLIC(manifest), env, Log())


def test_operating_limits_bypassed_without_changing_live_guard(state, relic_manifest, relic_envelope):
    manifest, envelope = configuration(relic_manifest, relic_envelope)
    current = sample(state, 0)
    current.velocities[0] = 99
    target = np.array(current.positions)
    target[0] = 10
    with pytest.raises(ContractError):
        Guard(manifest, envelope).command(target, current, 10, 1000, 0, 1)
    diagnostic = DiagnosticCommands(manifest, envelope, Log())
    command = diagnostic.command(target, current, 10, 1000, 0, 1)
    assert command.positions[0] == 10 and command.key == 1
    target[0] = np.nan
    with pytest.raises(ContractError, match='nonfinite'):
        diagnostic.command(target, current, 10.005, 1000.005, 0, 1)
    target[0] = 10
    target[12] += .1
    with pytest.raises(ContractError, match='arm movement'):
        diagnostic.command(target, current, 10.005, 1000.005, 0, 1)


def test_history_starts_zero_and_advances_only_on_ack(state, relic_manifest, relic_envelope):
    manifest, envelope = configuration(relic_manifest, relic_envelope)
    policy = FakeReLIC(manifest)
    core = SimulationDiagnosticRollout(policy, envelope, Log())
    current = sample(state, 0)
    core.initialize(current, dict(height_m=.52, foot_contacts=[1]*4), 10, 1000)
    hold = core.command(current, 10, 1000)
    core.activate(10.005)
    command = core.command(sample(state, .005, hold.key), 10.005, 1000.005)
    np.testing.assert_array_equal(policy.calls[0], np.zeros(12))
    core.command(sample(state, .025, hold.key), 10.025, 1000.025)
    assert len(policy.calls) == 1
    core.command(sample(state, .030, command.key), 10.030, 1000.030)
    np.testing.assert_allclose(policy.calls[1], .01)


def test_walk_stall_drift_and_timeout_recorded_without_stopping(state, plan):
    travel = DiagnosticTravel(plan, Log())
    travel.latch(posed(state, 0))
    travel.activate(10)
    for t in [3, 4, 10, 21, 30]:
        assert travel.update(posed(state, t, y=.2), 10+t)[0] > 0
    assert not travel.completed
    assert {'walking progress stalled', 'walking distance timeout',
            'walking lateral or heading deviation limit'} <= travel.violations


def test_demands_separate_prefall_peaks_and_reorder_joints(relic_envelope):
    rows = []
    for t, speed, height in [(0, 2, .52), (1, 9, .52), (2, 40, .2)]:
        rows.append(dict(trial_time=t, q=[0.]*19, dq=[0.]*18+[speed],
            positions=[0.]*19, pd_torque_Nm=[0.]*19, preceding_applied_torque_Nm=[0.]*19,
            height=height, tilt=0, linear_velocity=[0, 0, 0], angular_velocity=[0, 0, 0]))
    config = dict(simulation_joint_names=list(reversed(JOINTS)), envelope=relic_envelope.model_dump())
    report = demand_summary(rows, config)
    assert report['all_samples']['fl_hx']['speed_rad_s']['peak_abs'] == 40
    assert report['before_first_fall']['fl_hx']['speed_rad_s']['peak_abs'] == 9
    assert report['first_fall_proxy_s'] == 2 and not report['hardware_qualified']


def test_faster_profile_preserves_acceleration_initial_hold_and_distance_stop(state, plan):
    travel = DiagnosticTravel(plan, Log(), forward_speed=.3, speed_profile='constant')
    travel.latch(posed(state, 0))
    travel.activate(10)
    commands = []
    for tick in range(1, 1401):
        t = tick*.005
        commands.append(travel.update(posed(state, t), 10+t)[0])
    assert max(commands[:int(plan.initial_hold_s/.005)-1]) == 0
    assert max(abs(np.diff(commands))) <= plan.acceleration_m_s2*.005+1e-12
    assert commands[-1] == pytest.approx(.3)
    assert travel.update(posed(state, 7.005, x=plan.distance_m-.01), 17.005)[0] == 0
    assert plan.max_speed_m_s <= .15  # Original reviewed manifest is never rewritten.


def test_taper_override_and_invalid_simulation_speeds(plan):
    assert DiagnosticTravel(plan, Log(), forward_speed=.3)._desired_speed(.08) == pytest.approx(.064)
    assert DiagnosticTravel(plan, Log(), forward_speed=.3, speed_profile='constant')._desired_speed(.08) == .3
    assert DiagnosticTravel(plan, Log(), forward_speed=.3, speed_profile='constant')._desired_speed(-.08) == 0
    for value in [0, -1, 1.1, np.nan, np.inf]:
        with pytest.raises(ContractError, match='simulation forward speed'):
            DiagnosticTravel(plan, Log(), forward_speed=value)
    with pytest.raises(ContractError, match='explicit speed'):
        DiagnosticTravel(plan, Log(), speed_profile='constant')


def test_speed_override_reaches_real_graph_without_relaxing_live_adapter(state, plan, relic_envelope):
    from pathlib import Path
    from spot_deploy.contracts import check_artifacts
    from spot_deploy.exploy_policy import ExployReLICManifest
    from spot_deploy.relic_contract import load_manifest, make_policy
    from spot_deploy.relic_policy import observation
    from spot_deploy.simulation_diagnostic import SimulationVelocityPolicy
    root = Path(__file__).parents[1]
    path = root/'policies/relic-exploy-standing/manifest.json'
    data = load_manifest(path).model_dump()
    data.update(task='walking', walking=plan.model_dump())
    manifest = ExployReLICManifest.model_validate(data)
    policy = make_policy(check_artifacts(path, manifest), manifest, path)
    policy.initialize_height(dict(height_m=.52, foot_contacts=[1]*4))
    env = relic_envelope.model_copy(update=dict(scope='simulation'))
    wrapped = SimulationVelocityPolicy(policy, .3, env)
    previous = np.zeros(12, np.float32)
    with pytest.raises(ContractError, match='walking plan'):
        policy.evaluate(state, previous, [.3, 0, 0])
    target, _, obs, action = wrapped.evaluate(state, previous, [.3, 0, 0])
    expected = observation(state, manifest.arm_stowed_positions, previous, .52, [.3, 0, 0])
    np.testing.assert_allclose(obs[0], expected, atol=2e-7, rtol=0)
    assert np.isfinite(action).all()
    np.testing.assert_array_equal(target[12:], manifest.arm_stowed_positions)
    assert policy.manifest.walking.max_speed_m_s == plan.max_speed_m_s
    np.testing.assert_array_equal(policy.previous_action, previous)
    for invalid in [[.31, 0, 0], [.2, .01, 0], [.2, 0, .01], [-.2, 0, 0]]:
        with pytest.raises(ContractError, match='simulation forward'):
            wrapped.evaluate(state, previous, invalid)
    with pytest.raises(ContractError, match='simulation scope'):
        SimulationVelocityPolicy(policy, .3, env.model_copy(update=dict(scope='hardware')))
