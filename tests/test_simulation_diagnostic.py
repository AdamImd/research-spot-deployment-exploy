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
