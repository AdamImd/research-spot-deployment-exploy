import importlib.util
from pathlib import Path

import numpy as np

from test_relic_rollout import (
    FakeReLIC,
    relic_envelope as relic_envelope,
    relic_manifest as relic_manifest,
    sample,
)

SPEC = importlib.util.spec_from_file_location(
    'startup_audit', Path(__file__).parents[1]/'tools/audit_relic_startup.py')
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def test_audit_independent_zero_history_and_feedforward_removal(
        state, relic_manifest, relic_envelope):  # noqa: F811 -- imported pytest fixtures
    relic_manifest.relic.preparation = relic_manifest.relic.preparation.model_copy(
        update=dict(kind='direct', duration_s=0))
    relic_envelope.transition_s = 0
    policy = FakeReLIC(relic_manifest)
    state = sample(state, 0)
    height = dict(height_m=.52, foot_contacts=[1]*4)
    first = audit.audit_sample(policy, relic_envelope, state, height)
    second = audit.audit_sample(policy, relic_envelope, state, height)
    assert first['guard_passed'] and second['guard_passed']
    np.testing.assert_array_equal(first['raw_action'], second['raw_action'])
    for previous in policy.calls:
        np.testing.assert_array_equal(previous, 0)
    np.testing.assert_allclose(first['torque_step_Nm'],
                               np.array(first['policy_torque_Nm']) - 1)


def test_audit_reports_measured_state_failure_without_relaxing_guard(
        state, relic_manifest, relic_envelope):  # noqa: F811 -- imported pytest fixtures
    relic_manifest.relic.preparation = relic_manifest.relic.preparation.model_copy(
        update=dict(kind='direct', duration_s=0))
    relic_envelope.transition_s = 0
    state = sample(state, 0)
    state.positions[0] = relic_envelope.position_max[0]+.01
    result = audit.audit_sample(FakeReLIC(relic_manifest), relic_envelope, state,
                                dict(height_m=.52, foot_contacts=[1]*4))
    assert not result['guard_passed']
    assert result['first_guard_failure'] == 'measured joint position limit'
