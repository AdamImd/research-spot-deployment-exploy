from types import SimpleNamespace

from spot_deploy.sdk_read import classify_faults


def config(mode='known-payload-info-observe'):
    return SimpleNamespace(fault_policy=mode)


def fault(**changes):
    return dict(dict(kind='system_fault_state', name='payload.fault', code=9,
                     severity=1, message='COM mismatch'), **changes)


def test_exact_info_preserved_but_does_not_block():
    value = fault()
    result = classify_faults([value], 1, config())
    assert result['fault_policy_ready']
    assert result['accepted_faults'] == [value]
    assert not result['blocking_faults']


def test_default_and_other_faults_still_block():
    assert not classify_faults([fault()], 1, config('all-block'))['fault_policy_ready']
    for changed in [fault(code=10), fault(severity=2), fault(name='other'),
                    fault(kind='behavior_fault_state')]:
        result = classify_faults([fault(), changed], 2, config())
        assert not result['fault_policy_ready']
        assert result['blocking_faults'] == [changed]


def test_missing_fault_details_cannot_bypass_stop():
    assert not classify_faults([], 1, config())['fault_policy_ready']
    assert not classify_faults([fault()], 2, config())['fault_policy_ready']
