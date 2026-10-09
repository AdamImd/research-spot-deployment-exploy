from types import SimpleNamespace

import pytest
from bosdyn.api.robot_state_pb2 import PowerState

from spot_deploy.contracts import ContractError
from spot_deploy.sdk_control import ControlSpot


def control_for(power, *, power_error=False, cancel_error=False, lease_error=False):
    control = ControlSpot.__new__(ControlSpot)
    calls = []

    def operation(name, fail=False):
        def call(*args, **kwargs):
            calls.append(name)
            if fail:
                raise RuntimeError('injected failure')
        return call

    control.future = SimpleNamespace(cancel=operation('cancel', cancel_error))
    control.power_owned = True
    control.robot = SimpleNamespace(power_off=operation('power_off', power_error))
    control.reader = SimpleNamespace(state_client=SimpleNamespace(get_robot_state=lambda **_: 
        SimpleNamespace(power_state=SimpleNamespace(motor_power_state=power))))
    control.lease = object()
    control.lease_client = SimpleNamespace(return_lease=operation('return_lease', lease_error))
    control.timeout = 1
    control.envelope = SimpleNamespace(shutdown_timeout_s=2)
    return control, calls


@pytest.mark.parametrize('power', [PowerState.STATE_UNKNOWN, PowerState.STATE_ON,
                                 PowerState.STATE_POWERING_OFF, PowerState.STATE_POWERING_ON])
def test_only_explicit_off_confirms_shutdown(power):
    control, calls = control_for(power)
    with pytest.raises(ContractError, match='Safe power-off unconfirmed'):
        control.close()
    assert calls == ['cancel', 'power_off', 'return_lease']
    assert not control.shutdown_result['motors_off_confirmed']
    assert control.shutdown_result['lease_returned']


def test_successful_shutdown_has_separate_stage_evidence():
    control, _ = control_for(PowerState.STATE_OFF)
    status = control.close()
    assert status['motors_off_confirmed'] and status['lease_returned']
    assert status['errors'] == []
    assert status['elapsed_s'] >= status['safe_power_off_elapsed_s'] >= 0


@pytest.mark.parametrize('failure', ['power_error', 'cancel_error', 'lease_error'])
def test_cleanup_attempts_remaining_stages_after_failure(failure):
    control, calls = control_for(PowerState.STATE_OFF, **{failure: True})
    with pytest.raises(ContractError):
        control.close()
    assert calls == ['cancel', 'power_off', 'return_lease']
    assert len(control.shutdown_result['errors']) == 1
