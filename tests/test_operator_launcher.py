import json
import runpy
from pathlib import Path
from types import SimpleNamespace

import pytest

from spot_deploy.contracts import ContractError


ROOT = Path(__file__).resolve().parents[1]


def test_launcher_reports_recorded_fault_instead_of_generic_failure(tmp_path):
    reason = runpy.run_path(str(ROOT/'tools/run_joystick_bundle.py'))['failure_reason']
    (tmp_path/'result.json').write_text(json.dumps({'error': 'health failure during power-on'}))
    (tmp_path/'events.jsonl').write_text(json.dumps({'event': 'health', 'blocking_faults': [{
        'name': 'p1_system_monitor_fault', 'code': 2, 'message': 'CPU temperature exceeded'}]})+'\n')
    output = reason(tmp_path)
    assert 'p1_system_monitor_fault (code 2)' in output
    assert 'CPU temperature exceeded' in output


def test_launcher_reports_joystick_failure(tmp_path):
    reason = runpy.run_path(str(ROOT/'tools/run_joystick_bundle.py'))['failure_reason']
    (tmp_path/'result.json').write_text(json.dumps({'error': 'joystick event loss/resynchronization'}))
    assert reason(tmp_path) == 'joystick event loss/resynchronization'


def test_native_power_cancellation_preserves_health_cause_and_shutdown(monkeypatch):
    from spot_deploy import relic_control
    monkeypatch.setattr(relic_control, 'ReLICRollout', lambda *args: None)
    events = []

    class Control:
        closed = False
        def acquire(self):
            pass
        def heartbeat(self):
            raise ContractError('health worker original cause')
        def native_stand(self, stop):
            assert stop.wait(.5)
            raise ContractError('health failure during power-on')
        def close(self):
            self.closed = True

    control = Control()
    with pytest.raises(ContractError, match='health worker original cause'):
        relic_control.stand(object(), control, object(),
            SimpleNamespace(shutdown_timeout_s=.1), 300,
            SimpleNamespace(event=lambda name, **data: events.append((name, data))), None)
    assert control.closed
    assert events[0][0] == 'first_failure'
    assert events[0][1]['reason'] == 'health worker original cause'
