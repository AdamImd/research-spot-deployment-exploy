import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from spot_deploy.contracts import ContractError

SPEC = importlib.util.spec_from_file_location(
    "stock_rehearsal", Path(__file__).resolve().parents[1] / "tools/rehearse_stock.py")
stock = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stock)


@pytest.mark.parametrize("initially_on", [False, True])
def test_exact_stock_sequence(initially_on):
    calls = []
    io = SimpleNamespace(**{
        name: (lambda *args, name=name: calls.append((name, *args)))
        for name in ("sit", "stand", "power_on", "power_off", "hold")})
    stock.stock_sequence(io, initially_on)
    prefix = [("sit", "initial_sit"), ("power_off", "initial_power_off")] if initially_on else []
    assert calls == prefix + [("power_on",), ("stand",), ("hold", "standing_observation", 10.),
                              ("sit", "sit"), ("hold", "sitting_observation", 2.),
                              ("power_off", "power_off")]


def test_name_free_native_inspection_cli(tmp_path, monkeypatch):
    import sys
    args = ["rehearse_stock", "--manifest", "manifest", "--robot", "robot", "--envelope", "envelope",
            "--output", str(tmp_path / "result")]
    monkeypatch.setattr(sys, "argv", args)
    def inspect(options, record):
        assert options.operator is None and options.safety_operator is None
        assert not options.execute
        return {"inspection_only": True}
    monkeypatch.setattr(stock, "run", inspect)
    assert stock.main() == 0


def test_ui_release_ends_native_hold_without_sleep(monkeypatch):
    phases = []
    io = stock.StockIO(None, None, None, lambda: None, phases.append, lambda: True)
    monkeypatch.setattr(stock.time, "sleep", lambda _: pytest.fail("release ignored"))
    io.hold("standing_observation", 300)
    assert phases == ["standing_observation"]


@pytest.mark.parametrize("change", [{"severity": "SEVERITY_WARN"}, {"code": 10},
                                    {"name": "other.fault"}])
def test_only_exact_informational_warning_is_accepted(change):
    fault = {"name": "payload.fault", "code": 9, "severity": "SEVERITY_INFO"}
    stock.require_stock_faults({"system_fault_state": [fault]}, True)
    with pytest.raises(ContractError):
        stock.require_stock_faults({"system_fault_state": [fault]}, False)
    with pytest.raises(ContractError):
        stock.require_stock_faults({"behavior_fault_state": [fault]}, True)
    with pytest.raises(ContractError):
        stock.require_stock_faults({"system_fault_state": [fault | change]}, True)


@pytest.mark.parametrize("power", [0, 1, 2, 3, 4])
def test_only_explicit_off_confirms_shutdown(power):
    from bosdyn.api.robot_state_pb2 import PowerState

    reader = SimpleNamespace(config=SimpleNamespace(rpc_timeout_s=1), state_client=SimpleNamespace(
        get_robot_state=lambda **_: SimpleNamespace(power_state=SimpleNamespace(motor_power_state=power))))
    if power == PowerState.STATE_OFF:
        stock.require_power_off(reader)
    else:
        with pytest.raises(ContractError, match="not confirmed"):
            stock.require_power_off(reader)


def test_timed_out_power_on_still_requires_safe_cleanup():
    def timeout(**kwargs):
        raise TimeoutError()

    reader = SimpleNamespace(config=SimpleNamespace(rpc_timeout_s=1),
                             robot=SimpleNamespace(power_on=timeout))
    io = stock.StockIO(reader, None, None, lambda: None, lambda _: None)
    with pytest.raises(TimeoutError):
        io.power_on()
    assert io.power_managed


def test_bounded_camera_standing_observation():
    from types import SimpleNamespace
    calls = []
    io = SimpleNamespace(**{name: (lambda *args, name=name: calls.append((name, *args)))
                            for name in ('sit', 'power_off', 'power_on', 'stand', 'hold')})
    stock.stock_sequence(io, initially_on=False, standing_seconds=300.)
    assert ('hold', 'standing_observation', 300.) in calls
    assert calls[-1] == ('power_off', 'power_off')
    calls.clear()
    for duration in (0, 601, float('nan'), float('inf')):
        with pytest.raises(stock.ContractError):
            stock.stock_sequence(io, False, duration)
    assert not calls
