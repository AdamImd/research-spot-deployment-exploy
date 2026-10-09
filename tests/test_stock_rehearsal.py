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


@pytest.mark.parametrize("operator,safety", [("Adam", "adam"), ("Adam", ""), ("", "Minghao")])
def test_distinct_named_operators(operator, safety):
    with pytest.raises(ContractError):
        stock.require_operators(operator, safety)


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
