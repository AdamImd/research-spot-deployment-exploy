"""Benchmark timing must report configured limits separately from stream cadence."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location(
    'benchmark_relic_gpu', Path(__file__).resolve().parents[1] / 'tools/benchmark_relic_gpu.py')
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


def test_selected_watchdogs_and_nominal_period_have_separate_budgets():
    envelope = SimpleNamespace(max_inference_s=.005, max_command_gap_s=.015)
    sample = dict(core_s=.00527, production_s=.00542, deadline_response_s=.00577,
                  gap_s=.018, inference_s=.006, state_decode_s=.0001,
                  log_encode_s=.0001, release_lateness_s=.0001)
    result = benchmark.timing_stats([sample], envelope, .01)
    assert result['core_s']['budget_s'] == .005
    assert result['core_s']['over_budget'] == 1
    assert result['inference_s']['over_budget'] == 1
    assert result['gap_s']['budget_s'] == .015
    assert result['gap_s']['over_budget'] == 1  # A hardcoded .020 would miss this.
    assert result['production_s']['budget_s'] == .01
    assert result['production_s']['over_budget'] == 0
    assert result['deadline_response_s']['over_budget'] == 0


def test_unmeasured_intervals_are_not_reported_as_zero():
    envelope = SimpleNamespace(max_inference_s=.005, max_command_gap_s=.015)
    sample = dict(core_s=.001, production_s=.002, deadline_response_s=.003,
                  gap_s=None, inference_s=None, state_decode_s=.0001,
                  log_encode_s=.001, release_lateness_s=.001)
    result = benchmark.timing_stats([sample], envelope, .005)
    assert result['gap_s'] is None and result['inference_s'] is None
    assert result['core_s']['over_budget'] == 0
