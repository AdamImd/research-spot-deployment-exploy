"""Sanitize offline PD-target validation; private paths remain CLI inputs only."""
import argparse
import json
import importlib.util
from pathlib import Path

from spot_deploy.contracts import Envelope, load, sha256
from spot_deploy.records import atomic_json, verify_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--envelope', required=True, type=Path)
    args = parser.parse_args()
    assert (args.run / 'STATUS').read_text().strip() == 'completed'
    contract = json.loads((args.run / 'contract.json').read_text())
    envelope = load(args.envelope, Envelope)
    benchmark_path = Path.cwd() / 'tools/benchmark_relic_gpu.py'
    spec = importlib.util.spec_from_file_location('benchmark', benchmark_path)
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)
    summary = {'execution_commit': contract['commit'], 'hardware_access': False,
               'new_physical_trial': False, 'physical_policy_qualified': False,
               'automatic_retries': False, 'seed': 101, 'prewarm_inferences': 100,
               'guard_mode': 'enforced', 'stages': {},
               'envelope_sha256': sha256(args.envelope),
               'analysis_tool_sha256': sha256(Path(__file__)),
               'benchmark_analysis_tool_sha256': sha256(benchmark_path),
               'virtual_knee_target_rad': [-3., 0.], 'virtual_hip_target_rad': [-3., 3.],
               'leg_tracking_error_max_rad': list(envelope.tracking_error_max[:12]),
               'measured_angle_limits_changed': False}
    for name in ['hardware-replay', 'mujoco-walk', 'isaaclab-walk',
                 'mujoco-stand', 'isaaclab-stand', 'timing']:
        path = args.run / name
        assert verify_run(path, status='completed'), name
        result = json.loads((path / 'result.json').read_text())
        entry = {'complete_sha256': sha256(path / 'COMPLETE.json'),
                 'result_sha256': sha256(path / 'result.json')}
        if name == 'hardware-replay':
            entry.update(result)
        elif name == 'timing':
            for key in ['reason', 'failure', 'command_hz', 'command_samples', 'policy_samples',
                        'missed_releases', 'observed_5ms_gate', 'max_raw_action_error', 'timing']:
                entry[key] = result[key]
            samples = [json.loads(line) for line in (path / 'timing.jsonl').open()]
            entry['original_timing'] = entry.pop('timing')
            entry['timing'] = benchmark.timing_stats(samples, envelope, .005)
            entry['timing_trace_sha256'] = sha256(path / 'timing.jsonl')
            entry['live_computation_deadline_pass'] = entry['timing']['core_s']['over_budget'] == 0
            entry['live_command_gap_pass'] = entry['timing']['gap_s']['over_budget'] == 0
            entry['live_inference_deadline_pass'] = entry['timing']['inference_s']['over_budget'] == 0
        else:
            for key in ['reason', 'failure', 'guard_mode', 'policy_duration_s', 'trial_duration_s',
                        'policy_samples', 'stable_final_window', 'first_fall_proxy_s',
                        'max_observation_error', 'max_raw_action_error', 'max_target_error_rad',
                        'real_time_factor', 'late_window_metrics', 'walking_completed']:
                entry[key] = result[key]
            if result['walking']:
                entry['walking'] = {key: result['walking'][key] for key in
                    ['forward_m', 'lateral_m', 'remaining_m', 'heading_error_rad', 'speed_m_s']}
        summary['stages'][name] = entry
    summary['timing_gate_passed'] = (summary['stages']['timing']['observed_5ms_gate']
        and summary['stages']['timing']['live_computation_deadline_pass'])
    summary['ready_for_new_physical_trial'] = False
    summary['decision'] = 'Target-bound simulation validation passed; timing gate failed. No new trial.'
    summary['limitations'] = ('Ideal simulated state/ACK clocks and host-only timing; no physical '
        'transport test. Historical rejected-step slew is unverified. Model/firmware saturation '
        'equivalence is not established. A short simulated walk is not goal completion.')
    atomic_json(args.output, summary)
    print(json.dumps({'execution_commit': summary['execution_commit'],
                      'output': str(args.output), 'hardware_access': False}))


if __name__ == '__main__':
    main()
