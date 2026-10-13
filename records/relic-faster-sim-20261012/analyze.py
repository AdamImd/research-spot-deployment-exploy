"""Analyze requests, measured forward motion and goal stopping from frozen sim logs."""
import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import sha256
from spot_deploy.records import verify_run

ROOT = Path(__file__).resolve().parents[2]


def load(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    summary = dict(hardware_access=False, hardware_qualified=False, request_m_s=.3, cells={})
    curves = {}
    base = ROOT/'runs/relic-faster-sim-20261012-002'
    for backend in ['mujoco', 'isaaclab']:
        for profile in ['taper', 'constant']:
            name = backend+'-'+profile
            path = base/name
            assert verify_run(path, status='completed')
            result, config = load(path/'result.json'), load(path/'configuration.json')
            assert result['reason'] != 'integrity_stop' and result['trial_duration_s'] == 60
            events, trajectory = rows(path/'events.jsonl'), rows(path/'rollout.jsonl')
            origin = next(e for e in events if e['event'] == 'walk_origin')
            axis = np.array([np.cos(origin['yaw_rad']), np.sin(origin['yaw_rad'])])
            position = np.array([r['position'] for r in trajectory])
            forward = (position[:, :2]-origin['odom_position_m'][:2])@axis
            t = np.array([r['trial_time'] for r in trajectory])
            predictions = [e for e in events if e['event'] == 'policy']
            pt = np.array([e['monotonic_s']-10 for e in predictions])
            pv = np.array([e['velocity_command'][0] for e in predictions])
            requested = pv[np.clip(np.searchsorted(pt, t+1e-8, side='right')-1, 0, len(pv)-1)]
            states = [e['state'] for e in events if e['event'] == 'command']
            assert len(states) == len(trajectory) == 12001
            actual = np.array([s['linear_velocity_odom'][:2] for s in states])@axis
            goal = config['manifest']['walking']['distance_m']
            tol = config['manifest']['walking']['tolerance_m']
            crossings = np.flatnonzero(abs(forward-goal) <= tol)
            completion = [e for e in events if e['event'] == 'walk_progress' and e['phase'] == 'completed']
            high = requested >= .25
            summary['cells'][name] = dict(recording=str(path.relative_to(ROOT)),
                complete_sha256=sha256(path/'COMPLETE.json'), rollout_sha256=sha256(path/'rollout.jsonl'),
                source_sha256=sha256(path/'source.json'), policy_sha256=config['manifest']['policy_sha256'],
                manifest_file_sha256=sha256(ROOT/'local/relic-unguarded-walk-20261012-001/candidate/manifest.json'),
                policy_samples=result['policy_samples'], state_samples=len(trajectory),
                requested_forward_max_m_s=float(requested.max()), actual_forward_peak_m_s=float(actual.max()),
                actual_forward_median_when_request_ge025_m_s=float(np.median(actual[high])) if high.any() else None,
                final_forward_m=float(forward[-1]), maximum_forward_m=float(forward.max()),
                first_within_goal_tolerance_s=float(t[crossings[0]]) if len(crossings) else None,
                completion_s=completion[0]['monotonic_s']-10 if completion else None,
                walking_completed=result['walking_completed'], final_walking=result['walking'],
                final_within_goal_tolerance=bool(abs(forward[-1]-goal) <= tol),
                stable_final_window=result['stable_final_window'], first_fall_proxy_s=result['first_fall_proxy_s'],
                late_window=result['late_window_metrics'], demands=result['demands'],
                max_raw_action_error=result['max_raw_action_error'], max_target_error_rad=result['max_target_error_rad'],
                configuration=dict(speed_override=config['simulation_forward_speed_m_s'],profile=config['simulation_speed_profile'],
                    acceleration=config['manifest']['walking']['acceleration_m_s2'],
                    initial_hold=config['manifest']['walking']['initial_hold_s'],
                    noslip=config['mujoco_noslip_iterations'], seed=config['seed'], providers=config['providers']))
            curves[name] = (t, forward, requested, actual, [r['height'] for r in trajectory])
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), layout='constrained')
    for col, backend in enumerate(['mujoco', 'isaaclab']):
        for profile, color in [('taper', '#d97706'), ('constant', '#0284c7')]:
            t, x, request, actual, height = curves[backend+'-'+profile]
            axes[0, col].plot(t, x, label=profile, color=color)
            axes[1, col].plot(t, actual, color=color, label=profile+' measured')
            axes[1, col].plot(t, request, '--', color=color, label=profile+' requested', alpha=.8)
            axes[2, col].plot(t, height, color=color, label=profile)
        axes[0, col].axhspan(.73, .77, color='#22c55e', alpha=.12, label='goal ±0.02 m')
        axes[0, col].set_title(backend)
        axes[0, col].set_ylabel('Forward displacement (m)')
        axes[1, col].set_ylabel('Forward velocity (m/s)')
        axes[2, col].set_ylabel('Body height (m)')
        for row in range(3):
            axes[row, col].grid(alpha=.25)
            axes[row, col].set_xlim(0, 25)
            axes[row, col].set_xlabel('Simulation time (s)')
            axes[row, col].legend(fontsize=8)
    fig.suptitle('0.30 m/s simulation request: increased maximum versus removing near-goal taper\nOriginal weights/gains; operating stops disabled; measured speed is not requested speed')
    fig.savefig(args.output/'comparison.png', dpi=160)
    fig.savefig(args.output/'comparison.pdf')
    plt.close(fig)
    (args.output/'COMPLETE.json').write_text(json.dumps(dict(status='completed', hardware_access=False,
        tool_sha256=sha256(Path(__file__)), artifacts={p.name:sha256(p) for p in args.output.iterdir() if p.is_file()}),indent=2)+'\n')
    print(json.dumps({k:{x:v[x] for x in ['first_within_goal_tolerance_s','walking_completed','final_forward_m','requested_forward_max_m_s','actual_forward_peak_m_s','stable_final_window','first_fall_proxy_s']} for k,v in summary['cells'].items()},indent=2))


if __name__ == '__main__':
    main()
