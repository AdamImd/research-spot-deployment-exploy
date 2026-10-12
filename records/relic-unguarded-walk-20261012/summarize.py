"""Rebuild sanitized metrics from integrity-bound offline recordings."""

import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, sha256
from spot_deploy.records import verify_run

ROOT = Path(__file__).resolve().parents[2]
CELLS = {
    'mujoco-default-friction': 'runs/relic-unguarded-walk-20261012-001/mujoco',
    'isaaclab': 'runs/relic-unguarded-walk-20261012-001/isaaclab',
    'mujoco-matched-noslip10': 'runs/relic-unguarded-walk-20261012-002/mujoco',
}


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(x) for x in path.read_text().splitlines()]


def main():
    summary = dict(simulation_commit='8f11bcd689d31c1c2397aa8b9bce1b913336435d',
        hypothesis='Operating stops truncate a rollout that may or may not remain stable',
        hardware_access=False, hardware_qualified=False, seed=101, cells={})
    for name, relative in CELLS.items():
        path = ROOT / relative
        assert verify_run(path, status='completed'), relative
        config, result = read(path/'configuration.json'), read(path/'result.json')
        trajectory, events = rows(path/'rollout.jsonl'), rows(path/'events.jsonl')
        origin = next(e for e in events if e['event'] == 'walk_origin')
        axis = np.array([np.cos(origin['yaw_rad']), np.sin(origin['yaw_rad'])])
        feet = np.array([r['feet'] for r in trajectory])
        foot_forward = (feet[-1, :, :2] - feet[0, :, :2]) @ axis
        speeds = np.array([r['dq'] for r in trajectory])
        q = np.array([r['q'] for r in trajectory])
        times = np.array([r['trial_time'] for r in trajectory])
        derivative = np.diff(q, axis=0) / np.diff(times)[:, None]
        simulation_names = [n.replace('arm_', 'arm0_') for n in config['simulation_joint_names']]
        leg_ids = [simulation_names.index(n) for n in JOINTS[:12]]
        spike_row, leg = np.unravel_index(np.argmax(abs(speeds[:, leg_ids])), (len(times), 12))
        joint_id = leg_ids[leg]
        summary['cells'][name] = dict(source_recording=relative,
            source_complete_sha256=sha256(path/'COMPLETE.json'),
            rollout_sha256=sha256(path/'rollout.jsonl'),
            configuration_sha256=sha256(path/'configuration.json'),
            seed=config['seed'], backend=config['backend'], guard_mode=config['guard_mode'],
            prewarm_inference=config['prewarm_inference'],
            mujoco_noslip_iterations=config['mujoco_noslip_iterations'],
            physics_device=config['physics_device'], policy_device=config['policy_device'],
            policy_sha256=config['manifest']['policy_sha256'],
            gains=config['manifest']['gains'], walking_plan=config['manifest']['walking'],
            reason=result['reason'], trial_duration_s=result['trial_duration_s'],
            walking_completed=result['walking_completed'], walking=result['walking'],
            stable_final_window=result['stable_final_window'], late_window=result['late_window_metrics'],
            first_fall_proxy_s=result['first_fall_proxy_s'], demands=result['demands'],
            feet_forward_displacements_m=foot_forward.tolist(),
            mean_feet_forward_displacement_m=float(foot_forward.mean()),
            peak_speed_check=dict(joint=JOINTS[leg], time_s=float(times[spike_row]),
                reported_rad_s=float(speeds[spike_row, joint_id]),
                preceding_position_derivative_rad_s=float(derivative[spike_row-1, joint_id])
                    if spike_row else None),
            max_observation_error=result['max_observation_error'],
            max_raw_action_error=result['max_raw_action_error'],
            max_target_error_rad=result['max_target_error_rad'], timing=result['timing'])
    baseline = ROOT/'runs/relic-walking-tracking-20261012-001/mujoco'
    matched = ROOT/CELLS['mujoco-matched-noslip10']
    assert verify_run(baseline, status='completed')
    a, b = rows(baseline/'rollout.jsonl'), rows(matched/'rollout.jsonl')
    errors = {k: float(np.max(abs(np.array([r[k] for r in a])
                   - np.array([r[k] for r in b[:len(a)]]))))
              for k in ['q', 'dq', 'positions', 'pd_torque_Nm', 'position']}
    summary['guarded_prefix_comparison'] = dict(baseline_recording=str(baseline.relative_to(ROOT)),
        baseline_complete_sha256=sha256(baseline/'COMPLETE.json'),
        accepted_samples=len(a), through_s=a[-1]['trial_time'], maximum_errors=errors,
        identical_accepted_trajectory=all(v == 0 for v in errors.values()))
    destination = Path(__file__).with_name('result-summary.json')
    destination.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(destination)


if __name__ == '__main__':
    main()
