"""Recompute the sanitized trial summary from private recorded telemetry."""
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import sha256
from spot_deploy.policy import rotation
from spot_deploy.contracts import State
from spot_deploy.records import verify_run


def main():
    raw = Path('runs/relic-standing-harness-20261009-001')
    compact = Path('runs/relic-standing-harness-compact-20261009-001')
    post = Path('runs/relic-standing-harness-post-preflight-20261009-001')
    candidate = Path('local/relic-standing-harness-20261009-002')
    for path, status in [(raw, 'passed'), (compact, 'completed'), (post, 'passed')]:
        assert verify_run(path, status=status)
    events = [json.loads(line) for line in (raw / 'events.jsonl').read_text().splitlines()]
    def selected(name):
        return [event for event in events if event['event'] == name]
    result = json.loads((raw / 'result.json').read_text())
    manifest = json.loads((candidate / 'policy/manifest.json').read_text())
    commands, policies = selected('command'), selected('policy')
    q = np.array([event['state']['positions'] for event in commands])
    dq = np.array([event['state']['velocities'] for event in commands])
    target = np.array([event['positions'] for event in commands])
    loads = np.array([event['state']['loads'] for event in commands])
    ff = np.array([event['feedforward'] for event in commands])
    p_term = (target - q) * np.array(manifest['gains']['kp'])
    d_term = -dq * np.array(manifest['gains']['kd'])
    tilt = []
    for event in commands:
        r = rotation(State.model_validate(event['state']))
        tilt.append([np.arctan2(r[2, 1], r[2, 2]), np.arcsin(np.clip(-r[2, 0], -1, 1))])
    tilt = np.degrees(tilt)
    last_window = np.array([event['monotonic_s'] >= commands[-1]['monotonic_s'] - 2 for event in commands])
    observation = selected('arm_motion_observation')
    init = selected('relic_initialized')[0]
    shutdown = selected('shutdown_result')[-1]
    post_result = json.loads((post / 'result.json').read_text())
    compact_summary = json.loads((compact / 'summary.json').read_text())
    summary = dict(
        runtime_commit=json.loads((raw / 'run.json').read_text())['source']['commit'],
        completed=True, duration_s=result['duration_s'], policy_predictions=len(policies),
        commands=len(commands), action_history_acknowledgements=len(selected('policy_ack')),
        initial_previous_actions_zero=all(v == 0 for v in init['previous_actions']),
        initial_body_height_m=init['height']['height_m'],
        initial_foot_contacts=init['height']['foot_contacts'],
        policy_first_to_last_s=policies[-1]['monotonic_s'] - policies[0]['monotonic_s'],
        first_failure=None, arm_motion_mode='observe', arm_torque_guards='enforced',
        arm_motion_observation_events=len(observation),
        arm_position_violation_joints=sorted({i for event in observation for i in event['position_indices']}),
        arm_velocity_violation_joints=sorted({i for event in observation for i in event['velocity_indices']}),
        arm_tracking_violation_joints=sorted({i for event in observation for i in event['tracking_indices']}),
        arm_stow_displacement_observed=any(event['stow_displaced'] for event in observation),
        arm_targets_held=bool(np.all(target[:, 12:] == np.array(manifest['arm_stowed_positions']))),
        max_inference_ms=max(event['inference_s'] for event in policies) * 1000,
        command_gap_ms={key: value * 1000 for key, value in result['command_gap_s'].items() if key != 'count'},
        command_ack_ms={key: value * 1000 for key, value in result['command_ack_s'].items() if key != 'count'},
        max_abs_leg_target_minus_measured_rad=np.max(np.abs(target[:, :12] - q[:, :12]), axis=0).tolist(),
        max_abs_leg_p_term_Nm=np.max(np.abs(p_term[:, :12]), axis=0).tolist(),
        max_abs_leg_d_term_Nm=np.max(np.abs(d_term[:, :12]), axis=0).tolist(),
        max_abs_leg_estimated_torque_Nm=np.max(np.abs((p_term + d_term + ff)[:, :12]), axis=0).tolist(),
        max_abs_leg_reported_load_Nm=np.max(np.abs(loads[:, :12]), axis=0).tolist(),
        max_abs_roll_pitch_deg=np.max(np.abs(tilt), axis=0).tolist(),
        last_2s_max_abs_roll_pitch_deg=np.max(np.abs(tilt[last_window]), axis=0).tolist(),
        last_2s_max_leg_speed_rad_s=float(np.max(np.abs(dq[last_window, :12]))),
        body_drift_measured=False,
        shutdown_s=shutdown['elapsed_s'], motors_off_confirmed=shutdown['motors_off_confirmed'],
        lease_returned=shutdown['lease_returned'], shutdown_errors=shutdown['errors'],
        within_approved_10s_shutdown_budget=shutdown['within_shutdown_budget'],
        post_preflight_motor_off=post_result['snapshot']['motors_off'],
        post_preflight_all_checks=all(post_result['checks'].values()),
        compact_bytes=compact_summary['compressed_bytes'],
        raw_complete_sha256=sha256(raw / 'COMPLETE.json'),
        compact_complete_sha256=sha256(compact / 'COMPLETE.json'),
        post_complete_sha256=sha256(post / 'COMPLETE.json'),
        policy_qualified=False, retry_policy='none',
    )
    assert not selected('first_failure') and result['shutdown_confirmed']
    Path(__file__).with_name('result-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print('Sanitized summary regenerated; full hardware qualification remains false.')


if __name__ == '__main__':
    main()
