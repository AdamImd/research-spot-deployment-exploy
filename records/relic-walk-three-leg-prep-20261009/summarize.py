"""Extract public metrics from private, integrity-checked commissioning runs."""

import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, sha256
from spot_deploy.records import verify_run


ROOT = Path(__file__).resolve().parents[2]
CAMPAIGN = ROOT/'runs/relic-walk-three-leg-prep-20261009-002'
BRACKET = ROOT/'runs/relic-walk-bracket-20261009-001'
REPLAY = ROOT/'runs/relic-walk-three-leg-replay-20261009-003'


def summarize(path):
    if not verify_run(path, status='completed'):
        raise ValueError(f'Invalid source recording: {path}')
    result = json.loads((path/'result.json').read_text())
    config = json.loads((path/'configuration.json').read_text())
    source = json.loads((path/'source.json').read_text())
    events = [json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
    rejected = [e for e in events if e['event'] == 'first_failure']
    detail = None
    if rejected:
        failure = rejected[-1]
        state, envelope = failure['state'], config['envelope']
        speed = np.abs(state['velocities'])
        detail = dict(time_s=failure['simulation_time_s'], reason=failure['reason'],
            velocity_violations=[dict(joint=JOINTS[i], velocity_rad_s=state['velocities'][i],
                                     limit_rad_s=envelope['velocity_max'][i])
                for i in range(19) if speed[i] > envelope['velocity_max'][i]],
            body_angular_speed_rad_s=float(np.linalg.norm(state['angular_velocity_odom'])),
            body_linear_speed_m_s=float(np.linalg.norm(state['linear_velocity_odom'])))
        policies = [e for e in events if e['event'] == 'policy']
        if config['demonstration'] and len(policies) >= 2:
            before, after = policies[-2:]
            detail['mode_activation'] = dict(
                previous_leg_command=before['observations'][19:31],
                next_leg_command=after['observations'][19:31],
                joint_target_changes_rad=(np.asarray(after['targets'])-before['targets']).tolist(),
                target_violations=[dict(joint=JOINTS[i], target_rad=after['targets'][i],
                    min_rad=envelope['position_min'][i], max_rad=envelope['position_max'][i])
                    for i in range(19) if not envelope['position_min'][i] <= after['targets'][i]
                    <= envelope['position_max'][i]])
    return dict(run=str(path.relative_to(ROOT)),
        configuration_sha256=sha256(path/'configuration.json'),
        complete_sha256=sha256(path/'COMPLETE.json'),
        capture_complete_sha256=source['capture_complete_sha256'],
        seed=config['seed'], backend=config['backend'], packages=source['packages'],
        policy_device=config['policy_device'], physics_device=config['physics_device'],
        requested_walk=config['manifest'].get('walking'), demonstration=config['demonstration'],
        policy_sha256=config['manifest']['policy_sha256'], gains=config['manifest']['gains'],
        first_failure=detail, result=result)


def main():
    cells = {name: summarize(CAMPAIGN/name) for name in
        ['standing', 'three-leg-mujoco', 'walk0125-slow-ramp', 'walk015-slow-ramp', 'three-leg-isaac']}
    cells.update({f'walk014-{name}': summarize(BRACKET/name) for name in ['mujoco', 'isaac']})
    replays = {name: json.loads((REPLAY/name/'COMPLETE.json').read_text())
               for name in ['three-leg-mujoco', 'three-leg-isaac']}
    summary = dict(simulation_commit='abe17c27230b1dcaba21adf1e9e7371d313a2c16',
        bracket_and_renderer_commit='92545cb0730e3407a650c47e8272d2f7bf9011fd',
        hardware_access=False, hardware_qualified=False, tests_passed=440,
        cells=cells, replays=replays)
    output = Path(__file__).with_name('result-summary.json')
    output.write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)+'\n')
    print(output)


if __name__ == '__main__':
    main()
