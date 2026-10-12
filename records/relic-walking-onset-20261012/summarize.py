"""Recompute sanitized walking-review metrics from preserved private simulations."""

import json
from pathlib import Path

from spot_deploy.contracts import sha256
from spot_deploy.records import verify_run
from spot_deploy.walk_diagnostics import summarize

ROOT = Path(__file__).resolve().parents[2]
CELLS = {
    'hold1-speed014-slow': 'relic-walking-onset-20261012-001/walk014-hold1',
    'hold3-speed014-slow': 'relic-walking-onset-20261012-001/walk014-hold3',
    'hold3-speed013-slow': 'relic-walking-onset-20261012-001/walk013-hold3',
    'hold3-speed0135-slow': 'relic-walking-onset-20261012-002/walk0135-hold3-slow',
    'hold3-speed014-fast': 'relic-walking-onset-20261012-002/walk014-hold3-fast',
    'hold3-speed0135-fast': 'relic-walking-onset-20261012-002/walk0135-hold3-fast',
    'hold3-speed013-fast': 'relic-walking-onset-20261012-003/walk013-hold3-fast',
    'hold3-speed01325-fast': 'relic-walking-onset-20261012-003/walk01325-hold3-fast',
    'approved-simulation-tracking05': 'relic-walking-tracking-20261012-001/mujoco',
    'archived-isaac014': 'relic-walk-bracket-20261009-001/isaac',
}


def cell(relative):
    path = ROOT/'runs'/relative
    if not verify_run(path, status='completed'):
        raise ValueError('Invalid source integrity')
    config = json.loads((path/'configuration.json').read_text())
    result = json.loads((path/'result.json').read_text())
    source = json.loads((path/'source.json').read_text())
    rows = [json.loads(line) for line in (path/'rollout.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
    diagnostics, _, _, failure = summarize(rows, config, events)
    if failure and diagnostics['first_failure']['tracking_violations']:
        for violation in diagnostics['first_failure']['tracking_violations']:
            from spot_deploy.contracts import JOINTS
            violation['preceding_measured_load_Nm'] = failure['state']['loads'][JOINTS.index(violation['joint'])]
    return dict(run='runs/'+relative, backend=config['backend'], seed=config['seed'],
        policy_device=config['policy_device'], physics_device=config['physics_device'],
        parameters=config['manifest']['walking'], gains=config['manifest']['gains'],
        fl_hy_tracking_cap_rad=config['envelope']['tracking_error_max'][1],
        graph_sha256=config['manifest']['policy_sha256'],
        capture_complete_sha256=source['capture_complete_sha256'], packages=source['packages'],
        configuration_sha256=sha256(path/'configuration.json'),
        complete_sha256=sha256(path/'COMPLETE.json'),
        outcome={k: result[k] for k in ('reason', 'failure', 'trial_duration_s', 'walking_completed',
            'stable_final_window', 'max_observation_error', 'max_raw_action_error',
            'max_target_error_rad', 'timing')}, diagnostics=diagnostics)


def main():
    review = ROOT/'local/relic-walking-tracking-review-20261012-001'
    data = dict(hardware_access=False, hardware_qualified=False, real_hardware_deployment=False,
        instruction='Do not deploy on real hardware', simulation_commit='f7b414a6a55c124bd5a08f66da300d28ef69ccec',
        diagnostic_acceptance_commit='799a0824e29f2ba276660660f95e1f116c3f0311',
        tests_passed=445, cells={name: cell(path) for name, path in CELLS.items()},
        simulation_only_tracking_review=json.loads((review/'review.json').read_text()),
        simulation_only_approval=json.loads((review/'approval.json').read_text()),
        native_replay=json.loads((ROOT/'runs/relic-walking-review-artifacts-20261012-001'
                                 /'stepping-replay/COMPLETE.json').read_text()),
        slow_replay=json.loads((ROOT/'runs/relic-walking-slow-replay-20261012-002'
                               /'slow-replay-verification.json').read_text()))
    Path(__file__).with_name('result-summary.json').write_text(
        json.dumps(data, indent=2, sort_keys=True, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
