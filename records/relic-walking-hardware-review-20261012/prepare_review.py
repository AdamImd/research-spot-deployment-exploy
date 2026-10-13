"""Prepare a blocked faster-walking review; no SDK, lease or motion interfaces."""

import argparse
import json
import shutil
from pathlib import Path

from pydantic import ValidationError

from spot_deploy.contracts import Envelope, load, sha256
from spot_deploy.records import atomic_json, source_identity, verify_run
from spot_deploy.walking import WalkingPlan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-bundle', type=Path, required=True)
    parser.add_argument('--preflight', type=Path, required=True)
    parser.add_argument('--simulation-summary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = source_identity()
    if source['dirty']:
        raise RuntimeError('Use clean committed source for the review record')
    if not verify_run(args.preflight):
        raise RuntimeError('Preflight recording integrity did not pass')
    reference = args.reference_bundle
    manifest = json.loads((reference / 'policy/manifest.json').read_text())
    envelope = load(reference / 'envelope.json', Envelope)
    simulation = json.loads(args.simulation_summary.read_text())
    preflight = json.loads((args.preflight / 'result.json').read_text())
    snapshot = preflight['snapshot']
    plan = manifest['walking'] | {'max_speed_m_s': .3, 'initial_hold_s': 3.}
    try:
        WalkingPlan.model_validate(plan)
        fields = []
    except ValidationError as exc:
        fields = ['.'.join(map(str, e['loc'])) for e in exc.errors(include_input=False)]
    selected = {
        'status': 'review_only_not_activatable', 'hardware_qualified': False,
        'activation_approved': False, 'plan': plan, 'requested_speed_profile': 'constant',
        'goal_reference': 'Current post-native-stand position and yaw latched for this attempt',
        'goal_formula': 'p_goal = p_current + distance * [cos(yaw_current), sin(yaw_current), 0]',
        'policy_hz': manifest['policy_hz'], 'command_hz': manifest['stream_hz'],
        'startup': 'direct, no target fade, zero initial raw actions',
        'estop_authority': 'manufacturer tablet; no local E-stop bridge',
        'operators_for_final_confirmation': ['Adam', 'Minghao'],
        'limits_changed': False, 'gains_changed': False, 'weights_changed': False,
    }
    checks = {
        'preflight_recording_verified': True,
        'motors_off_at_capture': snapshot['motors_off'],
        'identity_matches': snapshot['identity_matches'],
        'model_matches_reference_policy': snapshot['robot_model_sha256'] == manifest['robot_model_sha256'],
        'payload_matches_reference_policy': snapshot['payload_config_sha256'] == manifest['payload_config_sha256'],
        'active_fault_count': snapshot['active_fault_count'],
        'stop_ready_at_capture': snapshot['estop_ready'],
        'tablet_operator_verification': 'Confirm tablet link/button and operator in position at final approval',
        'preflight_fresh_at_activation': 'Recollect against final approved manifest immediately before trial',
        'selected_plan_contract_invalid_fields': fields,
        'selected_request_strictly_below_body_speed_limit': plan['max_speed_m_s'] < envelope.max_linear_speed,
        'body_speed_limit_m_s': envelope.max_linear_speed,
        'constant_profile_available_on_hardware_path': False,
        'walking_distance_qualified': False,
        'source_bound_hardware_evidence_current': False,
        'simulations': {
            name: {k: cell[k] for k in ['final_forward_m', 'final_within_goal_tolerance',
                'first_fall_proxy_s', 'stable_final_window', 'leg_speed_reference_exceedances']}
            for name, cell in simulation['cells'].items()
        },
    }
    args.output.mkdir(parents=True, exist_ok=False)
    proposal = args.output / 'policy-proposal'
    shutil.copytree(reference / 'policy', proposal)
    original = proposal / 'manifest.json'
    original.unlink()
    manifest['walking'] = plan
    atomic_json(proposal / 'manifest.review.json', manifest)
    atomic_json(args.output / 'selected-trial.review.json', selected)
    atomic_json(args.output / 'checks.json', checks)
    shutil.copy2(reference / 'envelope.json', args.output / 'envelope.unchanged.json')
    atomic_json(args.output / 'evidence.json', {'schema_version': 1, 'evidence': []})
    atomic_json(args.output / 'activation.json', {
        'status': 'blocked_not_authorized', 'command': None,
        'required_final_question': 'Do you approve one recorded physical 0.75 m forward ReLIC trial '
            'using the exact reviewed manifest/envelope, with Adam operating and Minghao on the '
            'tablet, the rig following and path clear? This will power motors and move Spot.',
        'approval_required': 'Ask immediately before activation, after all gates pass; no automatic launch',
    })
    atomic_json(args.output / 'recording-template.json', {
        'status': 'template_only_no_robot_connection',
        'command': ['uv', 'run', '--no-sync', 'python', 'tools/record_rollout.py',
            '--run', 'APPROVED_TRIAL_RUN', '--output', 'NEW_COMPACT_RECORDING',
            '--manifest', 'FINAL_APPROVED_MANIFEST', '--state-hz', '0'],
        'scope': 'All unique sampled robot states, policy actions and commands; gzip JSONL',
        'includes': 'Frozen origin/goal, raw actions/ACKs, measured state and targets for PD-offset analysis',
        'note': 'Run after a completed or failed trial; never replace the raw safety log',
    })
    (args.output / 'REVIEW.md').write_text(
        '# 0.30 m/s walking hardware review\n\n'
        'Review only. No activation command is supplied. manifest.review.json is intentionally '
        'outside the current accepted walking contract; it cannot be promoted by renaming.\n\n'
        'Goal: 0.75 m ahead of the current post-stand heading; latch once per attempt. '
        'Initial hold 3 s, acceleration 0.1 m/s², policy-active cap 20 s, tolerance 2 cm. '
        'The selected constant request profile exists only in the simulation diagnostic.\n\n'
        'Resolve the 0.15 m/s host plan cap, reviewed body-speed headroom, guarded joint-speed '
        'failures, distance overshoot and source-bound qualification before preparing a live '
        'candidate. Keep manufacturer joint/load limits and tablet authority unchanged. '
        'Do not use unguarded simulation as passed hardware evidence.\n\n'
        'Fresh read-only preflight passed, motors off, no active faults, model/payload match. '
        'A ready robot stop state does not establish the tablet link or button response. '
        'Refresh preflight and confirm operators/rig/path at the final approval step.\n\n'
        'Record full state/action/timing/ACK/shutdown data, then create the lightweight '
        'robot/action archive with tools/record_rollout.py. No automatic retries.\n'
    )
    atomic_json(args.output / 'COMPLETE.json', {
        'status': 'review_prepared_activation_blocked', 'source': source,
        'hardware_motion_commands': 0, 'hardware_qualified': False,
        'preflight_complete_sha256': sha256(args.preflight / 'COMPLETE.json'),
        'reference_bundle_sha256': sha256(reference / 'bundle.json'),
        'simulation_summary_sha256': sha256(args.simulation_summary),
        'artifacts': {str(p.relative_to(args.output)): sha256(p)
                      for p in sorted(args.output.rglob('*')) if p.is_file()},
    })
    print(json.dumps({'review_created': True, 'activation_approved': False,
                      'hardware_qualified': False, 'output': str(args.output)}))


if __name__ == '__main__':
    main()
