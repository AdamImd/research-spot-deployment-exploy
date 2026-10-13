"""Recheck the recorded trial and its rejected prediction with proposed target bounds."""

import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import Envelope, State, load, sha256
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest
from spot_deploy.safety import Guard


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--envelope', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    status = json.loads((args.run / 'COMPLETE.json').read_text())['status']
    assert verify_run(args.run, status=status)
    manifest = load_manifest(args.manifest)
    envelope = load(args.envelope, Envelope)
    guard = Guard(manifest, envelope)
    latest_policy, count = None, 0
    for line in (args.run / 'events.jsonl').open():
        event = json.loads(line)
        if event['event'] == 'policy':
            latest_policy = event
        if event['event'] != 'command':
            continue
        state = State.model_validate(event['state'])
        now = event['monotonic_s']
        if event['phase'] == 'policy':
            at = min(latest_policy['monotonic_s'], latest_policy['state']['received_monotonic_s'])
            inference = latest_policy['inference_s']
        else:
            at, inference = state.received_monotonic_s, 0.
        command = guard.command(event['positions'], state, now, state.robot_time_s,
                                at, inference, event['feedforward'])
        np.testing.assert_array_equal(command.positions, event['positions'])
        count += 1
    state = State.model_validate(latest_policy['state'])
    at = min(latest_policy['monotonic_s'], state.received_monotonic_s)
    command = guard.command(latest_policy['targets'], state, latest_policy['monotonic_s'],
                            state.robot_time_s, at, latest_policy['inference_s'], [0.] * 19)
    np.testing.assert_array_equal(command.positions, latest_policy['targets'])
    result = {'passed': count == 1135, 'recorded_commands_checked': count,
        'previously_rejected_prediction_passes': True, 'targets_clipped_by_host': False,
        'hardware_access': False, 'new_physical_trial': False,
        'limitations': 'Recorded states only; does not predict future physical motion or new timing.',
        'raw_complete_sha256': sha256(args.run / 'COMPLETE.json'),
        'manifest_sha256': sha256(args.manifest), 'envelope_sha256': sha256(args.envelope)}
    args.output.mkdir(parents=True, exist_ok=False)
    atomic_json(args.output / 'result.json', result)
    atomic_json(args.output / 'COMPLETE.json', {'status': 'completed',
        'tool_sha256': sha256(Path(__file__)), 'artifacts': {'result.json': sha256(args.output / 'result.json')}})
    print(json.dumps(result))


if __name__ == '__main__':
    main()
