"""Replay recorded simulation commands through numeric guards without robot I/O."""

import argparse
import json
from pathlib import Path

from spot_deploy.contracts import Envelope, State, load, sha256
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest
from spot_deploy.safety import Guard


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--envelope', required=True, type=Path)
    parser.add_argument('--runs', required=True, type=Path, nargs='+')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    manifest = load_manifest(args.manifest)
    envelope = load(args.envelope, Envelope)
    cells = {}
    args.output.mkdir(parents=True, exist_ok=False)
    for run in args.runs:
        if not verify_run(run, status='completed'):
            raise ValueError('Recording integrity failed')
        guard = Guard(manifest, envelope)
        count, policy_at, inference = 0, None, None
        first_failure = None
        for line in (run / 'events.jsonl').open():
            event = json.loads(line)
            if event['event'] == 'policy':
                policy_at, inference = event['monotonic_s'], event['inference_s']
            if event['event'] != 'command':
                continue
            state = State.model_validate(event['state'])
            now = 10 + event['simulation_time_s']
            try:
                guard.command(event['positions'], state, now, state.robot_time_s,
                    policy_at, inference, event['feedforward'])
            except Exception as exc:
                first_failure = {'time_s': now - 10, 'reason': str(exc)}
                break
            count += 1
        cells[run.name] = {'commands_checked': count, 'first_failure': first_failure,
            'complete_sha256': sha256(run / 'COMPLETE.json'),
            'passed': count == 12001 and first_failure is None}
    atomic_json(args.output / 'result.json', {'cells': cells,
        'passed': all(c['passed'] for c in cells.values()),
        'manifest_sha256': sha256(args.manifest), 'envelope_sha256': sha256(args.envelope),
        'hardware_access': False,
        'limitations': 'Recorded logical simulation clocks; numeric guards and recorded inference '
            'duration only. Does not qualify live transport, fresh-state handover or distance completion.'})
    atomic_json(args.output / 'COMPLETE.json', {'status': 'completed',
        'tool_sha256': sha256(Path(__file__)), 'artifacts': {
            'result.json': sha256(args.output / 'result.json')}})
    print(json.dumps(cells))
    return 0 if all(c['passed'] for c in cells.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
