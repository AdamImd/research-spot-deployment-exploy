"""Operator-started, recorded session from a frozen reviewed joystick bundle.

--execute attests the named operators, functioning tablet and reviewed travelling
rig/arm workspace are ready now. Without it, this prints the profile and exits.
One invocation, no retries; every invocation creates a fresh recording directory.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from spot_deploy.contracts import sha256
from spot_deploy.records import atomic_json, source_identity, verify_run
from spot_deploy.relic_contract import load_manifest

ROOT = Path(__file__).resolve().parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--operator', default='Adam')
    parser.add_argument('--safety-operator', default='Minghao')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    os.chdir(ROOT)
    bundle = args.bundle.resolve()
    manifest = bundle/'policy/manifest.json'
    plan = load_manifest(manifest)
    if plan.task != 'joystick-arm':
        raise ValueError('requires a reviewed joystick-arm bundle')
    print(f'{plan.teleop.duration_s:g}s; hold printed2 enables, trigger1 toggles arm, '
          'printed4 finishes; tablet sole E-stop.', flush=True)
    if not args.execute:
        print('Prepared only. --execute confirms operators/tablet/rig/workspace ready and starts motors.')
        return 0
    source = source_identity()
    if source['dirty']:
        raise ValueError('requires clean committed source')
    if not args.operator.strip() or not args.safety_operator.strip() or (
            args.operator.strip().casefold() == args.safety_operator.strip().casefold()):
        raise ValueError('two distinct named operators required')
    out = ROOT/'runs'/('joystick-operator-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    out.mkdir(parents=True)
    atomic_json(out/'RUN.json', dict(source=source, bundle=str(bundle),
        manifest_sha256=sha256(manifest), envelope_sha256=sha256(bundle/'envelope.json'),
        evidence_sha256=sha256(bundle/'evidence.json'), operator=args.operator,
        safety_operator=args.safety_operator, explicit_operator_execute=True,
        setup_attestation='Named operators/tablet/rig/travel and arm/shutdown workspace ready',
        duration_s=plan.teleop.duration_s, automatic_retries=0))
    print(f'Recording: {out}', flush=True)
    child = None
    interrupted = False

    def request_stop(*_):
        nonlocal interrupted
        if not interrupted:
            interrupted = True
            if child is not None and child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)

    handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}

    def call(name, command, timeout):
        nonlocal child
        with (out/(name+'.log')).open('x') as log:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            try:
                code = child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                request_stop()
                try:
                    code = child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    (out/'STATUS').write_text('shutdown_unconfirmed_tablet_required\n')
                    print('Shutdown unconfirmed: use the tablet E-stop.', flush=True)
                    raise
            finally:
                if child.poll() is not None:
                    child = None
        (out/(name+'.exit')).write_text(str(code)+'\n')
        return code

    prefix = ['uv', 'run', '--no-sync', 'spot-deploy']
    preflight = prefix+['preflight', '--manifest', str(manifest), '--robot', str(bundle/'robot.json')]
    try:
        (out/'STATUS').write_text('preflight\n')
        code = call('preflight', preflight+['--output', str(out/'preflight')], 25)
        if code or interrupted:
            (out/'STATUS').write_text('preflight_failed_or_cancelled\n')
            return code or 1
        (out/'STATUS').write_text('trial_running\n')
        code = call('trial', prefix+['joystick', '--manifest', str(manifest),
            '--envelope', str(bundle/'envelope.json'), '--robot', str(bundle/'robot.json'),
            '--evidence', str(bundle/'evidence.json'), '--duration', str(plan.teleop.duration_s),
            '--operator', args.operator, '--safety-operator', args.safety_operator,
            '--output', str(out/'trial'), '--execute'], plan.teleop.duration_s+90)
        post = call('postcheck', preflight+['--output', str(out/'postcheck')], 25)
        result_file = out/'postcheck/result.json'
        off = result_file.is_file() and json.loads(result_file.read_text()).get(
            'snapshot', {}).get('motors_off') is True
        compact = None
        if (out/'trial/COMPLETE.json').exists():
            compact = call('compact', ['uv', 'run', '--no-sync', 'python', 'tools/record_rollout.py',
                '--run', str(out/'trial'), '--output', str(out/'compact'), '--state-hz', '50',
                '--manifest', str(manifest)], 60)
            if compact == 0 and not verify_run(out/'compact', status='completed'):
                compact = 1
        atomic_json(out/'OUTCOME.json', dict(trial_exit=code, postcheck_exit=post,
            compact_exit=compact, motors_off_confirmed=off, automatic_retries=0))
        status = ('completed' if code == post == compact == 0 and off
                  else 'finished_with_failure' if off else 'shutdown_unconfirmed_tablet_required')
        (out/'STATUS').write_text(status+'\n')
        if not off:
            print('Motors-off unconfirmed: use the tablet E-stop.', flush=True)
        print(f'{status}; recording: {out}', flush=True)
        return 0 if status == 'completed' else 1
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
