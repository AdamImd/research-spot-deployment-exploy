"""Standalone USB ReLIC teleop entry point. Default is joystick-only preview."""
import argparse
import json
from pathlib import Path
import shutil
import time

from spot_deploy.contracts import ContractError, artifact_path, check_artifacts, load
from spot_deploy.deployment import policy_files
from spot_deploy.joystick_teleop import JoystickInput, JoystickTeleopPlan, TeleopMotion
from spot_deploy.relic_contract import load_manifest
from spot_deploy.teleop_policy import TeleopManifest
from spot_deploy.cli import safe_error

ROOT = Path(__file__).resolve().parents[1]


def prepare(base, plan, output):
    manifest = load_manifest(base)
    if manifest.schema_version != 3 or manifest.task not in ('standing', 'walking'):
        raise ContractError('prepare from a standing or walking nine-input Exploy manifest')
    check_artifacts(base, manifest)
    data = manifest.model_dump()
    data.update(schema_version=5, task='joystick-arm', morphology='spot-arm-mobile',
                walking=None, teleop=plan.model_dump())
    prepared = TeleopManifest.model_validate(data)
    output.mkdir(parents=True, exist_ok=False)
    for name in policy_files(manifest):
        source = artifact_path(base, name)
        target = artifact_path(output/'manifest.json', name)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    (output/'manifest.json').write_text(prepared.model_dump_json(indent=2)+'\n')
    return prepared


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['preview', 'prepare', 'run'], nargs='?', default='preview')
    parser.add_argument('--profile', type=Path, default=ROOT/'examples/joystick-relic-arm.json')
    parser.add_argument('--manifest', type=Path, default=ROOT/'policies/relic-exploy-standing/manifest.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=None)
    # Forward live args to the common gated CLI; never bypass it.
    args, extra = parser.parse_known_args(argv)
    if args.mode == 'run':
        from spot_deploy.cli import main as control_main
        manifest = load_manifest(args.manifest)
        if manifest.task != 'joystick-arm' or args.output is None:
            parser.error('run requires a prepared joystick manifest and a fresh --output')
        duration = manifest.teleop.duration_s if args.duration is None else args.duration
        return control_main(['joystick', '--manifest', str(args.manifest), '--output', str(args.output),
                             '--duration', str(duration), *extra])
    if extra:
        parser.error('unknown arguments: '+ ' '.join(extra))
    plan = load(args.profile, JoystickTeleopPlan)
    if args.mode == 'prepare':
        if args.output is None:
            parser.error('prepare needs --output for a new policy directory')
        prepare(args.manifest, plan, args.output)
        print(json.dumps({'manifest': str(args.output/'manifest.json'), 'hardware_qualified': False}))
        return 0
    from types import SimpleNamespace
    stow = load_manifest(args.manifest).arm_stowed_positions
    duration = plan.duration_s if args.duration is None else args.duration
    if not 0 < duration <= 3600:
        raise ContractError('preview duration must be in (0,3600] seconds')
    source = JoystickInput(plan)
    motion = TeleopMotion(SimpleNamespace(teleop=plan, arm_stowed_positions=stow), source)
    print('PREVIEW ONLY — no robot connection. Center + hold button 1; trigger 0 toggles arm; button 3 exits.', flush=True)
    print(json.dumps(source.link.metadata), flush=True)
    start, printed = time.monotonic(), -1.
    try:
        while time.monotonic()-start < duration:
            now = time.monotonic()
            motion.update(now, start-plan.initial_hold_s)
            if now-printed >= .1:
                print(json.dumps(motion.status), flush=True)
                printed = now
            if motion.completed:
                break
            time.sleep(.005)
    finally:
        source.close()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(safe_error(exc))
        raise SystemExit(1) from None
