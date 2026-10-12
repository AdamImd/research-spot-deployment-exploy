"""Inspect native model and compare native versus NumPy actuator torques offline.

No policy rollout or SDK access. Requires the Isaac-compatible Python overlay
plus the locked extra native-physics-audit requirements.
"""

import argparse
import json
from pathlib import Path
import sys

from spot_deploy.contracts import sha256
from spot_deploy.records import atomic_json, runtime_packages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path[:0] = [str(args.simulation_source), str(args.simulation_source/'tools')]
    app = backend = None
    try:
        from isaaclab.app import AppLauncher
        app = AppLauncher(headless=True, device='cpu',
            kit_args='--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0').app
        from spot_relic_sim.isaac_backend import IsaacBackend
        from spot_relic_sim.native_audit import audit_actuators
        from spot_relic_sim.physics_diagnostics import model_snapshot
        from spot_relic_sim.scenarios import Scenario
        backend = IsaacBackend(args.output/'native-model',
            Scenario(name='actuator-audit', duration=.02, smoke=True, seed=101))
        atomic_json(args.output/'physics-model.json', model_snapshot(backend))
        result = audit_actuators(backend, args.output/'numpy-model')
        atomic_json(args.output/'result.json', result)
        atomic_json(args.output/'source.json', dict(hardware_access=False, physics_rollout=False,
            packages=runtime_packages(), tool_sha256=sha256(Path(__file__)),
            source_hashes={str(p): sha256(p) for p in sorted(
                (args.simulation_source/'spot_relic_sim').glob('*.py'))}))
        atomic_json(args.output/'COMPLETE.json', dict(status='completed', hardware_access=False,
            artifacts={str(p.relative_to(args.output)): sha256(p)
                       for p in args.output.iterdir() if p.is_file()}))
        print(json.dumps(result), flush=True)
    except BaseException:
        import traceback
        (args.output/'ERROR.txt').write_text(traceback.format_exc())
        raise
    finally:
        if backend:
            backend.close()
        if app:
            app.close(skip_cleanup=True)


if __name__ == '__main__':
    main()
