"""Generate an IK motion plan for visual review; no actor/physics/robot execution."""
import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import State, load, sha256
from spot_deploy.leg_ik import LegPositionIK
from spot_deploy.records import RunRecord, atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest
from spot_deploy.simulation_demo import ThreeLegPlan, ThreeLegSequence
from spot_deploy.static_support import StaticSupport
from spot_deploy.contracts import JOINTS, artifact_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not verify_run(args.capture):
        raise ValueError('capture integrity failure')
    plan = load(args.plan, ThreeLegPlan)
    if not plan.cartesian_path:
        raise ValueError('this preparation tool requires the Cartesian IK plan')
    manifest = load_manifest(args.manifest)
    path = artifact_path(args.manifest, manifest.relic.preparation.model_file)
    if sha256(path) != manifest.relic.preparation.model_sha256:
        raise ValueError('pinned kinematic model mismatch')
    prep = manifest.relic.preparation
    support = StaticSupport(path, prep.foot_radius_m, prep.friction_coefficient)
    models = [LegPositionIK(support, path, leg) for leg in ('fl','fr','hl','hr')]
    sequence = ThreeLegSequence(plan, models[0])
    state = State.model_validate(json.loads((args.capture/'initial-state.json').read_text()))
    initial = np.array(state.positions)
    initial[12:] = manifest.arm_stowed_positions
    body_z = plan.foot_radius_m-min(
        model.forward(initial[model.ids])[0][2] for model in models)
    record = RunRecord(args.output, 'kinematic-ik-plan',
                       [args.plan,args.manifest,path,args.capture/'COMPLETE.json'])
    config = dict(backend='kinematic-plan', kinematic_plan_only=True,
        actor_executed=False, physics_executed=False, hardware_access=False,
        other_legs='fixed captured standing joint pose, no claimed RL balance',
        frame='body: x forward, y left, z up; targets follow current body orientation',
        visual_body_height_m=body_z, simulation_joint_names=list(JOINTS),
        demonstration=plan.model_dump(), model_sha256=sha256(path))
    atomic_json(args.output/'configuration.json',config)
    last_q, rows = initial.copy(), []
    try:
        with (args.output/'rollout.jsonl').open('x') as stream:
            for tick in range(round(plan.duration_s/.005)+1):
                t = tick*.005
                sequence.update(state,t)
                q = initial.copy()
                if sequence.leg:
                    q[sequence.ids] = sequence.pose
                feet = [model.forward(q[model.ids])[0]+[0,0,body_z] for model in models]
                row = dict(trial_time=t, phase='kinematic-plan',demo_phase=sequence.phase,
                    position=[0,0,body_z],root_quaternion_wxyz=[1,0,0,0],
                    q=q.tolist(),dq=((q-last_q)/.005).tolist(),feet=np.array(feet).tolist(),
                    ik_target_body_m=(sequence.target_body_m.tolist()
                                      if sequence.target_body_m is not None else None),
                    ik_solve_residual_m=sequence.ik_residual_m if sequence.leg else None)
                rows.append(row)
                stream.write(json.dumps(row)+'\n')
                last_q=q
        residuals=[r['ik_solve_residual_m'] for r in rows if r['ik_solve_residual_m'] is not None]
        result=dict(reason='kinematic-plan-only', duration_s=plan.duration_s, frames=len(rows),
            max_ik_solve_residual_m=max(residuals),
            selected_leg_peak_planned_speed_rad_s=float(np.max(np.abs(
                np.array([r['dq'][:3] for r in rows])))),
            model_sha256=sha256(path),hardware_access=False,hardware_qualified=False,
            actor_executed=False,physics_executed=False,
            limitations='Planned joint kinematics only: no balance, contact-load, collision or robustness qualification.')
        record.finish('completed',result)
        print(json.dumps(result))
    except BaseException as exc:
        record.finish('failed',dict(reason='IK planning failed',error_type=type(exc).__name__))
        raise


if __name__ == '__main__':
    main()
