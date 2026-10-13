"""Verify the live adapter against an existing full rehearsal, without physics or robot IO."""
import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, State
from spot_deploy.policy import rotation
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_policy import ROOT_COM_B
from spot_deploy.records import atomic_json


def verify(manifest_path, run, capture, output):
    manifest=load_manifest(manifest_path)
    policy=make_policy(manifest_path.parent/manifest.policy_file,manifest,manifest_path)
    policy.warmup()
    assert not policy.previous_action.any()
    policy.initialize_height(json.loads((capture/'initial-height.json').read_text()))
    config=json.loads((run/'configuration.json').read_text())
    if config['demonstration'] != manifest.demo.model_dump():
        raise ValueError('rehearsal plan differs from live candidate')
    events=[json.loads(s) for s in (run/'events.jsonl').read_text().splitlines()]
    reference={round((r['state']['robot_time_s']-1000)*200):r for r in events if r['event']=='policy'}
    names=config['simulation_joint_names']
    sdk_order=[names.index(n.replace('arm0_','arm_')) for n in JOINTS]
    errors=np.zeros(3)
    count=0
    peak=0.
    for line in (run/'rollout.jsonl').open():
        row=json.loads(line)
        t=row['trial_time']
        tick=round(t*200)
        state=State(robot_time_s=1000+t,received_monotonic_s=10+t,
            positions=np.asarray(row['q'])[sdk_order].tolist(),
            velocities=np.asarray(row['dq'])[sdk_order].tolist(),
            loads=row['preceding_applied_torque_Nm'],odom_quaternion_wxyz=row['root_quaternion_wxyz'],
            linear_velocity_odom=[0.]*3,angular_velocity_odom=[0.]*3,
            body_position_odom=row['position'],body_pose_robot_time_s=1000+t,
            last_command_key=tick,last_command_received_robot_s=1000+t)
        r=rotation(state)
        state.linear_velocity_odom=(r@(np.asarray(row['linear_velocity'])
            -np.cross(row['angular_velocity'],ROOT_COM_B))).tolist()
        state.angular_velocity_odom=(r@row['angular_velocity']).tolist()
        policy.update_command(state,t)
        if tick not in reference:
            continue
        expected=reference[tick]
        # Exact state retained in the inference record; sequence update uses every
        # recorded200Hz state, not only the50Hz predictions.
        inference_state=State.model_validate(expected['state'])
        prev=np.asarray(expected['observations'][-12:],dtype=np.float32)
        target, elapsed, obs, actions=policy.evaluate(inference_state,prev)
        for i,(a,b,tol) in enumerate(zip([obs[0],actions,target],
            [expected['observations'],expected['raw_actions'],expected['targets']], [1e-5,1e-4,3e-5])):
            np.testing.assert_allclose(a,b,rtol=0,atol=tol)
            errors[i]=max(errors[i],float(np.max(np.abs(a-b))))
        peak=max(peak,elapsed)
        count+=1
    assert count==len(reference)
    atomic_json(output,dict(passed=True,samples=count,
        max_errors=dict(zip(['observations','raw_actions','targets'],errors.tolist())),
        max_evaluation_s=peak,history_after_discarded_warmup_zero=True,
        physics_executed=False,hardware_access=False,hardware_qualified=False))
    print(json.dumps(dict(passed=True,samples=count,max_errors=errors.tolist())))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    verify(args.manifest,args.run,args.capture,args.output)
