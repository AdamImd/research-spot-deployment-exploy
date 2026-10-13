"""Replay recorded simulator states as visual-mesh video and dynamics plots.

Kinematic replay only: never call mj_step, contact solving or a robot adapter.
Failed rollouts retain their failure label and original recorded duration.
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from spot_deploy.contracts import JOINTS, sha256
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.replay_visuals import restore_visuals, validate_visual_faces


def plots(rows, config, result, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=np.array([r['trial_time'] for r in rows])
    names=[n.replace('arm_','arm0_') for n in config['simulation_joint_names']]
    sdk_ids=[names.index(n) for n in JOINTS]
    q=np.array([r['q'] for r in rows])[:,sdk_ids]
    dq=np.array([r['dq'] for r in rows])[:,sdk_ids]
    target=np.array([r['positions'] for r in rows])
    figure,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,layout='constrained')
    axes[0,0].plot(t,[r['height'] for r in rows])
    axes[0,0].set_ylabel('Body height (m)')
    axes[0,1].plot(t,np.rad2deg([r['tilt'] for r in rows]))
    axes[0,1].set_ylabel('Tilt (degrees)')
    radius=(config.get('demonstration') or {}).get('foot_radius_m',.036)
    feet=np.array([r['feet'] for r in rows])[:,:,2]-radius
    for i,leg in enumerate(['fl','fr','hl','hr']):
        axes[1,0].plot(t,feet[:,i],label=leg)
    axes[1,0].axhline(0,color='black',linewidth=.8)
    axes[1,0].set_ylabel('Foot clearance proxy (m)')
    axes[1,0].legend(ncol=4)
    limits=config['envelope']['velocity_max'][:12]
    axes[1,1].plot(t,np.max(np.abs(dq[:,:12])/limits,axis=1),label='max speed / reviewed limit')
    axes[1,1].axhline(1,color='red',linestyle='--')
    axes[1,1].set_ylabel('Joint speed / limit')
    axes[2,0].plot(t,np.rad2deg(np.max(np.abs(target[:,:12]-q[:,:12]),axis=1)))
    axes[2,0].set_ylabel('Maximum leg target offset (degrees)')
    torque=np.array([r['pd_torque_Nm'] for r in rows])[:,:12]
    axes[2,1].plot(t,np.max(np.abs(torque),axis=1))
    axes[2,1].set_ylabel('Peak requested leg PD torque (Nm)')
    for ax in axes.flat:
        ax.grid(alpha=.25)
    axes[2,0].set_xlabel('Recorded simulation time (s)')
    axes[2,1].set_xlabel('Recorded simulation time (s)')
    figure.suptitle(f"ReLIC simulation: {config['backend']} · {result['reason']} · {config.get('guard_mode', 'enforced')}\n"
                   + ('UNLIMITED ACTUATOR TORQUE · simulation only' if config.get('actuator_torque_limits') is False
                    else 'Original gains and weights; foot clearance does not measure support load'))
    figure.savefig(output/'dynamics.png',dpi=160)
    figure.savefig(output/'dynamics.pdf')
    plt.close(figure)


def ik_plan_plots(rows, config, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1,2,figsize=(11,4),layout='constrained')
    active = [r for r in rows if r.get('ik_target_body_m') is not None]
    points = np.array([r['ik_target_body_m'] for r in active])
    axes[0].plot(points[:,0],points[:,2],label='Lift, square, return')
    path = np.array(config['demonstration']['cartesian_path']['waypoints_m'])
    axes[0].plot(path[:,0],path[:,2],'o--',label='Front workspace waypoints')
    axes[0].set(xlabel='Body-forward x (m)',ylabel='Body-relative z (m)',aspect='equal')
    for i,name in enumerate(('FL HX','FL HY','FL knee')):
        axes[1].plot([r['trial_time'] for r in rows],
                     np.rad2deg([r['q'][i] for r in rows]),label=name)
    axes[1].set(xlabel='Planned time (s)',ylabel='IK joint target (degrees)')
    for ax in axes:
        ax.grid(alpha=.25)
        ax.legend()
    figure.suptitle('Kinematic IK plan only · body/other legs fixed · no actor, physics or robot execution')
    figure.savefig(output/'ik-plan.png',dpi=150)
    figure.savefig(output/'ik-plan.pdf')
    plt.close(figure)


def video(rows,config,result,source,output,fps):
    import mujoco as mj
    from PIL import Image,ImageDraw,ImageFont
    sys.path.insert(0,str(source.resolve()))
    from spot_relic_sim.mujoco_backend import convert_model
    from spot_relic_sim.contract import ASSET
    canonical=convert_model(output/'model')
    restored=restore_visuals(canonical, ASSET/'spot_with_arm.urdf', output/'visuals')
    model=mj.MjModel.from_xml_path(str(restored))
    validate_visual_faces(model, output/'visuals/visual-manifest.json')
    floor=mj.mj_name2id(model,mj.mjtObj.mjOBJ_GEOM,'floor')
    for i in range(model.ngeom):
        if model.geom_contype[i] and i!=floor:
            model.geom_rgba[i,3]=0
    model.vis.global_.offwidth=720
    model.vis.global_.offheight=480
    data=mj.MjData(model)
    names=config['simulation_joint_names']
    # Planned trajectories use canonical SDK arm0_* names; the native mesh
    # replay URDF uses arm_*. Dynamics recordings already use native names.
    ids=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,n.replace('arm0_','arm_',1)) for n in names]
    if min(ids)<0:
        raise ValueError('Replay model joint mismatch')
    qids=model.jnt_qposadr[ids]
    camera=mj.MjvCamera()
    camera.azimuth,camera.elevation,camera.distance=125,-18,2.25
    renderer=mj.Renderer(model,height=480,width=720)
    font=ImageFont.load_default(size=18)
    times=np.array([r['trial_time'] for r in rows])
    frames=np.arange(times[0],times[-1]+1e-9,1/fps)
    command=['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-filter_threads','1',
             '-f','rawvideo','-pix_fmt','rgb24',
             '-s','720x560','-r',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast',
             '-threads','2','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',
             str(output/'replay.mp4')]
    try:
        with (output/'ffmpeg.log').open('wb') as log:
            process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=log)
            try:
                for t in frames:
                    index=int(np.argmin(np.abs(times-t)))
                    if abs(times[index]-t)>.0026:
                        raise ValueError('Video sample cannot be aligned to a recorded physics tick')
                    row=rows[index]
                    data.qpos[:3]=row['position']
                    data.qpos[3:7]=row['root_quaternion_wxyz']
                    data.qpos[qids]=row['q']
                    data.qvel[:]=0
                    mj.mj_forward(model,data)
                    camera.lookat[:]=[row['position'][0],row['position'][1],.4]
                    renderer.update_scene(data,camera=camera)
                    target = row.get('ik_target_body_m')
                    if target is not None:
                        from spot_deploy.policy import rotation
                        from types import SimpleNamespace
                        body_rotation = rotation(SimpleNamespace(
                            odom_quaternion_wxyz=row['root_quaternion_wxyz']))
                        marker = renderer.scene.geoms[renderer.scene.ngeom]
                        mj.mjv_initGeom(marker,mj.mjtGeom.mjGEOM_SPHERE,
                            np.array([.012,.012,.012]),
                            np.array(row['position'])+body_rotation@target,
                            np.eye(3).flatten(),np.array([0.,1.,1.,.8]))
                        renderer.scene.ngeom += 1
                    canvas=Image.new('RGB',(720,560),(15,22,31))
                    canvas.paste(Image.fromarray(renderer.render()),(0,48))
                    draw=ImageDraw.Draw(canvas)
                    phase=row.get('demo_phase') or row['phase']
                    draw.text((12,7),f"{config['backend']} · {phase} · t={t:.2f} s",font=font,fill='white')
                    subtitle = ('UNGUARDED simulation diagnostic' if config.get('guard_mode') == 'disabled-simulation-diagnostic'
                                else 'Recorded visual-mesh replay · simulation only')
                    if config.get('kinematic_plan_only'):
                        subtitle = 'IK plan · body/support legs fixed · no RL or dynamics'
                    if config.get('actuator_torque_limits') is False:
                        subtitle = 'UNLIMITED ACTUATOR TORQUE · simulation only'
                    if config.get('simulation_forward_speed_m_s') is not None:
                        subtitle = (f"UNGUARDED · {config['simulation_forward_speed_m_s']:.2f} m/s max request"
                                    f" · {config['simulation_speed_profile']}")
                    draw.text((12,30),subtitle,font=font,fill='white')
                    draw.text((12,533),f"Run outcome: {result['reason']} · no new dynamics",font=font,fill='white')
                    process.stdin.write(np.asarray(canvas).tobytes())
                process.stdin.close()
                if process.wait(timeout=30):
                    raise RuntimeError('Video encoder failed')
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=10)
    finally:
        renderer.close()
    return len(frames)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--simulation-source',type=Path,default=Path('simulation'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fps',type=int,choices=[25,50,100,200],default=50)
    args=parser.parse_args()
    if not verify_run(args.run,status='completed'):
        parser.error('Source recording failed integrity verification')
    config=json.loads((args.run/'configuration.json').read_text())
    result=json.loads((args.run/'result.json').read_text())
    rows=[json.loads(line) for line in (args.run/'rollout.jsonl').read_text().splitlines()]
    if not rows:
        parser.error('No recorded states to render')
    args.output.mkdir(parents=True,exist_ok=False)
    if config.get('kinematic_plan_only'):
        ik_plan_plots(rows,config,args.output)
    else:
        plots(rows,config,result,args.output)
    count=video(rows,config,result,args.simulation_source,args.output,args.fps)
    probe=subprocess.run(['ffprobe','-v','error','-show_streams','-of','json',
                          str(args.output/'replay.mp4')],capture_output=True,text=True,check=True)
    stream=json.loads(probe.stdout)['streams'][0]
    if int(stream['nb_frames'])!=count:
        raise ValueError('Encoded frame count mismatch')
    atomic_json(args.output/'COMPLETE.json',dict(status='completed',frames=count,fps=args.fps,
        recorded_duration_s=rows[-1]['trial_time'],source_reason=result['reason'],
        source_complete_sha256=sha256(args.run/'COMPLETE.json'),tool_sha256=sha256(Path(__file__)),
        source_trajectory_sha256=sha256(args.run/'rollout.jsonl'),
        visual_manifest_sha256=sha256(args.output/'visuals/visual-manifest.json'),
        visual_model_sha256=sha256(args.output/'visuals/replay.xml'),
        visual_restoration_tool_sha256=sha256(Path(__file__).resolve().parents[1]
                                             /'src/spot_deploy/replay_visuals.py'),
        new_dynamics=False,hardware_access=False,
        artifacts={p.name:sha256(p) for p in args.output.glob('*') if p.is_file()}))
    print(json.dumps(dict(output=str(args.output),frames=count,source_reason=result['reason'])))


if __name__=='__main__':
    main()
