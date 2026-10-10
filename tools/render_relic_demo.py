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
    figure.suptitle(f"ReLIC simulation: {config['backend']} · {result['reason']}\n"
                   'Original gains and weights; foot clearance does not measure support load')
    figure.savefig(output/'dynamics.png',dpi=160)
    figure.savefig(output/'dynamics.pdf')
    plt.close(figure)


def video(rows,config,result,source,output,fps):
    import mujoco as mj
    from PIL import Image,ImageDraw,ImageFont
    sys.path.insert(0,str(source.resolve()))
    from spot_relic_sim.mujoco_backend import convert_model
    model=mj.MjModel.from_xml_path(str(convert_model(output/'model')))
    floor=mj.mj_name2id(model,mj.mjtObj.mjOBJ_GEOM,'floor')
    for i in range(model.ngeom):
        if model.geom_contype[i] and i!=floor:
            model.geom_rgba[i,3]=0
    model.vis.global_.offwidth=720
    model.vis.global_.offheight=480
    data=mj.MjData(model)
    names=config['simulation_joint_names']
    ids=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT,n) for n in names]
    if min(ids)<0:
        raise ValueError('Replay model joint mismatch')
    qids=model.jnt_qposadr[ids]
    camera=mj.MjvCamera()
    camera.azimuth,camera.elevation,camera.distance=125,-18,2.25
    renderer=mj.Renderer(model,height=480,width=720)
    font=ImageFont.load_default(size=18)
    times=np.array([r['trial_time'] for r in rows])
    frames=np.arange(times[0],times[-1]+1e-9,1/fps)
    command=['ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
             '-s','720x560','-r',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast',
             '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output/'replay.mp4')]
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
                    canvas=Image.new('RGB',(720,560),(15,22,31))
                    canvas.paste(Image.fromarray(renderer.render()),(0,48))
                    draw=ImageDraw.Draw(canvas)
                    phase=row.get('demo_phase') or row['phase']
                    draw.text((12,7),f"{config['backend']} · {phase} · t={t:.2f} s",font=font,fill='white')
                    draw.text((12,30),'Recorded visual-mesh replay · simulation only',font=font,fill='white')
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
        new_dynamics=False,hardware_access=False,
        artifacts={p.name:sha256(p) for p in args.output.glob('*') if p.is_file()}))
    print(json.dumps(dict(output=str(args.output),frames=count,source_reason=result['reason'])))


if __name__=='__main__':
    main()
