"""Rebuild an attribution report from recorded states; never advance physics."""
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
# When copied into records/<date>/, parent depth is the same as local/<date>/.
sys.path.insert(0, str(ROOT/'simulation'))
from spot_relic_sim.contract import Contract, Policy  # noqa: E402
from spot_deploy.contracts import JOINTS, sha256  # noqa: E402
from spot_deploy.records import verify_run  # noqa: E402

CELLS = {
    'MuJoCo baseline': 'runs/relic-physics-attribution-20261012-001/mujoco-noslip0',
    'MuJoCo no-slip10': 'runs/relic-physics-attribution-20261012-001/mujoco-noslip10',
    'MuJoCo 4 substeps': 'runs/relic-physics-attribution-20261012-001/mujoco-substeps4',
    'Isaac baseline': 'runs/relic-physics-attribution-20261012-001/isaaclab',
    'MuJoCo mesh fix': 'runs/relic-physics-mesh-fix-20261012-003/mujoco-noslip0',
    'MuJoCo mesh fix no-slip10': 'runs/relic-physics-mesh-fix-20261012-003/mujoco-noslip10',
}


def read(path):
    return json.loads(path.read_text())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def trace(path, allow_post_audit_error=False):
    data = rows(path/'rollout.jsonl')
    config = read(path/'configuration.json')
    if allow_post_audit_error:
        assert (path/'ERROR.txt').exists() and 'ModuleNotFoundError' in (path/'ERROR.txt').read_text()
        assert len(data) == 12001 and data[-1]['trial_time'] == 60
        assert len(rows(path/'timing.jsonl')) == 12001
    else:
        assert verify_run(path, status='completed')
    origin = next(e for e in rows(path/'events.jsonl') if e['event'] == 'walk_origin')
    axis = np.array([np.cos(origin['yaw_rad']), np.sin(origin['yaw_rad'])])
    forward = (np.array([r['position'] for r in data])[:, :2]-origin['odom_position_m'][:2]) @ axis
    names = [n.replace('arm_', 'arm0_') for n in config['simulation_joint_names']]
    ids = [names.index(n) for n in JOINTS[:12]]
    return data, config, forward, ids


def body_properties(first, second):
    am = {n: i for i, n in enumerate(first['body_names'])}
    bm = {n: i for i, n in enumerate(second['body_names'])}
    names = sorted(am.keys() & bm.keys())
    return dict(total_mass_MuJoCo_kg=first['total_mass'], total_mass_Isaac_kg=second['total_mass'],
        largest_link_mass_error_kg=max(abs(first['body_mass'][am[n]]-second['body_mass'][bm[n]]) for n in names),
        maximum_inertia_eigenvalue_error_kg_m2=max(float(np.max(abs(
            np.sort(first['inertia'][am[n]])
            -np.linalg.eigvalsh(np.array(second['inertia'][bm[n]]).reshape(3, 3))))) for n in names),
        maximum_COM_position_error_m=max(float(np.max(abs(np.array(first['center_of_mass'][am[n]])
            -np.array(second['center_of_mass'][bm[n]])[:3]))) for n in names),
        armature_max_error_kg_m2=float(np.max(abs(np.array(first['armature'])-second['armature']))))


def mesh_bounds(run):
    import mujoco as mj
    model = mj.MjModel.from_xml_path(str(run/'model/spot.xml'))
    result = {}
    for name in ['body', 'fl_uleg', 'fr_uleg', 'arm_link_wr1']:
        body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
        vertices = []
        for geom in np.flatnonzero((model.geom_bodyid == body) & (model.geom_contype != 0)):
            if model.geom_type[geom] != mj.mjtGeom.mjGEOM_MESH:
                continue
            mesh = model.geom_dataid[geom]
            v = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh]+model.mesh_vertnum[mesh]]
            rotation = np.zeros(9)
            mj.mju_quat2Mat(rotation, model.geom_quat[geom])
            vertices.extend(v@rotation.reshape(3, 3).T+model.geom_pos[geom])
        result[name] = dict(minimum_m=np.min(vertices, axis=0).tolist(), maximum_m=np.max(vertices, axis=0).tolist())
    return result


def first_step(mj_path, isaac_path):
    a, ca, _, ids = trace(mj_path)
    b, _, _, _ = trace(isaac_path, allow_post_audit_error='isaaclab-air' not in str(isaac_path))
    # Correct air path has a COMPLETE; native baseline completed physics but failed its auxiliary audit.
    return dict(initial_q_max_error_rad=float(np.max(abs(np.array(a[0]['q'])-b[0]['q']))),
        initial_target_max_error_rad=float(np.max(abs(np.array(a[0]['positions'])-b[0]['positions']))),
        first_step_leg_velocity_max_error_rad_s=float(np.max(abs(np.array(a[1]['dq'])[ids]-np.array(b[1]['dq'])[ids]))),
        first_step_all_joint_velocity_max_error_rad_s=float(np.max(abs(np.array(a[1]['dq'])-b[1]['dq']))),
        first_step_gripper_velocity_rad_s=[a[1]['dq'][ca['simulation_joint_names'].index('arm_f1x')],
                                         b[1]['dq'][ca['simulation_joint_names'].index('arm_f1x')]],
        first_step_foot_normal_force_w_N=[a[1]['physics_diagnostic']['foot_normal_force_w_N'],
                                       b[1]['physics_diagnostic']['foot_normal_force_w_N']])


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=ROOT/'runs/relic-physics-attribution-analysis-20261012-001')
    output = parser.parse_args().output
    output.mkdir(parents=True, exist_ok=False)
    summary = dict(hardware_access=False, hardware_qualified=False, cells={})
    plot_data = {}
    for name, relative in CELLS.items():
        path = ROOT/relative
        data, config, forward, ids = trace(path, allow_post_audit_error=name == 'Isaac baseline')
        t = np.array([r['trial_time'] for r in data])
        speeds = np.array([r['dq'] for r in data])[:, ids]
        fallen = np.array([r['height'] < .25 or r['tilt'] > .9 for r in data])
        entry = dict(recording=relative, raw_rollout_sha256=sha256(path/'rollout.jsonl'),
            config_sha256=sha256(path/'configuration.json'),
            physics_recording_status='full raw trajectory; auxiliary audit failed' if name == 'Isaac baseline' else 'completed/hash verified',
            final_forward_m=float(forward[-1]), first_fall_s=float(t[np.flatnonzero(fallen)[0]]) if fallen.any() else None,
            max_leg_speed_rad_s=float(abs(speeds).max()),
            peak_requested_leg_PD_Nm=float(abs(np.array([r['pd_torque_Nm'] for r in data])[:, :12]).max()),
            max_leg_target_gap_rad=float(abs(np.array([r['positions'] for r in data])[:, :12]
                -np.array([r['q'] for r in data])[:, ids]).max()),
            seed=config['seed'], warmup=config['prewarm_inference'],
            substeps=config['mujoco_substeps'], noslip_iterations=config['mujoco_noslip_iterations'])
        summary['cells'][name] = entry
        plot_data[name] = (t, [r['height'] for r in data], forward, np.max(abs(speeds), axis=1))
    base = ROOT/'runs/relic-physics-attribution-20261012-001'
    recovery = ROOT/'runs/relic-physics-attribution-recovery-20261012-002'
    fixed = ROOT/'runs/relic-physics-mesh-fix-20261012-003'
    summary['first_step_ground'] = first_step(base/'mujoco-noslip0', base/'isaaclab')
    summary['first_step_air'] = first_step(base/'mujoco-air', recovery/'isaaclab-air')
    summary['first_step_air_fixed_mesh'] = first_step(fixed/'mujoco-air', recovery/'isaaclab-air')
    summary['model_properties'] = body_properties(read(base/'mujoco-noslip0/physics-model.json'), read(base/'isaaclab/physics-model.json'))
    summary['collision_bounds_original'] = mesh_bounds(base/'mujoco-noslip0')
    summary['collision_bounds_fixed'] = mesh_bounds(fixed/'mujoco-noslip0')
    summary['native_actuator_audit'] = read(recovery/'native-audit/result.json')
    # At 20 ms, identical previous raw actions rule out history as the initial cause.
    a = [e for e in rows(base/'mujoco-noslip0/events.jsonl') if e['event'] == 'policy']
    b = [e for e in rows(base/'isaaclab/events.jsonl') if e['event'] == 'policy']
    oa, ob = np.array(a[1]['observations'], dtype=np.float32), np.array(b[1]['observations'], dtype=np.float32)
    policy = Policy()
    sensitivity = {}
    contract = Contract.load(ROOT/'simulation/configs/articulation.json')
    for label, indices in [('body', list(range(9))), ('q', list(range(34, 53))), ('dq', list(range(53, 72))),
        ('arm_q_dq', [34+int(i) for i in contract.arm_ids]+[53+int(i) for i in contract.arm_ids]),
        ('leg_q_dq', [34+int(i) for i in contract.leg_ids]+[53+int(i) for i in contract.leg_ids]), ('all', list(range(84)))]:
        trial = oa.copy()
        trial[indices] = ob[indices]
        sensitivity[label] = float(np.max(abs(policy(trial)-policy(ob))))
    summary['policy_input_substitution_at_20ms'] = dict(original_max_raw_action_difference=float(np.max(abs(policy(oa)-policy(ob)))),
        remaining_raw_difference=sensitivity, dimensionless=True, offline_only=True)
    summary['gripper_pair_probe'] = {}
    probe = ROOT/'runs/relic-gripper-contact-probe-20261012-004'
    for name in ['mujoco-default', 'mujoco-explicit-finger-jaw', 'isaaclab-filtered-force']:
        path = probe/name
        assert verify_run(path, status='completed')
        r = read(path/'result.json')
        joint = r['joint_names'].index('arm_f1x')
        entry = dict(rows=5, source_result_sha256=sha256(path/'result.json'),
            gripper_velocity_rad_s=[row['dq'][joint] for row in r['rows']],
            root_initial_height_m=r['rows'][0]['root_height_m'], specific_pairs=r['specific_pairs'])
        if name.startswith('isaac'):
            entry['finger_jaw_normal_force_norm_N'] = [float(np.linalg.norm(
                row['finger_jaw_normal_force_w_N'][0][0][0])) for row in r['rows'][1:]]
        summary['gripper_pair_probe'][name] = entry
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 1, figsize=(11, 10), layout='constrained')
    for name, (t, h, x, v) in plot_data.items():
        style = '--' if 'mesh fix' in name else '-'
        for axis, values in zip(axes, [h, x, v]):
            axis.plot(t, values, style, label=name, linewidth=1.3, alpha=.8)
    axes[0].set_ylabel('Body height (m)')
    axes[0].set_xlim(0, 25)
    axes[1].set_ylabel('Forward displacement (m)')
    axes[1].set_xlim(0, 60)
    axes[1].axhline(.75, color='black', linestyle=':', label='0.75 m goal')
    axes[2].set_ylabel('Maximum leg speed (rad/s)')
    axes[2].set_xlim(0, 20)
    axes[2].axhline(8, color='black', linestyle=':', label='8 rad/s knee operating bound')
    for axis in axes:
        axis.grid(alpha=.25)
        axis.set_xlabel('Simulation time (s)')
    axes[0].legend(ncol=2, fontsize=8)
    figure.suptitle('ReLIC simulator attribution: unchanged weights/gains, operating stops disabled\nContact/integration and collision conversion comparisons; no hardware')
    figure.savefig(output/'comparison.png', dpi=160)
    figure.savefig(output/'comparison.pdf')
    plt.close(figure)
    (output/'COMPLETE.json').write_text(json.dumps(dict(status='completed', hardware_access=False,
        artifacts={p.name: sha256(p) for p in output.iterdir() if p.is_file()}, tool_sha256=sha256(Path(__file__))), indent=2)+'\n')
    print(json.dumps(summary['cells'], indent=2))
    print(output)


if __name__ == '__main__':
    main()
