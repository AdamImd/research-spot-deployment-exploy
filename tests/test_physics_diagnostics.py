import numpy as np
import pytest
from pathlib import Path


def backend(tmp_path, monkeypatch):
    pytest.importorskip('mujoco')
    root = Path(__file__).parents[1]
    monkeypatch.syspath_prepend(str(root/'simulation'))
    from spot_relic_sim.contract import Contract
    from spot_relic_sim.mujoco_backend import MujocoBackend
    from spot_relic_sim.scenarios import Scenario
    return MujocoBackend(Contract.load(root/'simulation/configs/articulation.json'), tmp_path,
                         Scenario(name='probe', duration=1, smoke=True, seed=101))


def test_instrumentation_does_not_change_physics_state_or_standard_step(tmp_path, monkeypatch):
    first = backend(tmp_path/'first', monkeypatch)
    second = backend(tmp_path/'second', monkeypatch)
    from spot_relic_sim.physics_diagnostics import contacts, control_step, model_snapshot
    target = first.data.qpos[first.qids].copy()
    target[1] += .05
    before = first.data.qpos.copy()
    snapshot = model_snapshot(first)
    report = contacts(first)
    assert snapshot['solver']['timestep'] == .005
    assert np.array(report['foot_normal_force_w_N']).shape == (4, 3)
    np.testing.assert_array_equal(first.data.qpos, before)
    first.step(target)
    control_step(second, target)
    np.testing.assert_array_equal(first.data.qpos, second.data.qpos)
    np.testing.assert_array_equal(first.data.qvel, second.data.qvel)


def test_substeps_hold_pd_and_advance_delay_history_only_once(tmp_path, monkeypatch):
    instance = backend(tmp_path, monkeypatch)
    from spot_relic_sim.physics_diagnostics import control_step
    instance.model.opt.timestep = .005/4
    count = 0
    torque = instance.torque
    def counted(target):
        nonlocal count
        count += 1
        return torque(target)
    instance.torque = counted
    control_step(instance, instance.data.qpos[instance.qids].copy(), substeps=4)
    assert count == 1 and instance.data.time == pytest.approx(.005)
    with pytest.raises(ValueError, match='1 or 4'):
        control_step(instance, np.zeros(19), substeps=2)


def test_collision_assets_do_not_alias_visual_meshes_and_match_source_bounds(tmp_path, monkeypatch):
    instance = backend(tmp_path, monkeypatch)
    import mujoco as mj
    import xml.etree.ElementTree as ET
    from spot_relic_sim.contract import ASSET
    model = instance.model
    urdf = ET.parse(ASSET/'spot_with_arm.urdf')
    collision_checks = 0
    for link in urdf.findall('link'):
        collisions = [c for c in link.findall('collision') if c.find('geometry/mesh') is not None]
        meshes = [c.find('geometry/mesh') for c in collisions]
        if not meshes:
            continue
        expected = []
        for collision, mesh in zip(collisions, meshes):
            source = ASSET/mesh.attrib['filename']
            positions, referenced = [], []
            for line in source.read_text().splitlines():
                if line.startswith('v '):
                    positions.append([float(x) for x in line.split()[1:4]])
                elif line.startswith('f '):
                    referenced.extend(int(x.split('/')[0])-1 for x in line.split()[1:])
            from spot_deploy.replay_visuals import rpy_quaternion
            origin = collision.find('origin')
            rpy = origin.get('rpy', '0 0 0') if origin is not None else '0 0 0'
            xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
            scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            rotation = np.zeros(9)
            mj.mju_quat2Mat(rotation, rpy_quaternion(np.fromstring(rpy, sep=' ')))
            vertices = np.array(positions)[np.unique(referenced)] * scale
            expected.extend(vertices@rotation.reshape(3, 3).T+np.fromstring(xyz, sep=' '))
        body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, link.attrib['name'])
        actual = []
        for geom in np.flatnonzero((model.geom_bodyid==body)&(model.geom_contype!=0)):
            if model.geom_type[geom] != mj.mjtGeom.mjGEOM_MESH:
                continue
            mesh = model.geom_dataid[geom]
            assert mj.mj_id2name(model, mj.mjtObj.mjOBJ_MESH, mesh).startswith('collision__')
            vertices = model.mesh_vert[model.mesh_vertadr[mesh]:
                                       model.mesh_vertadr[mesh]+model.mesh_vertnum[mesh]]
            rotation = np.zeros(9)
            mj.mju_quat2Mat(rotation, model.geom_quat[geom])
            actual.extend(vertices@rotation.reshape(3, 3).T+model.geom_pos[geom])
        np.testing.assert_allclose(np.min(actual, axis=0), np.min(expected, axis=0), atol=1e-6, rtol=0)
        np.testing.assert_allclose(np.max(actual, axis=0), np.max(expected, axis=0), atol=1e-6, rtol=0)
        collision_checks += 1
    assert collision_checks >= 14


def test_unlimited_torque_bypasses_all_clipping_but_retains_integrity(tmp_path, monkeypatch):
    instance = backend(tmp_path, monkeypatch)
    q = instance.data.qpos[instance.qids].copy()
    instance.data.qvel[instance.dids] = -20.
    target = q + 5.
    requested = instance.kp * 5. + instance.kd * 20.
    bounded = instance.torque(target)
    assert np.all(np.abs(bounded) <= np.abs(requested))
    assert np.all(bounded[instance.knees] != requested[instance.knees])
    instance.torque_limits = False
    np.testing.assert_allclose(instance.torque(target), requested, rtol=0, atol=1e-10)
    assert instance.metadata()['actuator_torque_limits'] is False
    with pytest.raises(ValueError, match='nineteen finite'):
        instance.torque(np.full(19, np.nan))
    instance.data.qvel[instance.dids[0]] = np.inf
    with pytest.raises(ValueError, match='nonfinite actuator'):
        instance.torque(target)


@pytest.mark.parametrize('extra', [[], ['--no-guards', '--backend', 'isaaclab']])
def test_unlimited_torque_requires_explicit_mujoco_diagnostic(extra, tmp_path):
    import subprocess
    import sys
    root = Path(__file__).parents[1]
    command = [sys.executable, str(root/'tools/simulate_relic.py'),
               '--simulation-source', str(root/'simulation'), '--capture', str(tmp_path),
               '--manifest', 'unused', '--envelope', 'unused', '--backend', 'mujoco',
               '--output', str(tmp_path/'output'), '--no-actuator-torque-limits', *extra]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 2
    assert 'unlimited actuator torque requires MuJoCo and --no-guards' in result.stderr
    assert not (tmp_path/'output').exists()


@pytest.mark.parametrize('limited', [True, False])
def test_dynamics_plot_labels_both_actuator_modes(tmp_path, limited):
    import runpy
    from spot_deploy.contracts import JOINTS
    root = Path(__file__).parents[1]
    plots = runpy.run_path(str(root/'tools/render_relic_demo.py'))['plots']
    rows = [dict(trial_time=t, q=[0.]*19, dq=[0.]*19, positions=[0.]*19,
                 height=.5, tilt=0., feet=[[0., 0., .036]]*4, pd_torque_Nm=[0.]*19)
            for t in [0., .005]]
    config = dict(backend='mujoco', simulation_joint_names=list(JOINTS),
                  envelope={'velocity_max': [1.]*19}, actuator_torque_limits=limited)
    plots(rows, config, {'reason': 'completed'}, tmp_path)
    assert (tmp_path/'dynamics.png').stat().st_size > 1000
    assert (tmp_path/'dynamics.pdf').stat().st_size > 1000


def test_torque_analysis_accounts_for_float32_state_without_hiding_clipping():
    import runpy
    root = Path(__file__).parents[1]
    bound_fn = runpy.run_path(str(root/'records/relic-three-leg-native-recovery-20261013/analyze.py'))['pd_roundoff_bound']
    rng = np.random.default_rng(101)
    q, dq = rng.uniform(-3., 3., (2, 1000, 19))
    logged_q, logged_dq = q.astype(np.float32), dq.astype(np.float32)
    kp, kd = np.full(19, 120.), np.full(19, 5.)
    target = np.zeros(19)
    demand = kp*(target-logged_q)-kd*logged_dq
    applied = kp*(target-q)-kd*dq
    bound = bound_fn(logged_q, logged_dq, kp, kd)
    assert np.all(np.abs(demand-applied) <= bound+1e-12)
    assert not np.any(np.abs(demand-applied) > bound+1e-5)
    applied[0, 0] = demand[0, 0]-.01
    assert np.abs(demand[0, 0]-applied[0, 0]) > bound[0, 0]+1e-5
