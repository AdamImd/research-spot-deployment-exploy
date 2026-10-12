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
