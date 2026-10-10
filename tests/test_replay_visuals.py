import json
from pathlib import Path

import numpy as np
import pytest

from spot_deploy.contracts import ContractError
from spot_deploy.replay_visuals import restore_visuals, validate_visual_faces


def test_authored_material_faces_survive_without_changing_kinematics_or_physics(tmp_path, monkeypatch):
    mj = pytest.importorskip('mujoco')
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'simulation'))
    from spot_relic_sim.mujoco_backend import convert_model
    from spot_relic_sim.contract import ASSET
    canonical = convert_model(tmp_path/'canonical')
    original_bytes = canonical.read_bytes()
    baseline = mj.MjModel.from_xml_path(str(canonical))
    restored = restore_visuals(canonical, ASSET/'spot_with_arm.urdf', tmp_path/'visuals')
    model = mj.MjModel.from_xml_path(str(restored))
    manifest_path = restored.with_name('visual-manifest.json')
    validate_visual_faces(model, manifest_path)
    manifest = json.loads(manifest_path.read_text())
    body = [group for group in manifest['material_groups'] if group['link'] == 'body']
    assert sum(group['triangles'] for group in body) == 33207
    assert {group['material'] for group in body} == {'BlackAbs', 'wrap'}
    assert len(manifest['material_groups']) > 20
    for group in body:
        material = mj.mj_name2id(model, mj.mjtObj.mjOBJ_MATERIAL, group['name'])
        if group['material'] == 'BlackAbs':
            np.testing.assert_array_equal(model.mat_rgba[material], [0, 0, 0, 1])
        else:
            assert group['textured'] and np.any(model.mat_texid[material] >= 0)
    assert model.ntex > 0 and canonical.read_bytes() == original_bytes
    for field in ('nq', 'nv', 'nbody', 'njnt'):
        assert getattr(baseline, field) == getattr(model, field)
    for field in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_pos',
                  'body_quat', 'jnt_pos', 'jnt_axis', 'jnt_range', 'qpos0', 'dof_armature'):
        np.testing.assert_allclose(getattr(model, field), getattr(baseline, field), atol=1e-12)
    for field in ('geom_type', 'geom_friction', 'geom_size', 'geom_pos', 'geom_quat'):
        np.testing.assert_allclose(getattr(model, field)[model.geom_contype != 0],
                                   getattr(baseline, field)[baseline.geom_contype != 0], atol=1e-12)
    # The same replay pose must give identical articulated link frames.
    before, after = mj.MjData(baseline), mj.MjData(model)
    for data in (before, after):
        data.qpos[2] = .52
        data.qpos[7:] = np.linspace(-.15, .15, model.nq-7)
    mj.mj_forward(baseline, before)
    mj.mj_forward(model, after)
    np.testing.assert_allclose(after.xpos, before.xpos, atol=1e-12)
    np.testing.assert_allclose(after.xquat, before.xquat, atol=1e-12)
    manifest['material_groups'][0]['triangles'] += 1
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ContractError, match='lost authored visual triangles'):
        validate_visual_faces(model, manifest_path)
