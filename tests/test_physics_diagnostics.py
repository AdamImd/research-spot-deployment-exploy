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
