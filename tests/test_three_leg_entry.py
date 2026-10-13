from pathlib import Path
import numpy as np

from spot_deploy.contracts import load,check_artifacts
from spot_deploy.leg_ik import LegPositionIK
from spot_deploy.relic_contract import load_manifest,make_policy
from spot_deploy.relic_policy import DEFAULT_Q
from spot_deploy.simulation_demo import ThreeLegPlan,ThreeLegSequence,SimulationThreeLegPolicy

ROOT=Path(__file__).parents[1]


def baseline():
    path=ROOT/'policies/relic-exploy-standing/manifest.json'
    manifest=load_manifest(path)
    return make_policy(check_artifacts(path,manifest),manifest,path)


def test_entry_is_continuous_with_four_foot_command_and_keeps_history_input(state):
    state.positions=DEFAULT_Q.astype(float).tolist()
    base=baseline()
    plan=load(ROOT/'configs/relic-three-leg-front-left-ik.json',ThreeLegPlan)
    policy=SimulationThreeLegPolicy(base,ROOT/'simulation/source/relic/relic/assets/spot/pretrained/policy.onnx',plan)
    policy.initialize_height({'height_m':.52,'foot_contacts':[1]*4})
    previous=np.linspace(-.2,.3,12,dtype=np.float32)
    original=previous.copy()
    policy.update(state,plan.initial_hold_s-1e-6)
    before=policy.evaluate(state,previous)
    policy.update(state,plan.initial_hold_s)
    entered=policy.evaluate(state,previous)
    np.testing.assert_array_equal(entered[0],before[0])
    np.testing.assert_array_equal(entered[3],before[3])
    np.testing.assert_array_equal(previous,original)
    assert policy.sequence.phase=='three-leg-handover'
    assert policy.sequence.entry_alpha==0


def test_entry_unloads_foot_continuously_while_mode_blends(state):
    state.positions=DEFAULT_Q.astype(float).tolist()
    base=baseline()
    plan=load(ROOT/'configs/relic-three-leg-front-left-ik.json',ThreeLegPlan)
    ik=LegPositionIK(base.support,base.support_model,'fl')
    seq=ThreeLegSequence(plan,ik)
    start=ik.forward(state.positions[:3])[0]
    seq.update(state,plan.initial_hold_s)
    np.testing.assert_allclose(seq.target_body_m,start,rtol=0,atol=1e-12)
    previous=start.copy()
    for t in np.arange(plan.initial_hold_s+.005,plan.initial_hold_s+plan.entry_blend_s+.0025,.005):
        seq.update(state,t)
        assert np.linalg.norm(seq.target_body_m-previous)<.001
        previous=seq.target_body_m.copy()
    assert seq.target_body_m[2]>start[2]
    seq.update(state,plan.initial_hold_s+plan.entry_blend_s)
    assert seq.phase=='lifting'
    seq.update(state,plan.initial_hold_s+plan.entry_blend_s+1)
    assert np.linalg.norm(seq.target_body_m-start)>0
    assert plan.duration_s==46
