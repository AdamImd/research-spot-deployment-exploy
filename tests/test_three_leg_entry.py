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
    policy.update(state,(plan.initial_hold_s+plan.support_transfer_s)-1e-6)
    before=policy.evaluate(state,previous)
    policy.update(state,(plan.initial_hold_s+plan.support_transfer_s))
    entered=policy.evaluate(state,previous)
    np.testing.assert_array_equal(entered[0],before[0])
    # Raw network output stays native; only emitted targets are slew limited.
    np.testing.assert_array_equal(entered[3],policy.actor.session.run(['actions'],{'obs':entered[2]})[0][0])
    np.testing.assert_array_equal(previous,original)
    assert policy.sequence.phase=='lifting'


def test_slow_lift_path_remains_continuous_during_entry(state):
    state.positions=DEFAULT_Q.astype(float).tolist()
    base=baseline()
    plan=load(ROOT/'configs/relic-three-leg-front-left-ik.json',ThreeLegPlan)
    ik=LegPositionIK(base.support,base.support_model,'fl')
    seq=ThreeLegSequence(plan,ik)
    start=ik.forward(state.positions[:3])[0]
    seq.update(state,(plan.initial_hold_s+plan.support_transfer_s))
    np.testing.assert_allclose(seq.target_body_m,start,rtol=0,atol=1e-12)
    previous=start.copy()
    for t in (plan.initial_hold_s+plan.support_transfer_s)+np.arange(1,round(2*200)+1)/200:
        seq.update(state,t)
        assert np.linalg.norm(seq.target_body_m-previous)<.001
        previous=seq.target_body_m.copy()
    assert seq.target_body_m[2]>start[2]
    seq.update(state,(plan.initial_hold_s+plan.support_transfer_s)+2)
    assert seq.phase=='lifting'
    assert np.linalg.norm(seq.target_body_m-start)>0
    assert plan.duration_s==46


def test_limiter_bounds_steps_and_does_not_catch_up_after_gap():
    from types import SimpleNamespace
    from spot_deploy.simulation_demo import DemoTargetSlew
    limiter=DemoTargetSlew()
    sequence=SimpleNamespace(last_time=4.98,plan=SimpleNamespace(initial_hold_s=5.,support_transfer_s=0.,target_slew_rad_s=1.))
    limiter.apply(sequence,np.zeros(19))
    sequence.last_time=5.
    np.testing.assert_array_equal(limiter.apply(sequence,np.ones(19)*100)[:12],0)
    sequence.last_time=5.02
    np.testing.assert_allclose(limiter.apply(sequence,np.ones(19)*100)[:12],.02,atol=1e-12)
    sequence.last_time=100.
    np.testing.assert_allclose(limiter.apply(sequence,np.ones(19)*100)[:12],.04,atol=1e-12)


def test_unfiltered_entry_preserves_actor_proposals_and_raw_history(state):
    state.positions=DEFAULT_Q.astype(float).tolist()
    base=baseline()
    plan=load(ROOT/'configs/relic-three-leg-front-left-ik-unfiltered.json',ThreeLegPlan)
    assert plan.target_slew_rad_s is None
    policy=SimulationThreeLegPolicy(base,ROOT/'simulation/source/relic/relic/assets/spot/pretrained/policy.onnx',plan)
    policy.initialize_height({'height_m':.52,'foot_contacts':[1]*4})
    previous=np.linspace(-.2,.3,12,dtype=np.float32)
    original=previous.copy()
    policy.update(state,4.98)
    before=policy.evaluate(state,previous)
    policy.update(state,5.)
    entered=policy.evaluate(state,previous)
    np.testing.assert_array_equal(entered[0],policy.proposed_targets)
    np.testing.assert_allclose(entered[0][:3],state.positions[:3],atol=1e-6)
    assert np.max(np.abs(entered[0][:12]-before[0][:12]))>.02
    np.testing.assert_array_equal(entered[3],policy.actor.session.run(['actions'],{'obs':entered[2]})[0][0])
    np.testing.assert_array_equal(previous,original)
