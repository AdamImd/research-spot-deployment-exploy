from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from spot_deploy.contracts import ContractError, check_artifacts, load
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_policy import ACTION_IDS, DEFAULT_Q, joint_targets, observation
from spot_deploy.simulation_demo import ThreeLegPlan, ThreeLegSequence, SimulationThreeLegPolicy
from spot_deploy.walking import WalkingPlan

ROOT=Path(__file__).parents[1]


@pytest.fixture
def plan():
    return load(ROOT/'configs/relic-three-leg-front-left.json',ThreeLegPlan)


def test_latch_at_lift_not_initial_pose_and_quintic_return(state,plan):
    seq=ThreeLegSequence(plan)
    state.positions[:3]=[.1,.8,-1.4]
    np.testing.assert_array_equal(seq.update(state,0),0)
    state.positions[:3]=[.12,.85,-1.5]
    command=seq.update(state,5)
    np.testing.assert_allclose(command[:3],state.positions[:3])
    seq.update(state,7)
    np.testing.assert_allclose(seq.pose,(np.array(state.positions[:3])+plan.lifted_positions)/2)
    seq.update(state,9)
    assert seq.phase=='three-leg-hold'
    np.testing.assert_allclose(seq.pose,plan.lifted_positions)
    seq.update(state,16)
    np.testing.assert_allclose(seq.pose,(np.array(state.positions[:3])+plan.lifted_positions)/2)
    np.testing.assert_array_equal(seq.update(state,18),0)
    assert seq.leg is None and seq.phase=='four-foot-final'
    with pytest.raises(ContractError,match='backwards'):
        seq.update(state,17)


def test_three_leg_plan_only_accepts_requested_leg(plan):
    with pytest.raises(ValueError):
        ThreeLegPlan.model_validate(plan.model_dump()|{'leg':'fr'})
    with pytest.raises(ValueError):
        ThreeLegPlan.model_validate(plan.model_dump()|{'lifted_positions':[.12,float('nan'),-1.9]})


def test_selected_leg_observation_override_history_and_other_joints_match_reference(state,plan):
    path=ROOT/'policies/relic-exploy-standing/manifest.json'
    manifest=load_manifest(path)
    base=make_policy(check_artifacts(path,manifest),manifest,path)
    policy=SimulationThreeLegPolicy(base,
        ROOT/'simulation/source/relic/relic/assets/spot/pretrained/policy.onnx',plan)
    policy.initialize_height({'height_m':.52,'foot_contacts':[1]*4})
    state.positions=DEFAULT_Q.astype(float).tolist()
    previous=np.zeros(12,np.float32)
    policy.update(state,0)
    policy.update(state,5)
    policy.update(state,9)
    target,_,obs,raw=policy.evaluate(state,previous)
    np.testing.assert_array_equal(obs[0,19:22],np.array(plan.lifted_positions,dtype=np.float32))
    np.testing.assert_array_equal(obs[0,22:31],0)
    reference=observation(state,manifest.arm_stowed_positions,previous,.52)
    reference[19:31]=policy.command
    np.testing.assert_array_equal(reference,obs[0])
    expected_raw=policy.actor.session.run(['actions'],{'obs':reference[None]})[0][0]
    np.testing.assert_array_equal(raw,expected_raw)
    expected=joint_targets(raw,manifest.arm_stowed_positions).astype(float)
    expected[:3]=plan.lifted_positions
    np.testing.assert_allclose(target,expected,atol=1e-7)
    other=[i for i in ACTION_IDS if i not in [0,1,2]]
    np.testing.assert_allclose(target[other],DEFAULT_Q[other]+.2*raw[
        [ACTION_IDS.index(i) for i in other]],atol=1e-7)
    np.testing.assert_array_equal(base.previous_action,0)
    np.testing.assert_array_equal(previous,0)
    policy.update(state,18)
    target,_,obs,raw=policy.evaluate(state,previous)
    np.testing.assert_array_equal(obs[0,19:31],0)
    np.testing.assert_allclose(target[:12],joint_targets(raw,manifest.arm_stowed_positions)[:12])


def test_demo_rejects_walking_baseline(plan):
    baseline=SimpleNamespace(manifest=SimpleNamespace(task='walking'))
    with pytest.raises(ContractError,match='standing baseline'):
        SimulationThreeLegPolicy(baseline,ROOT/'missing.onnx',plan)


def test_new_plans_do_not_change_legacy_standing_graph_contract():
    from spot_deploy.exploy_policy import INPUT_SIZES
    assert len(INPUT_SIZES)==9 and 'leg_command' not in INPUT_SIZES
    manifest=load_manifest(ROOT/'policies/relic-exploy-standing/manifest.json')
    assert manifest.task=='standing' and manifest.walking is None
    assert load(ROOT/'configs/relic-walk-075.json',WalkingPlan).stall_timeout_s==5
