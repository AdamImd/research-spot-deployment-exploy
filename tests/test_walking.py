import json
import math
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.contracts import ContractError, State, check_artifacts, load
from spot_deploy.exploy_policy import ExployReLICManifest
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_policy import observation
from spot_deploy.walking import ForwardTravel, WalkingPlan
from test_control_loop import Log

ROOT = Path(__file__).parents[1]


@pytest.fixture
def plan():
    return load(ROOT / 'configs/relic-walk-075.json', WalkingPlan)


def posed(state, t, x=0., y=0., yaw=0., speed=0.):
    return state.model_copy(update=dict(robot_time_s=1000+t, received_monotonic_s=10+t,
        body_position_odom=[10+x, 20+y, .52], body_pose_robot_time_s=1000+t,
        odom_quaternion_wxyz=[math.cos(yaw/2), 0, 0, math.sin(yaw/2)],
        linear_velocity_odom=[speed*math.cos(yaw), speed*math.sin(yaw), 0]))


def test_forward_distance_uses_frozen_initial_heading_not_world_x(state, plan):
    travel = ForwardTravel(plan, Log())
    travel.latch(posed(state, 0, yaw=math.pi/2))
    travel.activate(10)
    travel.update(posed(state, .1, x=-.005, y=.015, yaw=math.pi/2), 10.1)
    assert travel.progress['forward_m'] == pytest.approx(.015)
    assert travel.progress['lateral_m'] == pytest.approx(.005)
    assert travel.progress['requested_forward_m_s'] == 0


def test_measured_distance_and_stationary_hold_complete_without_time_distance(state, plan):
    log = Log()
    travel = ForwardTravel(plan, log)
    travel.latch(posed(state, 0))
    travel.activate(10)
    x, velocity = 0., 0.
    for tick in range(1, 4001):
        t = tick * .005
        x += velocity * .005
        velocity = travel.update(posed(state, t, x=x, speed=velocity), 10+t)[0]
        if travel.completed:
            break
    assert travel.completed and .73 <= x <= .77
    assert velocity == 0 and 8 < t < 20
    assert log.events[0][0] == 'walk_origin'
    assert travel.progress['phase'] == 'completed'


def test_no_motion_times_out_instead_of_claiming_distance(state, plan):
    travel = ForwardTravel(plan, Log())
    travel.latch(posed(state, 0))
    travel.activate(10)
    with pytest.raises(ContractError, match='timeout'):
        travel.update(posed(state, 20.01), 30.01)
    assert not travel.completed


@pytest.mark.parametrize('field,value', [('body_position_odom', None),
    ('body_pose_robot_time_s', None), ('body_pose_robot_time_s', 999.)])
def test_missing_or_stale_robot_pose_fails_closed(state, plan, field, value):
    travel = ForwardTravel(plan, Log())
    with pytest.raises(ContractError):
        travel.latch(posed(state, 0).model_copy(update={field: value}))


def test_pose_timestamp_replay_jump_and_heading_limits(state, plan):
    for change, match in [({'body_pose_robot_time_s':999.999}, 'backwards'),
                          ({'body_position_odom':[11.,20.,.52]}, 'discontinuity'),
                          ({'odom_quaternion_wxyz':[math.cos(.1),0,0,math.sin(.1)]}, 'heading')]:
        travel = ForwardTravel(plan, Log())
        travel.latch(posed(state, 0))
        travel.activate(10)
        with pytest.raises(ContractError, match=match):
            travel.update(posed(state, .005).model_copy(update=change), 10.005)
    travel = ForwardTravel(plan, Log())
    travel.latch(posed(state, 0))
    travel.activate(10)
    with pytest.raises(ContractError, match='discontinuity'):
        travel.update(posed(state, 0, x=.001), 10.005)


@pytest.mark.parametrize('axis,direction,match', [('x',-1,'backwards'), ('y',1,'lateral')])
def test_displacement_bounds_without_odometry_jump(state, plan, axis, direction, match):
    travel = ForwardTravel(plan, Log())
    travel.latch(posed(state, 0))
    travel.activate(10)
    with pytest.raises(ContractError, match=match):
        for tick in range(1, 20):
            travel.update(posed(state, tick*.1, **{axis:direction*.01*tick}), 10+tick*.1)


def test_target_overshoot_stops(state, plan):
    travel = ForwardTravel(plan, Log())
    travel.latch(posed(state, 0))
    travel.activate(10)
    with pytest.raises(ContractError, match='overshoot'):
        for tick in range(1, 80):
            travel.update(posed(state, tick*.1, x=tick*.01, speed=.1), 10+tick*.1)


def test_manifest_requires_plan_and_standing_rejects_velocity(state, plan):
    path = ROOT / 'policies/relic-exploy-standing/manifest.json'
    m = load_manifest(path)
    data = m.model_dump()
    data['task'] = 'walking'
    with pytest.raises(ValidationError, match='plan'):
        ExployReLICManifest.model_validate(data)
    data['task'] = 'standing'
    data['walking'] = plan.model_dump()
    with pytest.raises(ValidationError, match='plan'):
        ExployReLICManifest.model_validate(data)
    p = make_policy(check_artifacts(path,m),m,path)
    p.initialize_height({'height_m':.52,'foot_contacts':[1]*4})
    with pytest.raises(ContractError, match='walking plan'):
        p.evaluate(state,np.zeros(12),velocity_command=[.1,0,0])


def test_nonzero_exploy_velocity_matches_independent_raw_actor_and_joint_mapping(state, plan):
    from spot_deploy.relic_policy import ReLICPolicy, joint_targets
    path = ROOT / 'policies/relic-exploy-standing/manifest.json'
    data = load_manifest(path).model_dump()
    data.update(task='walking',walking=plan.model_dump())
    m = ExployReLICManifest.model_validate(data)
    p = make_policy(check_artifacts(path,m),m,path)
    p.initialize_height({'height_m':.52,'foot_contacts':[1]*4})
    ref = ReLICPolicy(ROOT / 'simulation/source/relic/relic/assets/spot/pretrained/policy.onnx')
    previous = np.zeros(12, np.float32)
    for speed in [0,.025,.1,.05,0]:
        target,_,obs,action = p.evaluate(state,previous,velocity_command=[speed,0,0])
        expected = observation(state,m.arm_stowed_positions,previous,.52,[speed,0,0])
        reference_action = ref.session.run(['actions'],{'obs':expected[None]})[0][0]
        np.testing.assert_allclose(obs[0],expected,atol=2e-7,rtol=0)
        np.testing.assert_allclose(action,reference_action,atol=1e-5,rtol=0)
        np.testing.assert_allclose(target,joint_targets(reference_action,m.arm_stowed_positions),atol=2e-6)
        np.testing.assert_array_equal(p.previous_action,np.zeros(12))
        previous = action
    for invalid in [[.2,0,0],[.1,.01,0],[-.1,0,0],[.1,0,.01]]:
        with pytest.raises(ContractError):
            p.evaluate(state,previous,velocity_command=invalid)


def test_walk_requires_distinct_evidence_gates(tmp_path, plan):
    from spot_deploy.readiness import report
    manifest = tmp_path/'manifest.json'
    manifest.write_text(json.dumps({'task':'walking','walking':plan.model_dump()}))
    result=report(manifest_path=manifest)
    assert not result['evidence_ready']
    assert {'walking_distance','walking_clearance'} <= {row['gate'] for row in result['checks']}


def test_sdk_stream_body_pose_and_timestamp_are_preserved():
    from bosdyn.api.robot_state_pb2 import RobotStateStreamResponse
    from spot_deploy.sdk_read import decode_state
    message=RobotStateStreamResponse()
    for name in ['position','velocity','load']:
        getattr(message.joint_states,name).extend([0]*19)
    message.joint_states.acquisition_timestamp.seconds=1000
    message.kinematic_state.acquisition_timestamp.seconds=1000
    pose=message.kinematic_state.odom_tform_body
    pose.rotation.w=1
    pose.position.x=4
    pose.position.y=5
    pose.position.z=.52
    state=decode_state(message,received=10)
    assert state.body_position_odom == [4,5,.52] and state.body_pose_robot_time_s == 1000


def test_pose_state_finite_and_shapes(state):
    for position in [[1,2], [float('nan'),0,0]]:
        with pytest.raises(ValidationError):
            State.model_validate(state.model_dump() | {'body_position_odom':position})


def test_walk_plan_rejects_infeasible_timeout(plan):
    with pytest.raises(ValidationError):
        WalkingPlan.model_validate(plan.model_dump() | {'max_duration_s':5})


def test_final_hold_requires_low_measured_speed_and_checks_drift(state, plan):
    travel = ForwardTravel(plan, Log())
    travel.latch(posed(state, 0))
    travel.activate(10)
    for tick in range(1, 76):
        t = tick*.1
        travel.update(posed(state, t, x=.01*tick, speed=.1), 10+t)
    for tick in range(76, 91):
        travel.update(posed(state, tick*.1, x=.75, speed=.1), 10+tick*.1)
    assert not travel.completed and travel.velocity == 0
    for tick in range(91, 103):
        travel.update(posed(state, tick*.1, x=.75, speed=0), 10+tick*.1)
    assert travel.completed
    with pytest.raises(ContractError, match='final position drift'):
        travel.update(posed(state, 10.3, x=.726), 20.3)
