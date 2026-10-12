import copy

import numpy as np
import pytest

from spot_deploy.contracts import JOINTS, ContractError
from spot_deploy.walk_diagnostics import summarize


def inputs():
    names = list(reversed(JOINTS))
    config = dict(simulation_joint_names=names, manifest={'walking': {'initial_hold_s': 1.}},
                  envelope={'velocity_max': [8.]*19})
    rows = []
    for tick in range(3):
        q, dq = [0.]*19, [0.]*19
        q[names.index('fr_kn')] = -1.4-.05*tick
        dq[names.index('fr_kn')] = -.1*tick
        rows.append(dict(trial_time=tick*.5, q=q, dq=dq, positions=[0.]*19,
            pd_torque_Nm=[0.]*19, preceding_applied_torque_Nm=[0.]*19,
            position=[0, .125*tick, .5], feet=[[i*.1, 0, .036] for i in range(4)]))
    events = [dict(event='walk_origin', yaw_rad=np.pi/2, odom_position_m=[0, 0, .5], robot_time_s=1000),
              dict(event='policy', velocity_command=[0, 0, 0], state={'body_pose_robot_time_s': 1000}),
              dict(event='policy', velocity_command=[.14, 0, 0], state={'body_pose_robot_time_s': 1001})]
    state = dict(positions=[0.]*19, velocities=[0.]*19)
    state['velocities'][JOINTS.index('fr_kn')] = -8.2
    events.append(dict(event='first_failure', simulation_time_s=1.005,
                       reason='measured joint velocity or load limit', state=state))
    return rows, config, events


def test_body_translation_is_separate_from_foot_steps_and_rejected_velocity():
    rows, config, events = inputs()
    report, data, _, _ = summarize(rows, config, events)
    assert report['forward_before_nonzero_request_m'] == pytest.approx(.125)
    assert report['final_body_forward_m'] == pytest.approx(.25)
    assert report['final_mean_foot_forward_m'] == 0
    assert report['feet']['fr']['clearance_crossings_15mm'] == 0
    assert report['selected_joint'] == 'fr_kn'
    assert report['selected_joint_max_position_derived_speed_rad_s'] == pytest.approx(.1)
    assert report['first_failure']['velocity_violations'][0]['velocity_rad_s'] == -8.2
    np.testing.assert_allclose(data['q'][:, JOINTS.index('fr_kn')], [-1.4, -1.45, -1.5])


def test_real_foot_geometry_is_reported_without_claiming_loaded_contact():
    rows, config, events = inputs()
    rows[1]['feet'][0][2] = .076
    rows[2]['feet'][0][1] = .1
    report, _, _, _ = summarize(rows, config, events)
    assert report['feet']['fl']['first_clearance_15mm_s'] == .5
    assert report['feet']['fl']['final_forward_displacement_m'] == pytest.approx(.1)
    assert report['feet']['fl']['max_planar_excursion_m'] == pytest.approx(.1)
    assert 'not measured' in report['support_evidence']
    assert not report['hardware_qualified']


def test_unordered_times_and_missing_joint_names_are_rejected():
    rows, config, events = inputs()
    bad = copy.deepcopy(rows)
    bad[1]['trial_time'] = 0
    with pytest.raises(ContractError, match='timestamps'):
        summarize(bad, config, events)
    config['simulation_joint_names'][0] = 'unknown'
    with pytest.raises(ContractError, match='named joint'):
        summarize(rows, config, events)
