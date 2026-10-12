"""Offline walking diagnostics; body odometry is not evidence of foot stepping."""

import numpy as np

from .contracts import JOINTS, ContractError


def arrays(rows, config, origin):
    names = [name.replace('arm_', 'arm0_') for name in config['simulation_joint_names']]
    if len(names) != 19 or set(names) != set(JOINTS) or len(rows) < 2:
        raise ContractError('walking diagnostics need complete named joint states')
    ids = [names.index(name) for name in JOINTS]
    t = np.array([r['trial_time'] for r in rows])
    if np.any(np.diff(t) <= 0):
        raise ContractError('walking diagnostic timestamps must increase')
    yaw = origin['yaw_rad']
    forward = np.array([np.cos(yaw), np.sin(yaw)])
    position = np.array([r['position'] for r in rows])
    feet = np.array([r['feet'] for r in rows])
    return dict(t=t, q=np.array([r['q'] for r in rows])[:, ids],
        dq=np.array([r['dq'] for r in rows])[:, ids],
        target=np.array([r['positions'] for r in rows]),
        torque=np.array([r['pd_torque_Nm'] for r in rows]),
        applied=np.array([r['preceding_applied_torque_Nm'] for r in rows]),
        body_forward=(position[:, :2]-origin['odom_position_m'][:2]) @ forward,
        foot_forward=(feet[:, :, :2]-feet[0, :, :2]) @ forward,
        foot_excursion=np.linalg.norm(feet[:, :, :2]-feet[0, :, :2], axis=2),
        clearance=feet[:, :, 2]-.036)


def summarize(rows, config, events):
    origins = [e for e in events if e.get('event') == 'walk_origin']
    if len(origins) != 1 or not config['manifest'].get('walking'):
        raise ContractError('walking diagnostics require exactly one measured origin and walking plan')
    origin = origins[0]
    data = arrays(rows, config, origin)
    policies = [e for e in events if e.get('event') == 'policy']
    moving = [e for e in policies if e['velocity_command'][0] > 0]
    requested_at = (moving[0]['state']['body_pose_robot_time_s']-origin['robot_time_s']
                    if moving else None)
    before = np.flatnonzero(data['t'] < requested_at) if moving else np.arange(len(rows))
    failures = [e for e in events if e.get('event') == 'first_failure']
    failure = failures[-1] if failures else None
    limits = np.asarray(config['envelope']['velocity_max'])
    ratios = np.abs(data['dq'])/limits
    reject = None
    if failure:
        state = failure['state']
        reject = dict(time_s=failure['simulation_time_s'], reason=failure['reason'],
            velocity_violations=[dict(joint=JOINTS[i], velocity_rad_s=velocity,
                                      limit_rad_s=float(limits[i]))
                for i, velocity in enumerate(state['velocities'][:12]) if abs(velocity) > limits[i]],
            tracking_violations=[])
        issued = [e for e in events if e.get('event') == 'command']
        target = (policies[-1].get('targets') if failure['reason'].startswith('command ')
                  and policies else issued[-1].get('positions') if issued else None)
        if target is not None:
            error = np.asarray(target)-state['positions']
            torque = (np.asarray(config['manifest']['gains']['kp'])*error
                      - np.asarray(config['manifest']['gains']['kd'])*state['velocities'])
            caps = config['envelope']['tracking_error_max']
            reject['target_source'] = ('rejected policy prediction, not issued'
                if failure['reason'].startswith('command ') else 'last issued command')
            reject['tracking_violations'] = [dict(joint=JOINTS[i], error_rad=float(error[i]),
                limit_rad=caps[i], target_rad=target[i], measured_rad=state['positions'][i],
                predicted_pd_torque_Nm=float(torque[i])) for i in range(12) if abs(error[i]) > caps[i]]
    if reject and reject['velocity_violations']:
        selected = max(reject['velocity_violations'],
                       key=lambda v: abs(v['velocity_rad_s'])/v['limit_rad_s'])['joint']
    elif reject and reject['tracking_violations']:
        selected = max(reject['tracking_violations'],
                       key=lambda v: abs(v['error_rad'])/v['limit_rad'])['joint']
    else:
        selected = JOINTS[int(np.argmax(np.max(ratios[:, :12], axis=0)))]
    joint = JOINTS.index(selected)
    feet = {}
    for i, leg in enumerate(('fl', 'fr', 'hl', 'hr')):
        airborne = data['clearance'][:, i] >= .015
        onset = np.flatnonzero(airborne & np.r_[True, ~airborne[:-1]])
        feet[leg] = dict(first_clearance_15mm_s=float(data['t'][onset[0]]) if len(onset) else None,
            clearance_crossings_15mm=int(len(onset)),
            max_clearance_m=float(data['clearance'][:, i].max()),
            max_planar_excursion_m=float(data['foot_excursion'][:, i].max()),
            final_forward_displacement_m=float(data['foot_forward'][-1, i]))
    # Position-derived speed is independent of the simulator's dq field.
    differentiated = np.diff(data['q'][:, joint])/np.diff(data['t'])
    rejected_speed = None
    if failure and failure['simulation_time_s'] > data['t'][-1]:
        rejected_speed = float((failure['state']['positions'][joint]-data['q'][-1, joint])
                               /(failure['simulation_time_s']-data['t'][-1]))
    report = dict(hardware_access=False, hardware_qualified=False,
        first_nonzero_velocity_request_s=requested_at,
        forward_before_nonzero_request_m=float(data['body_forward'][before[-1]]) if len(before) else 0.,
        final_body_forward_m=float(data['body_forward'][-1]),
        final_mean_foot_forward_m=float(data['foot_forward'][-1].mean()), feet=feet,
        support_evidence='foot-center geometry only; normal forces and loaded support not measured',
        selected_joint=selected, selected_joint_speed_limit_rad_s=float(limits[joint]),
        selected_joint_max_accepted_speed_rad_s=float(np.abs(data['dq'][:, joint]).max()),
        selected_joint_max_position_derived_speed_rad_s=float(np.abs(differentiated).max()),
        selected_joint_rejected_step_position_derived_speed_rad_s=rejected_speed,
        selected_joint_peak_requested_pd_torque_Nm=float(np.abs(data['torque'][:, joint]).max()),
        first_failure=reject, accepted_trajectory_end_s=float(data['t'][-1]))
    return report, data, origin, failure
