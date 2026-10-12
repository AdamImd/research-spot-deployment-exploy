"""Read-only contact instrumentation and controlled MuJoCo integration diagnostics."""

import numpy as np


def control_step(backend, target, substeps=1):
    """Hold one 200 Hz PD torque over finer physics ticks; delay advances once."""
    import mujoco as mj
    if substeps not in (1, 4):
        raise ValueError('diagnostic substeps must be 1 or 4')
    before = float(backend.data.time)
    backend.data.qfrc_applied[backend.dids] = backend.torque(target)
    for _ in range(substeps):
        mj.mj_step(backend.model, backend.data)
    mj.mj_forward(backend.model, backend.data)
    if not np.isclose(backend.data.time-before, .005, atol=1e-10, rtol=0):
        raise ValueError('diagnostic step must advance exactly one 200 Hz control interval')


def contacts(backend):
    if hasattr(backend, 'data'):
        import mujoco as mj
        model, data = backend.model, backend.data
        forces = np.zeros((4, 3))
        velocities = np.zeros((4, 3))
        contact_pairs = []
        for index, contact in enumerate(data.contact):
            bodies = model.geom_bodyid[contact.geom]
            force = np.zeros(6)
            mj.mj_contactForce(model, data, index, force)
            # Contact-force frame points from geom1 to geom2. Only normal force,
            # matching Isaac ContactSensor.net_forces_w (no tangential component).
            normal = np.array(contact.frame).reshape(3, 3)[0] * force[0]
            for j, body in enumerate(backend.feet):
                if body in bodies and 0 in bodies:
                    forces[j] += normal if bodies[1] == body else -normal
            contact_pairs.append(dict(bodies=[mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, int(b))
                                              if b else 'world' for b in bodies],
                penetration_m=max(0., -float(contact.dist)), normal_force_N=float(force[0])))
        for j, body in enumerate(backend.feet):
            velocity = np.zeros(6)
            mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, body, velocity, 0)
            velocities[j] = velocity[3:]
        return dict(foot_normal_force_w_N=forces.tolist(), foot_velocity_w_m_s=velocities.tolist(),
                    contact_pairs=contact_pairs,
                    joint_constraint_torque_Nm=data.qfrc_constraint[backend.dids].tolist(),
                    note='MuJoCo instantaneous solved force; constraint torque includes all constraints')
    sensor = backend.contact_sensor
    ids = [sensor.body_names.index(f'{leg}_foot') for leg in ('fl', 'fr', 'hl', 'hr')]
    return dict(foot_normal_force_w_N=sensor.data.net_forces_w[0, ids].cpu().numpy().tolist(),
                foot_velocity_w_m_s=backend.robot.data.body_lin_vel_w[0, backend.feet].cpu().numpy().tolist(),
                note='PhysX normal contact impulse / 5 ms, preceding physics step; not pointwise MuJoCo force')


def model_snapshot(backend):
    result = backend.metadata()
    if hasattr(backend, 'model'):
        import mujoco as mj
        m = backend.model
        result.update(body_names=[mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, i) for i in range(1, m.nbody)],
            joint_names=list(backend.contract.names),
            inertial_frame_quaternions_wxyz=m.body_iquat[1:].tolist(),
            solver=dict(timestep=float(m.opt.timestep), iterations=int(m.opt.iterations),
                noslip_iterations=int(m.opt.noslip_iterations), cone=int(m.opt.cone),
                geom_solref=m.geom_solref.tolist(), geom_solimp=m.geom_solimp.tolist(),
                geom_condim=m.geom_condim.tolist(), geom_friction=m.geom_friction.tolist()),
            armature=m.dof_armature[backend.dids].tolist(),
            damping=m.dof_damping[backend.dids].tolist(), frictionloss=m.dof_frictionloss[backend.dids].tolist())
    else:
        result.update(armature=backend.robot.data.joint_armature[0].cpu().numpy().tolist(),
            solver=dict(timestep=.005, type='PhysX TGS', articulation_position_iterations=4,
                        articulation_velocity_iterations=0, max_depenetration_velocity_m_s=1.),
            physics_config=backend.sim.cfg.to_dict())
        # Non-JSON configuration objects (material funcs etc.) need textual provenance.
        result['physics_config'] = repr(result['physics_config'])
    return result
