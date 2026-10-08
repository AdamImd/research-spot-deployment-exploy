"""Simulation-only posture preparation; never modifies ReLIC or its history."""
import numpy as np

from .contract import ARM, C, vector


def interpolate_pose(initial, final, time_s, duration_s=5.):
    """Quintic position interpolation with zero endpoint velocity/acceleration."""
    initial, final = vector(initial, 19, "initial pose"), vector(final, 19, "final pose")
    if not np.isfinite(time_s) or not np.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("Interpolation requires finite time and positive duration")
    s = np.clip(time_s / duration_s, 0., 1.)
    alpha = s ** 3 * (10 + s * (-15 + 6 * s))
    return initial + alpha * (final - initial)


def preparation_gains(contract):
    return np.asarray([C.ARM_STIFFNESS[ARM.index(n)] if n in ARM else 60.
                       for n in contract.names])


class StaticSupport:
    """URDF gravity compensation assuming four stationary foot contacts.

    Find minimum-norm foot forces balancing the unactuated floating base, then
    supply the remaining generalized gravity at the joints. This applies ONLY
    joint torque, through a PD target offset; no root force or constraint is used.
    The same reference model is used for both engines, so it is approximate for
    Isaac. Force feasibility and base-wrench residuals are exposed, not concealed.
    """

    def __init__(self, contract, output):
        import mujoco as mj
        from .mujoco_backend import convert_model
        self.mj, self.contract = mj, contract
        self.model = mj.MjModel.from_xml_path(str(convert_model(output)))
        self.data = mj.MjData(self.model)
        joints = [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, n) for n in contract.names]
        self.qids = self.model.jnt_qposadr[joints]
        self.dids = self.model.jnt_dofadr[joints]
        self.feet = [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, f"{leg}_foot")
                     for leg in ("fl", "fr", "hl", "hr")]

    def compute(self, q, quaternion):
        mj, model, data = self.mj, self.model, self.data
        data.qpos[:3] = [0., 0., 1.]
        data.qpos[3:7] = quaternion
        data.qpos[self.qids] = vector(q, 19, "support pose")
        data.qvel[:] = 0
        mj.mj_forward(model, data)
        jacobians = []
        for body in self.feet:
            jac = np.zeros((3, model.nv))
            # Sphere contact is vertically below its center on level ground.
            point = data.xpos[body] - np.array([0., 0., .036])
            mj.mj_jac(model, data, jac, None, point, body)
            jacobians.append(jac)
        jac = np.concatenate(jacobians, axis=0)
        forces = np.linalg.lstsq(jac[:, :6].T, data.qfrc_bias[:6], rcond=None)[0]
        remaining = data.qfrc_bias - jac.T @ forces
        contacts = forces.reshape(4, 3)
        ratio = np.linalg.norm(contacts[:, :2], axis=1) / np.maximum(contacts[:, 2], 1e-9)
        metadata = dict(contact_forces_world_N=contacts.tolist(),
                        base_wrench_residual=float(np.linalg.norm(remaining[:6])),
                        minimum_normal_force_N=float(contacts[:, 2].min()),
                        max_friction_ratio=float(ratio.max()))
        if not np.isfinite(remaining).all() or metadata["base_wrench_residual"] > 1e-6:
            raise ValueError("Cannot balance reference-model gravity")
        if metadata["minimum_normal_force_N"] < 0 or metadata["max_friction_ratio"] > .8:
            raise ValueError("Four-foot static support solution is infeasible")
        return remaining[self.dids].copy(), metadata
