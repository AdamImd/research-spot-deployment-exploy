"""URDF gravity support for a four-foot preparation phase; no robot I/O.

Only inertials and joint transforms are read. Visual meshes are never used.
Compare this solver with an independent dynamics library before qualification.
"""

import xml.etree.ElementTree as ET

import numpy as np

from .contracts import JOINTS, ContractError
from .policy import rotation


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def axis_rotation(axis, angle):
    s = skew(axis)
    return np.eye(3) + np.sin(angle) * s + (1 - np.cos(angle)) * (s @ s)


def vec(text, default=(0., 0., 0.)):
    result = np.asarray([float(v) for v in text.split()] if text else default)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ContractError("invalid support-model vector")
    return result


def origin(element):
    xyz = vec(element.get("xyz")) if element is not None else np.zeros(3)
    rpy = vec(element.get("rpy")) if element is not None else np.zeros(3)
    r = np.eye(3)
    for axis, angle in zip(np.eye(3)[::-1], rpy[::-1]):
        r = r @ axis_rotation(axis, angle)
    return xyz, r


class StaticSupport:
    def __init__(self, path, radius, friction):
        raw = path.read_text()
        if len(raw) > 4_000_000 or "<!DOCTYPE" in raw.upper() or "<!ENTITY" in raw.upper():
            raise ContractError("invalid support-model XML")
        tree = ET.fromstring(raw)
        self.radius, self.friction = radius, friction
        self.inertials, pending = {}, []
        for link in tree.findall("link"):
            name = link.attrib["name"]
            if name in self.inertials:
                raise ContractError("duplicate support-model link")
            element = link.find("inertial")
            mass = float(element.find("mass").get("value")) if element is not None else 0.
            if not np.isfinite(mass) or mass < 0:
                raise ContractError("invalid support-model mass")
            com = origin(element.find("origin"))[0] if element is not None else np.zeros(3)
            self.inertials[name] = (mass, com)
        for joint in tree.findall("joint"):
            name = joint.get("name").replace(".", "_")
            if name.startswith("arm_"):
                name = "arm0_" + name[4:]
            kind = joint.get("type")
            if kind not in ("fixed", "revolute", "continuous"):
                raise ContractError("support requires revolute or fixed joints")
            idx = None if kind == "fixed" else JOINTS.index(name)
            axis = vec(joint.find("axis").get("xyz") if joint.find("axis") is not None
                       else None, (1., 0., 0.))
            if np.linalg.norm(axis) < 1e-9:
                raise ContractError("zero support-model joint axis")
            pending.append((joint.find("parent").get("link"), joint.find("child").get("link"),
                            idx, *origin(joint.find("origin")), axis / np.linalg.norm(axis)))
        if sorted(j[2] for j in pending if j[2] is not None) != list(range(19)):
            raise ContractError("support model must contain all 19 unique joints")
        roots = set(self.inertials) - {j[1] for j in pending}
        if len(roots) != 1 or len(pending) != len(self.inertials) - 1:
            raise ContractError("support model must be a tree")
        self.root = roots.pop()
        seen, self.joints = {self.root}, []
        while pending:
            ready = [j for j in pending if j[0] in seen]
            if not ready:
                raise ContractError("disconnected support model")
            for joint in ready:
                if joint[1] in seen or joint[1] not in self.inertials:
                    raise ContractError("invalid support-model child")
                self.joints.append(joint)
                seen.add(joint[1])
                pending.remove(joint)
        self.feet = [f"{leg}_foot" for leg in ("fl", "fr", "hl", "hr")]
        if not set(self.feet).issubset(seen):
            raise ContractError("four foot links required for support")

    def compute(self, state):
        frames = {self.root: (np.zeros(3), rotation(state), [])}
        axes, pivots = np.zeros((19, 3)), np.zeros((19, 3))
        for parent, child, idx, xyz, r, axis in self.joints:
            p, rot, ancestors = frames[parent]
            pivot, basis = p + rot @ xyz, rot @ r
            chain = ancestors
            if idx is not None:
                axes[idx], pivots[idx] = basis @ axis, pivot
                basis = basis @ axis_rotation(axis, state.positions[idx])
                chain = ancestors + [idx]
            frames[child] = (pivot, basis, chain)

        def jacobian(point, chain):
            jac = np.zeros((3, 25))
            jac[:, :3], jac[:, 3:6] = np.eye(3), -skew(point)
            jac[:, np.asarray(chain, dtype=int) + 6] = np.cross(
                axes[chain], point - pivots[chain]).T
            return jac

        gravity = np.zeros(25)
        for name, (mass, com) in self.inertials.items():
            p, rot, chain = frames[name]
            gravity += jacobian(p + rot @ com, chain).T @ [0., 0., 9.81 * mass]
        jac = np.concatenate([jacobian(frames[name][0] - [0., 0., self.radius],
                                       frames[name][2]) for name in self.feet])
        forces = np.linalg.lstsq(jac[:, :6].T, gravity[:6], rcond=None)[0]
        residual = gravity - jac.T @ forces
        contacts = forces.reshape(4, 3)
        ratio = np.linalg.norm(contacts[:, :2], axis=1) / np.maximum(contacts[:, 2], 1e-9)
        if (not np.isfinite(residual).all() or np.linalg.norm(residual[:6]) > 1e-6
                or contacts[:, 2].min() < 0 or ratio.max() > self.friction):
            raise ContractError("four-foot support solution is infeasible")
        return residual[6:], dict(contact_forces_world_N=contacts.tolist(),
                                 base_wrench_residual=float(np.linalg.norm(residual[:6])),
                                 max_friction_ratio=float(ratio.max()))
