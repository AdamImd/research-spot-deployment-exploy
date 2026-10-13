"""Body-frame foot-position IK from the pinned support URDF; no robot capability."""
import xml.etree.ElementTree as ET

import numpy as np

from .contracts import JOINTS, ContractError
from .static_support import axis_rotation


class LegPositionIK:
    def __init__(self, support, model_path, leg='fl', tolerance_m=.0002):
        self.ids = [JOINTS.index(f'{leg}_{axis}') for axis in ('hx', 'hy', 'kn')]
        self.tolerance_m = tolerance_m
        by_child = {joint[1]: joint for joint in support.joints}
        child, chain = f'{leg}_foot', []
        while child != support.root:
            if child not in by_child:
                raise ContractError('IK foot chain is disconnected')
            joint = by_child[child]
            chain.append(joint)
            child = joint[0]
        self.chain = chain[::-1]
        if [j[2] for j in self.chain if j[2] is not None] != self.ids:
            raise ContractError('IK requires the named three-joint leg chain')
        limits = {}
        for element in ET.parse(model_path).findall('joint'):
            name = element.get('name').replace('.', '_')
            if name in (JOINTS[i] for i in self.ids):
                limit = element.find('limit')
                if limit is None:
                    raise ContractError('IK model needs explicit joint-angle bounds')
                limits[name] = (float(limit.get('lower')), float(limit.get('upper')))
        self.lower = np.array([limits[JOINTS[i]][0] for i in self.ids]) + .02
        self.upper = np.array([limits[JOINTS[i]][1] for i in self.ids]) - .02
        if (not np.isfinite([self.lower, self.upper]).all()
                or np.any(self.lower >= self.upper)):
            raise ContractError('invalid IK model angle bounds')

    def forward(self, angles):
        q = np.asarray(angles, dtype=float)
        if q.shape != (3,) or not np.isfinite(q).all():
            raise ContractError('IK requires three finite named leg angles')
        point, basis = np.zeros(3), np.eye(3)
        pivots, axes = [], []
        for _, _, idx, xyz, rotation, axis in self.chain:
            point = point + basis @ xyz
            basis = basis @ rotation
            if idx is not None:
                pivots.append(point.copy())
                axes.append(basis @ axis)
                basis = basis @ axis_rotation(axis, q[self.ids.index(idx)])
        jacobian = np.cross(np.array(axes), point - np.array(pivots)).T
        return point, jacobian

    def solve(self, target_body_m, seed):
        target = np.asarray(target_body_m, dtype=float)
        q = np.asarray(seed, dtype=float).copy()
        if (target.shape != (3,) or q.shape != (3,)
                or not np.isfinite([target, q]).all()):
            raise ContractError('IK target and seed must be finite xyz/angles')
        q = np.clip(q, self.lower, self.upper)
        for _ in range(80):
            actual, jacobian = self.forward(q)
            error = target - actual
            if np.linalg.norm(error) <= self.tolerance_m:
                if np.linalg.svd(jacobian, compute_uv=False).min() < .005:
                    raise ContractError('IK target too close to a leg singularity')
                return q.astype(np.float32), float(np.linalg.norm(error))
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + .002**2 * np.eye(3), error)
            step *= min(1., .12 / max(np.max(np.abs(step)), 1e-12))
            q = np.clip(q + step, self.lower, self.upper)
        raise ContractError('IK target unreachable within model bounds/tolerance')
