"""Calibrated masked RGB-D geometry. Optical coordinates: x right, y down, z forward."""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter


@dataclass
class Frame:
    view: str
    rgb: np.ndarray
    depth_m: np.ndarray
    K: np.ndarray
    odom_T_camera: np.ndarray
    timestamp_s: float
    metadata: dict

    def validate(self):
        h, w = self.depth_m.shape
        if self.rgb.shape != (h, w, 3) or self.rgb.dtype != np.uint8:
            raise ValueError('RGB/depth dimensions or RGB dtype invalid')
        if self.K.shape != (3, 3) or not np.isfinite(self.K).all():
            raise ValueError('invalid camera intrinsics')
        if abs(np.linalg.det(self.K)) < 1e-8:
            raise ValueError('singular camera intrinsics')
        t = self.odom_T_camera
        if (t.shape != (4, 4) or not np.isfinite(t).all()
                or not np.allclose(t[3], [0, 0, 0, 1])
                or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)):
            raise ValueError('invalid rigid camera transform')
        if not np.isfinite(self.timestamp_s):
            raise ValueError('invalid acquisition timestamp')
        return self

    def save(self, path):
        np.savez_compressed(path, rgb=self.rgb, depth_m=self.depth_m, K=self.K,
                            odom_T_camera=self.odom_T_camera, timestamp_s=self.timestamp_s,
                            view=self.view, metadata=json.dumps(self.metadata))

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as z:
            return cls(str(z['view']), z['rgb'], z['depth_m'], z['K'], z['odom_T_camera'],
                       float(z['timestamp_s']), json.loads(str(z['metadata']))).validate()


def masked_cloud(frame, mask, min_depth=.2, max_depth=4., erosion=1):
    frame.validate()
    mask = np.asarray(mask, dtype=bool)
    if mask.shape != frame.depth_m.shape:
        raise ValueError('mask must have the exact image dimensions')
    if not 0 < min_depth < max_depth or erosion not in (0, 1, 2):
        raise ValueError('invalid depth/erosion settings')
    if erosion:
        mask = np.array(Image.fromarray(mask).filter(ImageFilter.MinFilter(2*erosion+1)))
    d = frame.depth_m
    valid = mask & np.isfinite(d) & (d >= min_depth) & (d <= max_depth)
    v, u = np.nonzero(valid)
    rays = np.column_stack((u, v, np.ones(len(u)))) @ np.linalg.inv(frame.K).T
    camera = rays * (d[v, u] / rays[:, 2])[:, None]
    t = frame.odom_T_camera
    xyz = camera @ t[:3, :3].T + t[:3, 3]
    return xyz, frame.rgb[v, u], dict(mask_pixels=int(mask.sum()), valid_points=len(xyz),
                                    valid_depth_fraction=float(valid.sum()/max(mask.sum(), 1)))


def project_visible(xyz_odom, frame, tolerance_m=.05):
    """Project only points consistent with current measured depth (z-buffer occlusion)."""
    frame.validate()
    t = frame.odom_T_camera
    p = (np.asarray(xyz_odom)-t[:3, 3]) @ t[:3, :3]
    p = p[np.isfinite(p).all(axis=1) & (p[:, 2] > .01)]
    q = p @ frame.K.T
    uv = np.rint(q[:, :2]/q[:, 2:3]).astype(int)
    h, w = frame.depth_m.shape
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    uv, p = uv[inside], p[inside]
    z = frame.depth_m[uv[:, 1], uv[:, 0]]
    good = np.isfinite(z) & (z > 0) & (abs(z-p[:, 2]) <= tolerance_m)
    return np.unique(uv[good], axis=0)


def voxel_fuse(points, colors, views, voxel_m=.01):
    """Centroid/mean RGB plus per-voxel bitmask of contributing cameras."""
    if not np.isfinite(voxel_m) or voxel_m <= 0:
        raise ValueError('voxel size must be positive')
    if not len(points):
        return np.empty((0, 3)), np.empty((0, 3), np.uint8), np.empty(0, np.uint8)
    points, colors, views = np.asarray(points), np.asarray(colors), np.asarray(views)
    if not np.isfinite(points).all() or len(colors) != len(points) or len(views) != len(points):
        raise ValueError('invalid point arrays')
    _, inv = np.unique(np.floor(points/voxel_m).astype(np.int64), axis=0, return_inverse=True)
    counts = np.bincount(inv)
    xyz = np.column_stack([np.bincount(inv, weights=points[:, j])/counts for j in range(3)])
    rgb = np.column_stack([np.bincount(inv, weights=colors[:, j])/counts for j in range(3)])
    source = np.zeros(len(counts), np.uint8)
    np.bitwise_or.at(source, inv, views.astype(np.uint8))
    return xyz, np.rint(rgb).astype(np.uint8), source


def write_ply(path, xyz, rgb):
    header = ('ply\nformat binary_little_endian 1.0\ncomment frame odom; units metres\n'
              f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\n'
              'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
    data = np.empty(len(xyz), dtype=[('xyz', '<f4', (3,)), ('rgb', 'u1', (3,))])
    data['xyz'], data['rgb'] = xyz, rgb
    Path(path).write_bytes(header.encode()+data.tobytes())
