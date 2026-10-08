"""Actual movement and target-offset metrics for a single, unbroken episode."""
import numpy as np


def motion_metrics(rows, leg_ids, start=0., end=1.):
    selected = [r for r in rows if start - 1e-8 <= r["time"] <= end + 1e-8]
    if len(selected) < 2:
        return {"samples": len(selected), "full_window": False}
    times = np.asarray([r["time"] for r in selected])
    q = np.asarray([r["q"] for r in selected])[:, leg_ids]
    dq = np.asarray([r["dq"] for r in selected])[:, leg_ids]
    targets = np.asarray([r["target"] for r in selected])[:, leg_ids]
    linear = np.asarray([r["linear_velocity"] for r in selected])
    angular = np.asarray([r["angular_velocity"] for r in selected])
    height = np.asarray([r["height"] for r in selected])
    return dict(samples=len(selected), start_s=float(times[0]), end_s=float(times[-1]),
        full_window=bool(times[0] <= start + 1e-8 and times[-1] >= end - 1e-8),
        first_target_gap_deg=float(np.rad2deg(np.abs(targets[0] - q[0])).max()),
        peak_target_gap_deg=float(np.rad2deg(np.abs(targets - q)).max()),
        peak_joint_speed_rad_s=float(np.abs(dq).max()),
        peak_joint_displacement_deg=float(np.rad2deg(np.abs(q - q[0])).max()),
        peak_joint_sample_step_deg=float(np.rad2deg(np.abs(np.diff(q, axis=0))).max()),
        peak_planar_speed_m_s=float(np.linalg.norm(linear[:, :2], axis=1).max()),
        peak_body_z_speed_m_s=float(np.abs(linear[:, 2]).max()),
        peak_angular_speed_rad_s=float(np.linalg.norm(angular, axis=1).max()),
        height_min_m=float(height.min()), height_max_m=float(height.max()),
        height_range_m=float(np.ptp(height)),
        max_tilt_deg=float(np.rad2deg(max(r["tilt"] for r in selected))))
