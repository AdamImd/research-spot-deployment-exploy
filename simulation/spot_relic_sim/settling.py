"""Registered settling and late standing metrics for captured-pose diagnostics."""
import numpy as np


def window_metrics(rows):
    if not rows:
        raise ValueError("Metrics require state samples")
    heights = np.asarray([r["height"] for r in rows])
    tilts = np.asarray([r["tilt"] for r in rows])
    planar = np.linalg.norm([r["linear_velocity"][:2] for r in rows], axis=1)
    linear = np.linalg.norm([r["linear_velocity"] for r in rows], axis=1)
    angular = np.linalg.norm([r["angular_velocity"] for r in rows], axis=1)
    joint = np.max(np.abs([r["dq"] for r in rows]), axis=1)
    return dict(samples=len(rows), span_s=rows[-1]["time"] - rows[0]["time"],
        min_height_m=float(heights.min()), mean_height_m=float(heights.mean()),
        height_span_m=float(np.ptp(heights)), max_tilt_deg=float(np.rad2deg(tilts.max())),
        tilt_span_deg=float(np.rad2deg(np.ptp(tilts))),
        planar_speed_p95_m_s=float(np.quantile(planar, .95)),
        linear_speed_p95_m_s=float(np.quantile(linear, .95)),
        angular_speed_p95_rad_s=float(np.quantile(angular, .95)),
        joint_speed_p95_rad_s=float(np.quantile(joint, .95)))


def quiet_window(rows):
    m = window_metrics(rows)
    passed = (m["span_s"] >= 1 - 1e-8 and m["linear_speed_p95_m_s"] < .02
              and m["angular_speed_p95_rad_s"] < .05 and m["joint_speed_p95_rad_s"] < .1
              and m["height_span_m"] < .002 and m["tilt_span_deg"] < np.rad2deg(.005))
    return bool(passed), m


def standing_window(rows):
    m = window_metrics(rows)
    passed = (m["span_s"] >= 10 - 1e-8 and m["min_height_m"] >= .30
              and m["max_tilt_deg"] < 10 and m["height_span_m"] < .02
              and m["tilt_span_deg"] < 2 and m["planar_speed_p95_m_s"] < .05
              and m["angular_speed_p95_rad_s"] < .1 and m["joint_speed_p95_rad_s"] < .5)
    return bool(passed), m


def pose_quiet_window(rows):
    """Position-based supplemental check; do not discard reported velocities."""
    m = window_metrics(rows)
    m["max_joint_range_deg"] = float(np.rad2deg(np.ptp([r["q"] for r in rows], axis=0)).max())
    m["root_xy_range_norm_m"] = float(np.linalg.norm(np.ptp([r["position"][:2] for r in rows], axis=0)))
    passed = (m["span_s"] >= 1 - 1e-8 and m["height_span_m"] < .002
              and m["tilt_span_deg"] < np.rad2deg(.005) and m["max_joint_range_deg"] < 1
              and m["root_xy_range_norm_m"] < .002)
    return bool(passed), m


def prediction_metrics(contract, state, target):
    error = np.rad2deg(target[contract.leg_ids] - state["q"][contract.leg_ids])
    q = target[contract.leg_ids].reshape(3, 4)
    mirror = np.r_[q[0, 0] + q[0, 1], q[0, 2] + q[0, 3],
                   q[1:, 0] - q[1:, 1], q[1:, 2] - q[1:, 3]]
    return dict(max_leg_gap_deg=float(np.abs(error).max()),
        rms_leg_gap_deg=float(np.sqrt(np.mean(error ** 2))), per_leg_joint_gap_deg=error.tolist(),
        max_mirror_asymmetry_deg=float(np.rad2deg(np.abs(mirror)).max()),
        large_initial_gap=bool(np.abs(error).max() > 20))
