"""Spot motor-side torque envelope, numerically pinned to SDK 5.0.1.

Sources: docs/MANUFACTURER_LIMITS.md. Table matches the numerical data already
bundled in ReLIC assets; this module has no simulator or hardware dependency.
Limits are evaluated at measured knee position; outside-table values fail.
"""

import numpy as np

PROFILE = "spot-sdk-5.0.1"
KNEE_INDICES = (2, 5, 8, 11)
KNEE_TABLE = np.array([
    (-2.792900, 37.165077),
    (-2.767442, 39.435162),
    (-2.741984, 41.690054),
    (-2.716526, 43.928996),
    (-2.691068, 46.151304),
    (-2.665610, 48.356134),
    (-2.640152, 50.542751),
    (-2.614694, 52.710331),
    (-2.589236, 54.858078),
    (-2.563778, 56.985128),
    (-2.538320, 59.090595),
    (-2.512862, 61.173609),
    (-2.487404, 63.233231),
    (-2.461946, 65.268557),
    (-2.436488, 67.278557),
    (-2.411030, 69.262310),
    (-2.385572, 71.218735),
    (-2.360114, 73.146824),
    (-2.334656, 75.045502),
    (-2.309198, 76.913641),
    (-2.283740, 78.750154),
    (-2.258282, 80.553881),
    (-2.232824, 82.323664),
    (-2.207366, 84.058290),
    (-2.181908, 85.756542),
    (-2.156450, 87.417200),
    (-2.130992, 89.038971),
    (-2.105534, 90.620607),
    (-2.080076, 92.160793),
    (-2.054618, 93.658218),
    (-2.029160, 95.111538),
    (-2.003702, 96.519402),
    (-1.978244, 97.880505),
    (-1.952786, 99.193417),
    (-1.927328, 100.456764),
    (-1.901870, 101.669186),
    (-1.876412, 102.829296),
    (-1.850954, 103.935677),
    (-1.825496, 104.986988),
    (-1.800038, 105.981812),
    (-1.774580, 106.918785),
    (-1.749122, 107.796478),
    (-1.723664, 108.613632),
    (-1.698206, 109.368851),
    (-1.672748, 110.060806),
    (-1.647290, 110.688194),
    (-1.621832, 111.249767),
    (-1.596374, 111.744221),
    (-1.570916, 112.170376),
    (-1.545458, 112.526997),
    (-1.520000, 112.812984),
    (-1.494542, 113.027172),
    (-1.469084, 113.168530),
    (-1.443626, 113.236015),
    (-1.418168, 113.228657),
    (-1.392710, 113.145515),
    (-1.367252, 112.985744),
    (-1.341794, 112.748531),
    (-1.316336, 112.433109),
    (-1.290878, 112.038826),
    (-1.265420, 111.565041),
    (-1.239962, 111.011215),
    (-1.214504, 110.376869),
    (-1.189046, 109.661613),
    (-1.163588, 108.865128),
    (-1.138130, 107.987183),
    (-1.112672, 107.027561),
    (-1.087214, 105.986229),
    (-1.061756, 104.863220),
    (-1.036298, 103.658581),
    (-1.010840, 102.372505),
    (-0.985382, 101.005291),
    (-0.959924, 99.557270),
    (-0.934466, 98.028923),
    (-0.909008, 96.420799),
    (-0.883550, 94.733540),
    (-0.858092, 92.967882),
    (-0.832634, 91.124662),
    (-0.807176, 89.204767),
    (-0.781718, 87.209255),
    (-0.756260, 85.139231),
    (-0.730802, 82.995924),
    (-0.705344, 80.780594),
    (-0.679886, 78.494694),
    (-0.654428, 76.139643),
    (-0.628970, 73.717049),
    (-0.603512, 71.228605),
    (-0.578054, 68.676006),
    (-0.552596, 66.061146),
    (-0.527138, 63.385900),
    (-0.501680, 60.652325),
    (-0.476222, 57.862421),
    (-0.450764, 55.018473),
    (-0.425306, 52.122648),
    (-0.399848, 49.177254),
    (-0.374390, 46.184715),
    (-0.348932, 43.147428),
    (-0.323474, 40.067954),
    (-0.298016, 36.948864),
    (-0.272558, 33.792821),
    (-0.247100, 30.602500),
], dtype=float)
KNEE_TABLE.setflags(write=False)
# Joint-space bounding box. The two coupled motor constraints below still apply.
MAX_LOADS = (44.88, 44.88, 113.236015) * 4 + (
    89.89, 179.78, 89.89, 23.23, 23.23, 23.23, 11.31,
)
COUPLED_MOTOR_TORQUE = 89.89  # 101 * 0.89, referred to output coordinates.


def torque_limits(positions):
    q = np.asarray(positions, dtype=float)
    if q.shape != (19,) or not np.isfinite(q).all():
        raise ValueError("manufacturer torque limits require nineteen finite positions")
    knees = q[list(KNEE_INDICES)]
    if np.any(knees < KNEE_TABLE[0, 0]) or np.any(knees > KNEE_TABLE[-1, 0]):
        raise ValueError("knee position outside manufacturer torque table")
    limits = np.array(MAX_LOADS)
    limits[list(KNEE_INDICES)] = np.interp(knees, KNEE_TABLE[:, 0], KNEE_TABLE[:, 1])
    return limits


def check_torque(positions, loads):
    """Raise on a violation; never saturate or change a command."""
    torque = np.asarray(loads, dtype=float)
    if torque.shape != (19,) or not np.isfinite(torque).all():
        raise ValueError("manufacturer torque limits require nineteen finite loads")
    if np.any(np.abs(torque) > torque_limits(positions)):
        raise ValueError("manufacturer joint torque limit")
    # qdot = J * motor_qdot implies motor_tau = J.T * joint_tau.
    if abs(torque[13] - torque[14]) > COUPLED_MOTOR_TORQUE:
        raise ValueError("manufacturer coupled SH1/EL0 motor torque limit")
