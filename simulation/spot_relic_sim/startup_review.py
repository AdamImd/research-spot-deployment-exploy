"""Read shared-runtime traces without integrating or modifying their dynamics."""
import json
from pathlib import Path

import numpy as np

from .contract import sha256

LEG_NAMES = tuple(f"{leg}_{joint}" for leg in ("fl", "fr", "hl", "hr")
                  for joint in ("hx", "hy", "kn"))


def joint_ids(names):
    if len(names) != len(set(names)):
        raise ValueError("duplicate simulation joint name")
    return [names.index(n) for n in LEG_NAMES]


class StartupTrace:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        complete = json.loads((self.directory / "COMPLETE.json").read_text())
        if complete["status"] != "completed":
            raise ValueError("run did not complete its record")
        for name, digest in complete["artifacts"].items():
            path = (self.directory / name).resolve()
            if not path.is_relative_to(self.directory) or sha256(path) != digest:
                raise ValueError(f"changed or unsafe input: {name}")
        self.config = json.loads((self.directory / "configuration.json").read_text())
        self.result = json.loads((self.directory / "result.json").read_text())
        self.names = self.config.get("simulation_joint_names")
        if self.names is None:
            source = json.loads((self.directory / "source.json").read_text())["simulation_source"]
            self.names = json.loads((Path(source) / "configs/articulation.json").read_text())["joint_names"]
        self.ids = joint_ids(self.names)
        self.rows = [json.loads(s) for s in (self.directory / "rollout.jsonl").read_text().splitlines()]
        self.rows = [r for r in self.rows if r["phase"] == "policy"]
        if len(self.rows) < 2:
            raise ValueError("fewer than two recorded policy physics samples")
        self.t = np.array([r["time"] for r in self.rows])
        np.testing.assert_allclose(np.diff(self.t), .005, rtol=0, atol=1e-8)
        self.q = np.array([r["q"] for r in self.rows])[:, self.ids]
        self.dq = np.array([r["dq"] for r in self.rows])[:, self.ids]
        self.target = np.array([r["positions"] for r in self.rows])[:, :12]
        self.height = np.array([r["height"] for r in self.rows])
        self.tilt = np.rad2deg([r["tilt"] for r in self.rows])
        self.fd_time = (self.t[:-1] + self.t[1:]) / 2
        self.fd_dq = np.diff(self.q, axis=0) / np.diff(self.t)[:, None]
        if "pd_torque_Nm" in self.rows[0]:
            self.requested = np.array([r["pd_torque_Nm"] for r in self.rows])[:, :12]
            # Torque at state i+1 was applied during the interval starting at i.
            self.applied = np.array([r["preceding_applied_torque_Nm"] for r in self.rows[1:]])[:, :12]
        else:
            self.requested = self.applied = None

    def metrics(self):
        first = self.t <= 1 + 1e-8
        interval = self.t[:-1] < 1 - 1e-8
        six = self.t <= 6 + 1e-8
        i, j = np.unravel_index(np.abs(self.dq[six]).argmax(), self.dq[six].shape)
        result = dict(first_target_gap_deg=float(np.rad2deg(np.abs(self.target[0] - self.q[0])).max()),
                      first_second_complete=bool(self.t[-1] >= 1),
                      first_second_leg_excursion_deg=float(np.rad2deg(np.abs(self.q[first] - self.q[0])).max()),
                      first_second_peak_reported_leg_speed_rad_s=float(np.abs(self.dq[first]).max()),
                      first_second_peak_position_derived_leg_speed_rad_s=float(np.abs(self.fd_dq[interval]).max()),
                      first_second_height_drop_m=float(self.height[0] - self.height[first].min()),
                      first_second_max_tilt_deg=float(self.tilt[first].max()),
                      initial_height_m=float(self.height[0]),
                      reason=self.result["reason"], duration_s=float(self.t[-1]),
                      stable_final_window=self.result["stable_final_window"],
                      late_window_metrics=self.result["late_window_metrics"],
                      first_six_seconds=dict(
                          max_leg_excursion_deg=float(np.rad2deg(np.abs(self.q[six] - self.q[0])).max()),
                          min_height_m=float(self.height[six].min()),
                          min_height_time_s=float(self.t[np.argmin(self.height[six])]),
                          max_height_drop_m=float(self.height[0] - self.height[six].min()),
                          peak_velocity_time_s=float(self.t[i]), peak_velocity_joint=LEG_NAMES[j],
                          peak_reported_velocity_rad_s=float(self.dq[i, j])),
                      initial_foot_centers_xyz_m=self.rows[0]["feet"])
        if self.requested is not None:
            result.update(first_second_peak_requested_leg_torque_Nm=float(np.abs(self.requested[first]).max()),
                          first_second_peak_applied_leg_torque_Nm=float(np.abs(self.applied[interval]).max()))
            result["first_six_seconds"].update(
                peak_requested_torque_Nm=float(np.abs(self.requested[six]).max()),
                peak_applied_torque_Nm=float(np.abs(self.applied[self.t[:-1] < 6]).max()))
        return result
