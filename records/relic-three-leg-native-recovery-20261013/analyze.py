"""Analyze saved native-transition diagnostics; no simulation or robot capability."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS


def summary_values(values):
    return dict(
        min=float(np.min(values)), max=float(np.max(values)), p95=float(np.quantile(values, 0.95))
    )


def pd_roundoff_bound(q, dq, kp, kd):
    """Bound PD error from logged float32 state versus simulator float64 state."""
    return 0.5 * (
        np.abs(np.spacing(np.asarray(q, dtype=np.float32))).astype(float) * np.asarray(kp)
        + np.abs(np.spacing(np.asarray(dq, dtype=np.float32))).astype(float) * np.asarray(kd)
    )


def main(run, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=False)
    result = json.loads((run / "result.json").read_text())
    config = json.loads((run / "configuration.json").read_text())
    rows = [json.loads(x) for x in (run / "rollout.jsonl").read_text().splitlines()]
    t = np.array([r["trial_time"] for r in rows])
    dt = np.diff(t)
    assert len(t) > 1 and np.allclose(dt, 0.005, atol=1e-10)
    demand = np.array([r["pd_torque_Nm"] for r in rows])[:-1]
    applied = np.array([r["preceding_applied_torque_Nm"] for r in rows])[1:]
    assert demand.shape == applied.shape == (len(dt), 19)
    assert np.isfinite(demand).all() and np.isfinite(applied).all()
    # Applied torque in the next recorded state belongs to the preceding 5ms command.
    names = [n.replace("arm_", "arm0_") for n in config["simulation_joint_names"]]
    ids = [names.index(n) for n in JOINTS]
    roundoff = pd_roundoff_bound(
        np.array([r["q"] for r in rows])[:-1, ids],
        np.array([r["dq"] for r in rows])[:-1, ids],
        config["manifest"]["gains"]["kp"],
        config["manifest"]["gains"]["kd"],
    )
    difference = np.abs(demand - applied)
    saturation = difference > roundoff + 1e-5
    hips = [i for i, n in enumerate(JOINTS[:12]) if not n.endswith("_kn")]
    unlimited = config.get("actuator_torque_limits") is False
    if unlimited:
        assert not saturation.any(), "Unlimited actuator torque did not match requested demand"
    else:
        assert np.max(np.abs(applied[:, hips])) <= 45.0 + 1e-5
    w, x, y, z = np.array([r["root_quaternion_wxyz"] for r in rows]).T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    omega = np.array([r["angular_velocity"] for r in rows])
    omega_norm = np.linalg.norm(omega, axis=1)
    forces = np.array([r["physics_diagnostic"]["foot_normal_force_w_N"] for r in rows])[:, :, 2]
    ik = np.array(
        [np.nan if r.get("ik_tracking_error_m") is None else r["ik_tracking_error_m"] for r in rows]
    )
    phase = np.array([r.get("demo_phase") for r in rows])
    hold = np.isin(phase, ["three-leg-hold", "three-leg-ik-motion"])
    motion = phase == "three-leg-ik-motion"
    entry = (t >= 5) & (t <= 8)
    entry_intervals = entry[:-1]
    contacts = []
    for leg, j in zip(["fl", "fr", "hl", "hr"], range(4)):
        contacts.append(
            dict(
                leg=leg,
                normal_force_N=summary_values(forces[:, j]),
                hold_loaded_fraction=float(np.mean(forces[hold, j] > 5)) if np.any(hold) else None,
            )
        )
    joint_stats = []
    for j, name in enumerate(JOINTS):
        joint_stats.append(
            dict(
                joint=name,
                requested_peak_abs_Nm=float(np.max(np.abs(demand[:, j]))),
                applied_peak_abs_Nm=float(np.max(np.abs(applied[:, j]))),
                saturation_duration_s=float(np.sum(dt * saturation[:, j])),
                entry_saturation_duration_s=float(
                    np.sum(dt[entry_intervals] * saturation[entry_intervals, j])
                ),
            )
        )
    demo = result.get("three_leg_demo") or {}
    passed = bool(
        result["reason"] == "completed"
        and result.get("first_fall_proxy_s") is None
        and demo.get("passed")
        and result["stable_final_window"]
    )
    event_lines = (run / "events.jsonl").read_text().splitlines()
    guard_events = [
        json.loads(line)
        for line in event_lines
        if "disabled_guard" in line or "disabled_operating_check" in line
    ]
    guards = [
        dict(
            reason=e.get("reason"),
            simulation_time_s=e.get("simulation_time_s"),
            monotonic_s=e.get("monotonic_s"),
        )
        for e in guard_events
    ]
    summary = dict(
        scientific_transition_passed=passed,
        execution_completed=result["reason"] == "completed",
        result=result,
        source_commit=json.loads((run / "source.json").read_text()).get("commit"),
        sampling_hz=200,
        actuator_torque_limits=not unlimited,
        alignment="demand[k] vs preceding_applied[k+1]; 5ms interval",
        saturation_difference_threshold_Nm=1e-5,
        recording_precision=dict(
            state_dtype="float32",
            simulator_state_dtype="float64",
            rule="abs(request-applied) > half-ULP(q)*kp + half-ULP(dq)*kd + 1e-5 Nm",
            max_raw_difference_Nm=float(difference.max()),
            raw_pairs_above_1e_5=int((difference > 1e-5).sum()),
            max_excess_above_roundoff_bound_Nm=float((difference - roundoff).max()),
        ),
        foot_loaded_diagnostic_threshold_N=5,
        joint_torque=joint_stats,
        body=dict(
            max_abs_roll_deg=float(np.degrees(np.abs(roll).max())),
            max_abs_pitch_deg=float(np.degrees(np.abs(pitch).max())),
            max_angular_speed_rad_s=float(omega_norm.max()),
            entry_max_abs_roll_deg=float(np.degrees(np.abs(roll[entry]).max())),
            entry_max_abs_pitch_deg=float(np.degrees(np.abs(pitch[entry]).max())),
            entry_max_angular_speed_rad_s=float(omega_norm[entry].max()),
        ),
        foot_contacts=contacts,
        ik_motion_error_m=summary_values(ik[motion]) if np.any(motion) else None,
        observed_guard_crossings=guards,
        guard_crossings_deduplicated_by_reason=True,
        hardware_access=False,
        hardware_qualified=False,
        input_sha256={
            n: hashlib.sha256((run / n).read_bytes()).hexdigest()
            for n in ["result.json", "rollout.jsonl", "configuration.json", "COMPLETE.json"]
        },
    )
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(
        output / "aligned_metrics.npz",
        time_s=t,
        requested_Nm=demand,
        applied_Nm=applied,
        saturated=saturation,
        roll_rad=roll,
        pitch_rad=pitch,
        angular_body_rad_s=omega,
        foot_normal_N=forces,
        ik_error_m=ik,
    )
    fig, axes = plt.subplots(4, 2, figsize=(13, 13), sharex=True, layout="constrained")
    axes[0, 0].plot(t, np.degrees(roll), label="Roll")
    axes[0, 0].plot(t, np.degrees(pitch), label="Pitch")
    axes[0, 0].set_ylabel("Body attitude (degrees)")
    axes[0, 0].legend()
    axes[0, 1].plot(t, omega_norm)
    axes[0, 1].set_ylabel("Body angular speed (rad/s)")
    for j, leg in enumerate(["FL", "FR", "HL", "HR"]):
        axes[1, 0].plot(t, forces[:, j], label=leg)
    axes[1, 0].set_ylabel("Foot normal force (N)")
    axes[1, 0].legend(ncol=4)
    axes[1, 1].plot(t, ik * 100)
    axes[1, 1].axhline(3, color="red", ls="--", label="IK p95 criterion")
    axes[1, 1].set_ylabel("IK position error (cm)")
    axes[2, 0].plot(t[:-1], np.max(abs(demand[:, :12]), axis=1), label="Requested")
    axes[2, 0].plot(t[:-1], np.max(abs(applied[:, :12]), axis=1), label="Applied")
    axes[2, 0].legend()
    axes[2, 0].set_ylabel("Peak leg torque (Nm)")
    axes[2, 1].plot(t[:-1], saturation[:, :12].sum(axis=1))
    axes[2, 1].set_ylabel("Saturated leg joints (count)")
    axes[3, 0].plot(t, [r["height"] for r in rows])
    axes[3, 0].set_ylabel("Body height (m)")
    radius = config["demonstration"]["foot_radius_m"]
    for j, leg in enumerate(["FL", "FR", "HL", "HR"]):
        axes[3, 1].plot(t, np.array([r["feet"][j][2] for r in rows]) - radius, label=leg)
    axes[3, 1].set_ylabel("Foot clearance (m)")
    for ax in axes.flat:
        ax.grid(alpha=0.2)
        for boundary in [5, 11, 14, 26, 32, 34]:
            ax.axvline(boundary, color="gray", alpha=0.2, lw=0.7)
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time (s)")
    label = "PASS" if passed else "FAIL"
    fig.suptitle(
        f"Native unfiltered transition: {label} · simulation only\n"
        + (
            "UNLIMITED ACTUATOR TORQUE"
            if unlimited
            else "Operating stops disabled; actuator saturation retained"
        )
    )
    for ext in ["png", "pdf"]:
        fig.savefig(output / f"recovery.{ext}", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(4, 3, figsize=(13, 10), sharex=True, layout="constrained")
    window = (t[:-1] >= 4.8) & (t[:-1] <= 8)
    for j, ax in enumerate(axes.flat):
        ax.plot(t[:-1][window], demand[window, j], label="Requested")
        ax.plot(t[:-1][window], applied[window, j], label="Applied")
        ax.set_title(JOINTS[j])
        ax.set_ylabel("Nm")
        ax.grid(alpha=0.2)
        ax.axvline(5, color="gray", ls="--")
    axes[0, 0].legend()
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time (s)")
    fig.suptitle("Lift entry: requested and time-aligned applied joint torques")
    for ext in ["png", "pdf"]:
        fig.savefig(output / f"entry-torques.{ext}", dpi=160)
    plt.close(fig)
    print(
        json.dumps(
            dict(
                scientific_transition_passed=passed,
                first_fall_proxy_s=result.get("first_fall_proxy_s"),
                sequence=demo,
                body=summary["body"],
                ik_motion_error_m=summary["ik_motion_error_m"],
            )
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    main(args.run, args.output)
