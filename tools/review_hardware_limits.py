"""Offline demand audit of completed direct-start simulations; never creates an envelope.

The output contains model limits and measured simulation demand, not hardware
qualification. It deliberately omits robot identity and raw hardware records.
"""

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from spot_deploy.contracts import JOINTS


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read(path):
    return json.loads(path.read_text())


def require(condition, message):
    if not condition:
        raise ValueError(message)


def summarize(run, duration):
    complete = read(run / "COMPLETE.json")
    require(complete["status"] == "completed", "run did not complete")
    required = {"configuration.json", "rollout.jsonl", "events.jsonl", "result.json",
                "source.json", "run.json", "timing.jsonl"}
    require(required <= complete["artifacts"].keys(), "missing provenance hashes")
    for name, expected in complete["artifacts"].items():
        path = (run / name).resolve()
        require(path.is_relative_to(run.resolve()), "artifact outside run")
        require(digest(path) == expected, f"artifact hash mismatch: {name}")
    config = read(run / "configuration.json")
    require(config["hardware_access"] is False, "simulation only")
    require(config["preparation_s"] == 0, "this audit requires direct startup")
    require(config["envelope"]["joint_order"] == list(JOINTS), "SDK order mismatch")
    names = [n.replace("arm_", "arm0_") for n in config["simulation_joint_names"]]
    require(len(names) == 19 and set(names) == set(JOINTS), "native joint mapping")
    order = [names.index(n) for n in JOINTS]
    rows = [json.loads(line) for line in (run / "rollout.jsonl").open()]
    events = [json.loads(line) for line in (run / "events.jsonl").open()]
    handover, = [e for e in events if e["event"] == "relic_handover"]
    holds = [e for e in events if e["event"] == "preparation"
             and e["monotonic_s"] < handover["monotonic_s"]]
    hold = holds[-1]
    require(handover["previous_actions"] == [0.] * 12, "nonzero initial history")
    time = np.asarray([r["trial_time"] for r in rows])
    require(time[0] == 0 and time[-1] >= duration, "incomplete requested window")
    dt = 1 / config["command_hz"]
    np.testing.assert_allclose(np.diff(time), dt, atol=1e-9, rtol=0)
    handover_dt = handover["monotonic_s"] - hold["monotonic_s"]
    np.testing.assert_allclose(handover_dt, dt, atol=1e-9, rtol=0)
    q = np.asarray([r["q"] for r in rows])[:, order]
    dq = np.asarray([r["dq"] for r in rows])[:, order]
    target = np.asarray([r["positions"] for r in rows])
    torque = np.asarray([r["pd_torque_Nm"] for r in rows])
    applied = np.asarray([r["preceding_applied_torque_Nm"] for r in rows])
    ff = np.asarray([r["feedforward"] for r in rows])
    previous_target = np.vstack((hold["reference"], target[:-1]))
    slew = np.abs(target - previous_target) / dt
    tracking = np.maximum(np.abs(target - q), np.abs(previous_target - q))
    for values in (time, q, dq, target, torque, applied, ff):
        require(np.isfinite(values).all(), "nonfinite trace")
    gains = config["manifest"]["gains"]
    np.testing.assert_allclose(torque, np.asarray(gains["kp"]) * (target - q)
                               - np.asarray(gains["kd"]) * dq + ff, atol=1e-5, rtol=0)
    np.testing.assert_allclose(torque[0] - hold["feedforward"],
                               handover["torque_step_Nm"], atol=1e-5, rtol=0)
    height = np.asarray([r["height"] for r in rows])
    result = {}
    for label, end in (("first_trial", duration), ("full_trace", time[-1])):
        selected = time <= end + 1e-9
        joints = {}
        for i, name in enumerate(JOINTS):
            speed_peak = int(np.argmax(np.abs(dq[selected, i])))
            joints[name] = {
                "position_min_rad": float(q[selected, i].min()),
                "position_max_rad": float(q[selected, i].max()),
                "target_min_rad": float(target[selected, i].min()),
                "target_max_rad": float(target[selected, i].max()),
                "speed_max_rad_s": float(np.abs(dq[selected, i]).max()),
                "speed_peak_time_s": float(time[selected][speed_peak]),
                "requested_load_max_Nm": float(np.abs(torque[selected, i]).max()),
                "preceding_applied_load_max_Nm": float(np.abs(applied[selected, i]).max()),
                "tracking_error_max_rad": float(tracking[selected, i].max()),
                "target_rate_max_rad_s": float(slew[selected, i].max()),
            }
        result[label] = {
            "duration_s": float(end), "samples": int(selected.sum()), "joints": joints,
            "body_height_drop_max_m": float(height[0] - height[selected].min()),
            "tilt_max_deg": float(np.rad2deg(max(r["tilt"] for r, s in
                                                       zip(rows, selected) if s))),
            "linear_speed_max_m_s": float(max(np.linalg.norm(r["linear_velocity"])
                                               for r, s in zip(rows, selected) if s)),
            "angular_speed_max_rad_s": float(max(np.linalg.norm(r["angular_velocity"])
                                                  for r, s in zip(rows, selected) if s)),
            "coupled_sh1_minus_el0_load_max_Nm": float(
                np.abs(torque[selected, 13] - torque[selected, 14]).max()),
        }
    return {
        "backend": config["backend"], "seed": config["seed"],
        "command_hz": config["command_hz"], "policy_hz": config["policy_hz"],
        "artifacts_sha256": complete["artifacts"],
        "complete_sha256": digest(run / "COMPLETE.json"),
        "packages": read(run / "source.json")["packages"],
        "graph_sha256": config["manifest"]["policy_sha256"],
        "stable_final_window": read(run / "result.json")["stable_final_window"],
        "handover_torque_step_Nm": handover["torque_step_Nm"],
        "supported_hold_feedforward_Nm": hold["feedforward"],
        "handover_target_rate_rad_s": slew[0].tolist(),
        "arm_stowed_positions": config["manifest"]["arm_stowed_positions"],
        **result,
    }


def model_limits(path):
    joints = {j.attrib["name"].replace(".", "_"): j for j in ET.parse(path).getroot()
              .findall("joint")}
    result = {}
    for name in JOINTS:
        limit = joints[name].find("limit")
        result[name] = {k: float(limit.attrib[k]) for k in ("lower", "upper", "effort", "velocity")}
    return result


def compare(summary, proposal):
    """Sampled magnitude comparison, not a replay of transport or runtime guards."""
    result = {}
    for window in ("first_trial", "full_trace"):
        violations = []
        for i, name in enumerate(JOINTS):
            limits = proposal["joints"][name]
            actual = summary[window]["joints"][name]
            for kind in ("position", "target"):
                for side, comparison in (("min", lambda x, y: x < y),
                                         ("max", lambda x, y: x > y)):
                    value = actual[f"{kind}_{side}_rad"]
                    bound = limits[f"position_{side}_rad"]
                    if comparison(value, bound):
                        violations.append({"joint": name, "metric": f"{kind}_{side}_rad",
                                           "observed": value, "proposed": bound})
            for metric in ("speed_max_rad_s", "tracking_error_max_rad", "target_rate_max_rad_s"):
                if actual[metric] > limits[metric]:
                    violations.append({"joint": name, "metric": metric,
                                       "observed": actual[metric], "proposed": limits[metric]})
            load = max(actual["requested_load_max_Nm"], actual["preceding_applied_load_max_Nm"],
                       abs(summary["supported_hold_feedforward_Nm"][i]))
            for metric, value in (("load_max_Nm", load), ("handover_step_max_Nm",
                                   abs(summary["handover_torque_step_Nm"][i]))):
                if value > limits[metric]:
                    violations.append({"joint": name, "metric": metric,
                                       "observed": value, "proposed": limits[metric]})
        result[window] = {"sampled_joint_bounds_pass": not violations, "violations": violations}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, action="append", required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--trial-duration", type=float, default=10.)
    parser.add_argument("--proposal", type=Path,
                        help="Optional review-only joint bounds; never an approved envelope")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    require(args.trial_duration > 0, "duration must be positive")
    require(not args.output.exists(), "output must be new")
    summaries = [summarize(run, args.trial_duration) for run in args.run]
    require(len({s["backend"] for s in summaries}) == len(summaries), "duplicate backend")
    result = {
        "status": "review_only", "hardware_qualified": False,
        "analysis_script_sha256": digest(Path(__file__)),
        "analysis_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True).strip(),
        "analysis_tree_dirty": bool(subprocess.check_output(
            ["git", "status", "--porcelain"], text=True).strip()),
        "python": platform.python_version(), "numpy": np.__version__,
        "joint_order": list(JOINTS),
        "urdf_sha256": digest(args.urdf), "model_limits": model_limits(args.urdf),
        "simulations": {s["backend"]: s for s in summaries},
    }
    if args.proposal:
        proposal = read(args.proposal)
        require(proposal["approval_status"] == "unreviewed", "expected unreviewed draft")
        require(set(proposal["joints"]) == set(JOINTS), "proposal joint mismatch")
        result["proposal_sha256"] = digest(args.proposal)
        result["sampled_joint_comparison"] = {s["backend"]: compare(s, proposal) for s in summaries}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"Verified {len(summaries)} completed traces; review only: {args.output}")


if __name__ == "__main__":
    main()
