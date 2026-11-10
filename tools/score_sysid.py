"""Score and plot an offline paired ReLIC replay. No simulator or SDK needed."""

import argparse
import json
from pathlib import Path
import numpy as np
from spot_deploy.sysid import features, distances
from spot_deploy.contracts import JOINTS, sha256
from spot_deploy.relic_contract import load_manifest


def bandwidth(x):
    # Median pair distance of 256 uniformly sampled training ticks; never use validation.
    delta = x[:, None, :] - x[None, :, :]
    ds = np.sqrt(np.sum(delta * delta, axis=-1))
    value = float(np.median(ds[np.triu_indices(len(x), 1)]))
    if value <= 0:
        raise ValueError("degenerate calibration feature distribution")
    return value


def peak_flexion_times(q, t):
    """Kinematic swing-phase proxy: prominent knee flexions separated by at least 160 ms."""
    events = []
    for joint in q.T:
        minima = np.flatnonzero((joint[1:-1] < joint[:-2]) & (joint[1:-1] <= joint[2:])) + 1
        keep = []
        for i in sorted(minima, key=lambda i: joint[i]):
            sides = min(joint[max(0, i - 8) : i].max(), joint[i + 1 : i + 9].max())
            if sides - joint[i] >= 0.12 and all(abs(i - j) >= 8 for j in keep):
                keep.append(i)
        events.append(t[sorted(keep)].tolist())
    return events


def score(run, manifest, width=None, plot=False):
    report = json.loads((run / "result.json").read_text())
    if sha256(manifest) != report["manifest_sha256"]:
        raise ValueError("manifest mismatch")
    m = load_manifest(manifest)
    ix = [JOINTS.index(n) for n in m.actions.joint_order]
    gains = (np.array(m.gains.kp)[ix], np.array(m.gains.kd)[ix], np.array(m.actions.scale))
    with np.load(run / "paired.npz") as data:
        d = dict(data)

    def feature(which):
        return features(*(d[which + "_" + k] for k in ("q", "dq", "a")), *gains)

    x = feature("hw")
    y = feature("sim")
    if width is None:
        ids = np.linspace(0, len(x) - 1, min(256, len(x)), dtype=int)
        width = bandwidth(x[ids])
    result = dict(
        bandwidth=width,
        kernel="biased Gaussian RBF MMD squared",
        fall_s=report["fall_s"],
        complete=report["fall_s"] is None and len(x) == len(y),
        hardware_status=report["hardware_status"],
        command_trace_sha256=report["hardware_events_sha256"],
        timing_grid_error_ms=report["max_grid_error_s"] * 1000,
    )
    n = len(y)
    move = np.flatnonzero(np.any(abs(d["hw_cmd"]) > 1e-6, axis=1))
    split = int(move[0]) if len(move) else len(x)
    for phase, lo, hi in [("all", 0, n), ("startup", 0, min(n, split)), ("walking", split, n)]:
        if hi - lo < 2:
            continue
        j = np.linspace(lo, hi - 1, min(256, hi - lo), dtype=int)
        result[phase] = dict(
            **distances(x[j], y[j], width),
            ticks=hi - lo,
            peak_knee_flexion_times_s={
                k: peak_flexion_times(d[k + "_q"][lo:hi, -4:], d[k + "_t"][lo:hi])
                for k in ("sim", "hw")
            },
            body_velocity_rmse=np.sqrt(
                ((d["hw_v"][lo:hi] - d["sim_v"][lo:hi]) ** 2).mean(0)
            ).tolist(),
            knee_peak_speed_hardware=np.max(abs(d["hw_dq"][lo:hi, -4:]), axis=0).tolist(),
            knee_peak_speed_sim=np.max(abs(d["sim_dq"][lo:hi, -4:]), axis=0).tolist(),
        )
    if plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(4, 2, figsize=(12, 8), sharex="col", layout="constrained")
        fig.suptitle(
            "Frozen ReLIC · forward command replay · right-front knee and forward body velocity"
        )
        for c, (lo, hi) in enumerate([(0, split), (split, len(x))]):
            axs[0, c].set_title(
                ["Startup / zero velocity request", "Walking / recorded velocity request"][c]
            )
            for row, (key, index, unit) in enumerate(
                [
                    ("q", 9, "Knee angle (rad)"),
                    ("dq", 9, "Knee speed (rad/s)"),
                    ("a", 9, "Raw action"),
                    ("v", 0, "Body velocity (m/s)"),
                ]
            ):
                ax = axs[row, c]
                ax.plot(
                    d["hw_t"][lo:hi],
                    d["hw_" + key][lo:hi, index],
                    lw=1,
                    color="#1b638e",
                    label="Spot",
                )
                ax.plot(
                    d["sim_t"][lo:hi],
                    d["sim_" + key][lo:hi, index],
                    lw=1,
                    color="#ce5918",
                    label=report["backend"],
                )
                if key == "v":
                    ax.plot(d["hw_t"][lo:hi], d["hw_cmd"][lo:hi, 0], "k:", label="command")
                if report["fall_s"] is not None:
                    ax.axvline(report["fall_s"], color="red", ls=":")
                if c == 0:
                    ax.set_ylabel(unit)
                ax.grid(alpha=0.2)
            axs[3, c].set_xlabel("Time since first policy decision (s)")
        axs[3, 1].legend()
        fig.savefig(run / "comparison.png", dpi=135)
        plt.close(fig)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("manifest", type=Path)
    p.add_argument("--bandwidth", type=float)
    a = p.parse_args()
    s = score(a.run, a.manifest, a.bandwidth, True)
    (a.run / "score.json").write_text(json.dumps(s, indent=2, allow_nan=False) + "\n")
    print(
        json.dumps(
            {k: s[k] for k in ("complete", "fall_s", "bandwidth")}
            | {k: s["all"][k] for k in ("mean_w1", "mmd2")}
        )
    )
