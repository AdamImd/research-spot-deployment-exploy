"""Offline distributional identification for frozen ReLIC. No robot I/O."""

import json
from pathlib import Path
import numpy as np
from .contracts import JOINTS, sha256
from .records import verify_run
from .relic_contract import load_manifest

PAPER = [0.008, 0.180, 97.0, -108.79, 25.03, -22.22, 9.48, -8.32]


def validate_parameters(values):
    x = np.asarray(values, dtype=float)
    if x.shape != (8,) or not np.isfinite(x).all():
        raise ValueError("eight finite actuator parameters required")
    h, k, p, n, v, w, b, c = x
    if not (min(h, k) >= 0 and 0 < p and n < 0 and 0 <= b < v and w < c <= 0):
        raise ValueError("invalid signed torque-speed envelope")


def curves(values):
    validate_parameters(values)
    _, _, p, n, v, w, b, c = values
    return np.array([[w, p], [b, p], [v, 0]]), np.array([[w, 0], [c, n], [v, n]])


def extract(trial, manifest):
    trial = Path(trial)
    m = load_manifest(manifest)
    status = json.loads((trial / "COMPLETE.json").read_text())["status"]
    if status not in ("passed", "failed") or not verify_run(trial, status=status):
        raise ValueError("run integrity failed")
    inputs = json.loads((trial / "run.json").read_text())["inputs"]
    if sha256(manifest) not in inputs.values():
        raise ValueError("manifest not bound to recording")
    if (m.stream_hz, m.policy_hz) != (200, 50):
        raise ValueError("expected a 200/50 Hz recording")
    ix = [JOINTS.index(n) for n in m.actions.joint_order]
    rows = []
    acks = []
    height = None
    first = None
    previous = np.zeros(12)
    for line in (trial / "events.jsonl").open():
        e = json.loads(line)
        if e["event"] == "relic_initialized":
            height = e["requested_height_m"]
        if e["event"] == "policy_ack":
            previous = np.array(e["raw_actions"])
            acks.append(e["command_key"])
        if e["event"] != "policy":
            continue
        st = e["state"]
        obs = np.array(e["observations"])
        if first is None:
            first = st
        np.testing.assert_allclose(obs[-12:], previous, rtol=0, atol=1e-6)
        np.testing.assert_allclose(obs[9:12], e["velocity_command"], rtol=0, atol=1e-6)
        np.testing.assert_allclose(obs[12:19], m.arm_stowed_positions, rtol=0, atol=1e-6)
        rows.append(
            (
                e["monotonic_s"],
                np.array(st["positions"])[ix],
                np.array(st["velocities"])[ix],
                e["raw_actions"],
                obs[:3],
                e["velocity_command"],
            )
        )
    if len(rows) < 2 or height is None or not acks:
        raise ValueError("incomplete policy trace")
    hw = {k: np.stack(v) for k, v in zip(("t", "q", "dq", "a", "v", "cmd"), zip(*rows))}
    hw["t"] -= hw["t"][0]
    err = np.max(abs(hw["t"] - np.arange(len(rows)) * 0.02))
    if (
        err >= 0.01
        or np.any(np.diff(hw["t"]) <= 0)
        or not all(np.isfinite(v).all() for v in hw.values())
    ):
        raise ValueError("missing/ambiguous policy tick or non-finite trace")
    return (
        hw,
        first,
        dict(
            height=height,
            hardware_samples=len(rows),
            hardware_status=status,
            hardware_result=json.loads((trial / "result.json").read_text()).get("error"),
            hardware_events_sha256=sha256(trial / "events.jsonl"),
            manifest_sha256=sha256(manifest),
            max_grid_error_s=float(err),
            ack_samples=len(acks),
        ),
    )


def features(q, dq, a, kp, kd, scale):
    return np.concatenate((q * kp, dq * kd, a * kp * scale), axis=1)


def distances(x, y, bandwidth):
    """Equal-size uniform-in-time empirical W1 per channel and biased RBF MMD²."""
    if (
        x.shape != y.shape
        or x.ndim != 2
        or len(x) < 2
        or not np.isfinite(bandwidth)
        or bandwidth <= 0
    ):
        raise ValueError("invalid score inputs")
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("nonfinite input")
    w1 = np.abs(np.sort(x, axis=0) - np.sort(y, axis=0)).mean(0)

    def kernel(a, b):
        d = (a * a).sum(1)[:, None] + (b * b).sum(1)[None, :] - 2 * a @ b.T
        return np.exp(-np.maximum(0, d) / (2 * bandwidth**2)).mean()

    return dict(
        mean_w1=float(w1.mean()),
        w1_per_feature=w1.tolist(),
        mmd2=float(max(0, kernel(x, x) + kernel(y, y) - 2 * kernel(x, y))),
    )
