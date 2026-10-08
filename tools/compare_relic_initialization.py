"""One-off, offline comparison of zero and measured-pose ReLIC action seeds."""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, ContractError, State
from spot_deploy.records import RunRecord, atomic_json
from spot_deploy.relic_policy import (
    ACTION_IDS, ACTION_JOINTS, CHECKPOINT_SHA256, DEFAULT_Q, ReLICPolicy, joint_targets,
)


def measured_seed(state):
    """Invert the released action transform, in raw policy action order."""
    q = np.asarray(state.positions, dtype=np.float32)
    seed = (q[ACTION_IDS] - DEFAULT_Q[ACTION_IDS]) / np.float32(.2)
    np.testing.assert_allclose(joint_targets(seed, q[12:]), q, atol=1e-6, rtol=0)
    return seed


def plot_comparison(output, times, gap, delta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(9, 6.2), layout="constrained", sharex=True)
    for mode, label, color in (("zero", "Zero initial action", "#52677e"),
                               ("measured", "Measured-pose initial action", "#ba791b")):
        axes[0].plot(times, np.max(np.abs(gap[mode]), axis=1), label=label, color=color)
    axes[0].set_ylabel("Largest target–measured gap (°)")
    axes[0].legend(frameon=False)
    axes[1].plot(times, delta, color="#6c4caa", label="Largest difference between initializations")
    axes[1].axhline(1, color="#8d939a", ls="--", lw=1, label="1° threshold")
    axes[1].set_ylabel("Target difference (°)")
    axes[1].set_xlabel("Recorded time since initialization (s)")
    axes[1].legend(frameon=False)
    for ax in axes:
        ax.grid(alpha=.2)
        ax.set_ylim(bottom=0)
    fig.suptitle("ReLIC action-history initialization · paired recorded-state replay")
    fig.savefig(output / "comparison.png", dpi=150)
    plt.close(fig)
    return matplotlib.__version__


def compare(window, checkpoint, urdf, output):
    samples = [json.loads(line) for line in window.read_text().splitlines()]
    if not 2 <= len(samples) <= 1000 or any(s.get("event") != "shadow_sample" for s in samples):
        raise ContractError("comparison requires 2–1000 complete shadow samples")
    states = [State.model_validate(s["state"]) for s in samples]
    times = np.asarray([s.robot_time_s for s in states])
    if np.any(np.diff(times) <= 0) or np.any(np.diff(times) > .1):
        raise ContractError("comparison input timestamps are not contiguous fresh samples")
    times -= times[0]
    seed = measured_seed(states[0])
    inputs = [window, checkpoint, urdf, Path(__file__)]
    record = RunRecord(output, "relic-initialization-comparison", inputs)
    children = []
    try:
        atomic_json(output / "seed.json", {
            "action_joint_names": ACTION_JOINTS, "raw_seed": seed.tolist(),
            "initial_joint_positions_sdk": states[0].positions,
            "formula": "(q_measured - q_default) / 0.2 in action_joint_names order",
            "scope": "set once before first inference; normal previous-output feedback thereafter",
        })
        targets = {}
        for mode in ("zero", "measured"):
            policy = ReLICPolicy(checkpoint)
            if mode == "measured":
                policy.previous_action = seed.copy()
            child = RunRecord(output / mode, "relic-initialization-replay", inputs)
            children.append(child)
            shutil.copy2(urdf, child.directory / "robot.urdf")
            atomic_json(child.directory / "policy.json", {
                "name": f"ReLIC · {mode} initial action · one-off replay",
                "policy_hz": 50, "checkpoint_sha256": CHECKPOINT_SHA256,
                "action_joint_names": ACTION_JOINTS, "velocity_command": [0, 0, 0],
                "mode": "four-foot", "torso_target": [0, 0, .55],
                "arm_mode": "Initial measured arm pose held; arm is not predicted",
                "note": f"Recorded paired test, initial action={mode}; zero commands sent.",
            })
            outputs = []
            for index, state in enumerate(states):
                previous = policy.previous_action.copy()
                target, elapsed, observation, action = policy.predict(state)
                np.testing.assert_array_equal(observation[72:], previous)
                np.testing.assert_array_equal(policy.previous_action, action)
                child.event("shadow_sample", source_time=samples[index].get("time"),
                            state=state.model_dump(), observations=observation.tolist(),
                            raw_actions=action.tolist(), targets=target.tolist(), inference_s=elapsed)
                outputs.append(target)
            targets[mode] = np.asarray(outputs)
            child.finish("passed", {"samples": len(states), "motion_commands": 0,
                                    "initial_action": mode, "history_checks_passed": True})
        q = np.asarray([s.positions for s in states])
        gap = {mode: np.rad2deg(values[:, :12] - q[:, :12]) for mode, values in targets.items()}
        delta = np.max(np.abs(np.rad2deg(targets["measured"][:, :12] -
                                          targets["zero"][:, :12])), axis=1)
        settled = np.flatnonzero(np.maximum.accumulate(delta[::-1])[::-1] < 1)
        last_second = times >= times[-1] - 1
        result = {
            "samples": len(states), "duration_s": float(times[-1]), "deterministic": True,
            "checkpoint_sha256": CHECKPOINT_SHA256, "motion_commands": 0,
            "initial_raw_action_range": [float(seed.min()), float(seed.max())],
            "first_step": {mode: {
                "max_abs_target_minus_measured_deg": float(np.abs(gap[mode][0]).max()),
                "rms_target_minus_measured_deg": float(np.sqrt(np.mean(gap[mode][0] ** 2))),
            } for mode in targets},
            "first_target_difference_max_deg": float(delta[0]),
            "last_second_target_difference_max_deg": float(delta[last_second].max()),
            "difference_stays_below_one_degree_from_s": float(times[settled[0]]) if settled.size else None,
            "first_joint_targets": [{"joint": JOINTS[i], "measured_deg": float(np.rad2deg(q[0, i])),
                "zero_init_target_deg": float(np.rad2deg(targets["zero"][0, i])),
                "measured_init_target_deg": float(np.rad2deg(targets["measured"][0, i]))}
                for i in range(12)],
            "interpretation_limit": "Recorded-state shadow replay; no physical response or closed-loop validation",
        }
        np.savez(output / "curves.npz", time_s=times, measured_q=q,
                 zero_targets=targets["zero"], measured_init_targets=targets["measured"],
                 zero_gap_deg=gap["zero"], measured_gap_deg=gap["measured"], target_difference_deg=delta)
        result["matplotlib_version"] = plot_comparison(output, times, gap, delta)
        record.finish("passed", result)
        return result
    except BaseException as exc:
        result = {"error_type": type(exc).__name__, "motion_commands": 0}
        for child in children:
            child.finish("failed", result)
        record.finish("failed", result)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(compare(args.window, args.checkpoint, args.urdf, args.output), indent=2))


if __name__ == "__main__":
    main()
