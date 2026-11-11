"""Gated offline calibration campaign. Fit never opens joystick recordings."""

import argparse
import itertools
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from score_sysid import bandwidth, score
from spot_deploy.contracts import JOINTS, sha256, canonical_hash
from spot_deploy.relic_contract import load_manifest
from spot_deploy.sysid import PAPER, distances, extract, features

ROOT = Path(__file__).resolve().parents[1]
TRAIN = ("forward", "six", "random")
# Physical model search box (simulation only); non-overlapping breakpoints and extrema.
BOUNDS = np.array(
    [[0, 0.10], [0, 1.0], [60, 120], [-130, -60], [15, 35], [-35, -15], [0, 14], [-14, 0]]
)
SEED = 101


def save(path, x):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(x, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def dataset(folder, stage):
    study = json.loads((folder / "study.json").read_text())
    mapping = json.loads((folder / "recordings.json").read_text())
    if study["training"] != list(TRAIN) or study["holdout"] != "joystick" or study["repeats"] != 5:
        raise ValueError("incorrect study split")
    groups = {}
    ids = set()
    graph = set()
    for family in TRAIN if stage == "fit" else ("joystick",):
        spec = study["families"][family]
        trace = folder / spec["trace"]
        if sha256(trace) != spec["sha256"]:
            raise ValueError("frozen sequence changed")
        cmds = np.array(json.loads(trace.read_text())["velocity_command"])
        entries = mapping[family]
        if len(entries) != 5 or not all(entries):
            raise ValueError(f"{family} requires five separately recorded trials")
        groups[family] = []
        for e in entries:
            run = (folder / e["trial"]).resolve()
            mf = (folder / e["manifest"]).resolve()
            hw, _, meta = extract(run, mf)
            if meta["hardware_status"] != "passed":
                raise ValueError(f"failed capture {run}; preserve failure, no fit")
            key = meta["hardware_events_sha256"]
            if key in ids:
                raise ValueError("duplicate trial")
            ids.add(key)
            m = load_manifest(mf)
            graph.add(
                canonical_hash(
                    dict(
                        graph=m.policy_sha256,
                        gains=m.gains.model_dump(),
                        actions=m.actions.model_dump(),
                        stow=m.arm_stowed_positions,
                        preparation=m.relic.preparation.model_dump(),
                    )
                )
            )
            if cmds.shape != hw["cmd"].shape or not np.allclose(cmds, hw["cmd"], rtol=0, atol=1e-5):
                raise ValueError("effective command does not match registered trace")
            ix = [JOINTS.index(n) for n in m.actions.joint_order]
            x = features(
                hw["q"],
                hw["dq"],
                hw["a"],
                np.array(m.gains.kp)[ix],
                np.array(m.gains.kd)[ix],
                np.array(m.actions.scale),
            )
            groups[family].append(
                (run, mf, x[np.linspace(0, len(x) - 1, min(256, len(x)), dtype=int)], key)
            )
    if len(graph) != 1:
        raise ValueError("policy graph changed across recordings")
    return groups, graph.pop()


def simulate(out, exe, groups, theta, backend, width):
    vals = []
    details = []
    out.mkdir(parents=True, exist_ok=True)
    param = theta
    if not isinstance(theta, str):
        f = out / "parameters.json"
        if f.exists() and json.loads(f.read_text()) != list(theta):
            raise ValueError("inconsistent candidate on resume")
        save(f, list(theta))
        param = str(f.resolve())
    for fam, entries in groups.items():
        for i, (run, mf, _, _) in enumerate(entries):
            cell = out / f"{fam}-{i}"
            if (cell / "score.json").is_file():
                s = json.loads((cell / "score.json").read_text())
                if s["command_trace_sha256"] != entries[i][3] or s["bandwidth"] != width:
                    raise ValueError("cached cell bindings have changed")
            else:
                if cell.exists():
                    raise ValueError(f"partial cell {cell}; inspect and archive it before resuming")
                with (out / f"{fam}-{i}.log").open("x") as log:
                    subprocess.run(
                        [
                            str(exe),
                            str(ROOT / "tools/sysid_replay.py"),
                            "--backend",
                            backend,
                            "--parameters",
                            param,
                            "--trial",
                            str(run),
                            "--manifest",
                            str(mf),
                            "--output",
                            str(cell),
                        ],
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        check=True,
                        timeout=240,
                        env=dict(
                            os.environ,
                            PYTHONPATH=str(ROOT / "src")
                            + ":"
                            + str(ROOT / "simulation/source/relic"),
                        ),
                    )
                s = score(cell, mf, width)
                save(cell / "score.json", s)
            if not s["complete"]:
                return None, details + [{"family": fam, "repeat": i, "fall_s": s["fall_s"]}]
            vals.append([s["all"]["mean_w1"], s["all"]["mmd2"]])
            details.append(dict(family=fam, repeat=i, score=vals[-1], walking=s.get("walking")))
    return np.mean(vals, axis=0), details


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("stage", choices=["fit", "validate"])
    p.add_argument("--study", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--sim-python", type=Path, required=True)
    p.add_argument("--fit", type=Path)
    p.add_argument("--backend", choices=["isaaclab", "mujoco"], default="isaaclab")
    a = p.parse_args()
    if a.stage == "fit" and a.backend != "isaaclab":
        p.error("fit in Isaac Lab only")
    groups, graph = dataset(a.study, a.stage)
    a.output.mkdir(parents=True, exist_ok=True)
    fixed = dict(
        study_sha256=sha256(a.study / "study.json"),
        graph=graph,
        training_source=sha256(ROOT / "tools/sysid_replay.py"),
        scorer=sha256(ROOT / "tools/score_sysid.py"),
        estimator_module=sha256(ROOT / "src/spot_deploy/sysid.py"),
    )
    identity = dict(
        **fixed,
        stage=a.stage,
        backend=a.backend,
        fit_input=sha256(a.fit) if a.fit else None,
        driver_sha256=sha256(Path(__file__)),
        interpreter=str(a.sim_python.resolve()),
        trials={k: [(v[3], sha256(v[1])) for v in vs] for k, vs in groups.items()},
    )
    binding = a.output / "campaign.json"
    if binding.exists():
        if json.loads(binding.read_text()) != json.loads(json.dumps(identity)):
            raise ValueError("campaign inputs or source changed; use a new output directory")
    else:
        if any(a.output.iterdir()):
            raise ValueError("unbound nonempty campaign directory")
        save(binding, identity)
    if a.stage == "validate":
        if not a.fit:
            p.error("frozen fit required")
        prior = json.loads(a.fit.read_text())
        if any(prior[k] != v for k, v in fixed.items()):
            raise ValueError("validation contract changed")
        if set(prior["training_ids"]) & {e[3] for e in groups["joystick"]}:
            raise ValueError("holdout leakage")
        results = {}
        for name, theta in [
            ("baseline", "baseline"),
            ("paper", "paper"),
            ("fitted", prior["parameters"]),
        ]:
            v, rows = simulate(
                a.output / name, a.sim_python, groups, theta, a.backend, prior["bandwidth"]
            )
            results[name] = dict(mean=v.tolist() if v is not None else None, rows=rows)
        pairs = []
        for left, right in itertools.combinations(groups["joystick"], 2):
            d = distances(left[2], right[2], prior["bandwidth"])
            pairs.append([d["mean_w1"], d["mmd2"]])
        variation = np.median(pairs, axis=0)
        all_complete = all(r["mean"] is not None for r in results.values())
        beats = all_complete and all(
            np.all(
                np.array(results[name]["mean"]) - np.array(results["fitted"]["mean"]) > variation
            )
            for name in ("baseline", "paper")
        )
        save(
            a.output / "validation.json",
            dict(
                results=results,
                heldout_real_repeat_distances=pairs,
                distribution_improvement_beyond_median_repeat_distance=bool(beats),
                automatic_promotion=False,
                note="Check paired gains against within-hardware variability and gait/contacts before acceptance.",
            ),
        )
        return
    from cmaes import CMA

    width = bandwidth(next(iter(groups.values()))[0][2])

    def evaluate(tag, params):
        v, rows = simulate(a.output / tag, a.sim_python, groups, params, "isaaclab", width)
        save(a.output / tag / "scores.json", rows)
        return v

    repeat = []
    for entries in groups.values():
        for left, right in itertools.combinations(entries, 2):
            d = distances(left[2], right[2], width)
            repeat.append([d["mean_w1"], d["mmd2"]])
    noise = np.median(repeat, axis=0)
    base = evaluate("baseline", "baseline")
    paper = evaluate("paper", "paper")
    if base is None or paper is None:
        raise ValueError("baseline fall; inspect before fitting")
    denom = np.maximum(base, 1e-9)
    x = np.array(PAPER)
    active = []
    for j, (lo, hi) in enumerate(BOUNDS):
        probe = x.copy()
        probe[j] += 0.15 * (hi - lo) if x[j] + 0.15 * (hi - lo) <= hi else -0.15 * (hi - lo)
        y = evaluate(f"sensitivity-{j}", probe)
        if y is not None and np.any(abs(y - paper) > np.maximum(noise, 1e-8)):
            active.append(j)
    best = float(np.mean(paper / denom))
    best_x = x.copy()
    save(
        a.output / "screen.json",
        dict(
            active=active, repeat_noise=noise.tolist(), baseline=base.tolist(), paper=paper.tolist()
        ),
    )
    if active:
        lo, hi = BOUNDS[active].T
        mean = (x[active] - lo) / (hi - lo)
        # cmaes requires two coordinates. A neutral coordinate never enters the physics.
        if len(active) == 1:
            mean = np.append(mean, 0.5)
        optim = CMA(
            mean=mean,
            sigma=0.15,
            bounds=np.tile([0.0, 1.0], (len(mean), 1)),
            population_size=10,
            seed=SEED,
        )
        for generation in range(100):
            sols = []
            for ind in range(10):
                point = optim.ask()
                proposal = x.copy()
                proposal[active] = lo + point[: len(active)] * (hi - lo)
                v = evaluate(f"gen-{generation:03}-{ind}", proposal)
                loss = float(np.mean(v / denom)) if v is not None else 1e9
                sols.append((point, loss))
                if loss < best:
                    best = loss
                    best_x = proposal.copy()
            optim.tell(sols)
    save(
        a.output / "fit.json",
        dict(
            **fixed,
            parameters=best_x.tolist(),
            active=active,
            loss=best,
            bandwidth=width,
            seed=SEED,
            training_ids=[v[3] for rows in groups.values() for v in rows],
        ),
    )


if __name__ == "__main__":
    main()
