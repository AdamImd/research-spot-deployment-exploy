"""Audit completed shared-core simulations and preserve compact review evidence."""

import argparse
import gzip
import json
from pathlib import Path
import shutil

import numpy as np

from spot_deploy.contracts import sha256
from spot_deploy.records import atomic_json


def read(path):
    return json.loads(path.read_text())


def summarize(run, output, engines):
    output.mkdir(parents=True, exist_ok=False)
    provenance = read(run / "provenance.json")
    for group, base, key in (
        ("candidate", run / "candidate", "files_sha256"),
        ("capture", Path(provenance["capture"]["path"]), "artifacts_sha256"),
    ):
        for name, digest in provenance[group][key].items():
            assert sha256(base / name) == digest, (group, name)
    for name in ("RUN.md", "provenance.json", "COMPLETE.json"):
        shutil.copy2(run / name, output / name)
    summary = {}
    for engine in engines:
        source, destination = run / engine, output / engine
        destination.mkdir()
        complete = read(source / "COMPLETE.json")
        config = read(source / "configuration.json")
        names = config.get("simulation_joint_names")
        if names is None:
            reference = Path(read(source / "source.json")["simulation_source"])
            names = read(reference / "configs/articulation.json")["joint_names"]
        ids = [i for i, n in enumerate(names) if not n.startswith("arm_")]
        assert len(ids) == 12
        assert complete["status"] == "completed"
        for name, digest in complete["artifacts"].items():
            assert sha256(source / name) == digest, (engine, name)
        assert read(run / (engine + "-process") / "exit.json")["returncode"] == 0
        events = [json.loads(line) for line in (source / "events.jsonl").read_text().splitlines()]
        policies = [e for e in events if e["event"] == "policy"]
        commands = [e for e in events if e["event"] == "command"]
        acknowledged = np.zeros(12)
        pending = None
        for e in events:
            if e["event"] == "policy_ack":
                assert pending is not None and e["state_key"] >= e["command_key"]
                np.testing.assert_array_equal(e["raw_actions"], pending)
                acknowledged, pending = np.asarray(e["raw_actions"]), None
            if e["event"] == "policy":
                assert pending is None
                np.testing.assert_array_equal(e["observations"][-12:], acknowledged)
                pending = np.asarray(e["raw_actions"])
        np.testing.assert_array_equal(policies[0]["observations"][-12:], np.zeros(12))
        np.testing.assert_allclose(np.diff([e["monotonic_s"] for e in policies]), .02,
                                   rtol=0, atol=1e-10)
        np.testing.assert_allclose(np.diff([e["simulation_time_s"] for e in commands]), .005,
                                   rtol=0, atol=1e-10)
        np.testing.assert_array_equal(np.diff([e["key"] for e in commands]), 1)
        for e in commands:
            assert e["state"]["last_command_key"] == e["key"] - 1
            assert e["end_robot_time_s"] > e["state"]["robot_time_s"]
            if e["phase"] == "policy":
                np.testing.assert_array_equal(e["feedforward"], np.zeros(19))
        assert len(policies) == 2751 and len(commands) == 12001
        handover = next(e for e in events if e["event"] == "relic_handover")
        rows = [json.loads(line) for line in (source / "rollout.jsonl").read_text().splitlines()]
        first = next(r for r in rows if r["phase"] == "policy")
        # Simulator q/dq interleave arm joints; SDK targets/state are leg-major.
        startup = [r for r in rows if r["phase"] == "policy" and r["time"] <= 1 + 1e-10]
        first_gap = np.asarray(policies[0]["targets"])[:12] - policies[0]["state"]["positions"][:12]
        late_policies = [e for e in policies if e["monotonic_s"] >= policies[-1]["monotonic_s"] - 10 - 1e-10]
        summary[engine] = read(source / "result.json") | dict(
            history_error=0., command_count=len(commands),
            handover_max_torque_step_Nm=float(np.max(np.abs(handover["torque_step_Nm"]))),
            handover_height_m=first["height"], first_leg_target_gap_deg=float(np.rad2deg(np.abs(first_gap)).max()),
            first_second_height_drop_m=first["height"] - min(r["height"] for r in startup),
            first_second_max_reported_leg_speed_200hz_rad_s=float(np.abs(
                np.asarray([r["dq"] for r in startup])[:, ids]).max()),
            first_second_max_leg_excursion_deg=float(np.rad2deg(np.abs(
                np.asarray([r["q"] for r in startup])[:, ids] - np.asarray(first["q"])[ids])).max()),
            late_max_target_step_deg=float(np.rad2deg(np.abs(np.diff(
                [e["targets"][:12] for e in late_policies], axis=0))).max()),
            raw_paths_sha256={str((source / name).resolve()): sha256(source / name)
                              for name in ("events.jsonl", "rollout.jsonl")},
        )
        for name in ("result.json", "configuration.json", "source.json", "COMPLETE.json"):
            shutil.copy2(source / name, destination / name)
        shutil.copy2(run / (engine + "-process") / "exit.json", destination / "exit.json")
        # Retain all policy states, histories, acknowledgements and phase transitions.
        selected = [e for e in events if e["event"] != "command" and e["event"] != "preparation"]
        (destination / "policy-events.jsonl.gz").write_bytes(gzip.compress(
            ("\n".join(json.dumps(e, allow_nan=False) for e in selected) + "\n").encode(), mtime=0))
    atomic_json(output / "summary.json", summary)
    atomic_json(output / "ARCHIVE.json", {
        "source_run": str(run.resolve()), "tool_sha256": sha256(Path(__file__)),
        "artifacts": {str(p.relative_to(output)): sha256(p) for p in output.rglob("*") if p.is_file()},
        "note": "Original COMPLETE hashes refer to original raw runs; this archive retains selected events.",
        "hardware_qualified": False,
    })
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--engines", choices=["mujoco", "isaaclab"], nargs="+",
                        default=["mujoco", "isaaclab"])
    args = parser.parse_args()
    print(json.dumps(summarize(args.run, args.output, args.engines), indent=2))
