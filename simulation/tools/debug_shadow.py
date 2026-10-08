"""Recorded-state attribution and bounded closed-loop ReLIC shadow diagnostics.

No SDK imports or hardware access. See docs/SHADOW_DEBUG.md for the protocol.
"""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spot_relic_sim.contract import (  # noqa: E402
    ARM, ASSET, ROOT, Command, Contract, Policy, defaults, sha256,
)
from spot_relic_sim.record import provenance, write_json  # noqa: E402
from spot_relic_sim.scenarios import Scenario  # noqa: E402

SDK_NAMES = tuple(f"{leg}_{kind}" for leg in ("fl", "fr", "hl", "hr")
                  for kind in ("hx", "hy", "kn")) + ARM


@dataclass
class StandingDiagnostic(Scenario):
    held_arm: tuple = ()
    requested_height: float = .55

    def command(self, t):
        return Command(arm=np.asarray(self.held_arm, dtype=np.float32),
                       torso=np.array([0, 0, self.requested_height], dtype=np.float32))


def inputs(directory):
    paths = {name: directory / name for name in
             ("window.jsonl", "initial-state.json", "initial-height.json", "PROTOCOL.md")}
    rows = [json.loads(line) for line in paths["window.jsonl"].read_text().splitlines()]
    initial = json.loads(paths["initial-state.json"].read_text())
    height = json.loads(paths["initial-height.json"].read_text())["height_m"]
    assert len(rows) == 500 and all(r["event"] == "shadow_sample" for r in rows)
    assert 0 < height < 1.5
    return rows, initial, height, {str(p.resolve()): sha256(p) for p in paths.values()}


def replay(directory, output):
    rows, _, height, hashes = inputs(directory)
    contract, policy = Contract.load(ROOT / "configs/articulation.json"), Policy()
    ids = [contract.names.index(n) for n in SDK_NAMES]
    observations = np.asarray([r["observations"] for r in rows], dtype=np.float32)
    measured = np.asarray([r["state"]["positions"] for r in rows])
    times = np.asarray([r["state"]["received_monotonic_s"] for r in rows])
    times -= times[0]
    last = times >= times[-1] - 1
    assert np.all(np.diff(times) > 0)
    # Reproduce original logged inference before changing any input.
    logged = np.asarray([r["raw_actions"] for r in rows])
    reproduced = np.asarray([policy(o) for o in observations])
    replay_error = float(np.abs(logged - reproduced).max())
    assert replay_error < 2e-5
    results, curves = {}, {}
    for name, h, nominal_arm in (("original_height", .55, False),
                                  ("initial_height", height, False),
                                  ("initial_height_nominal_arm", height, True)):
        previous = np.zeros(12, dtype=np.float32)
        targets, actions = [], []
        for recorded in observations:
            obs = recorded.copy()
            obs[33], obs[72:84] = h, previous
            if nominal_arm:
                obs[12:19] = defaults(ARM)
                obs[34 + contract.arm_ids] = 0
                obs[53 + contract.arm_ids] = 0
            command = Command(arm=obs[12:19], torso=obs[31:34])
            previous = policy(obs)
            targets.append(contract.targets(previous, command)[ids])
            actions.append(previous)
        target = np.asarray(targets)
        gap = np.rad2deg(target[:, :12] - measured[:, :12])
        symmetry = np.column_stack((target[:, 0] + target[:, 3], target[:, 1] - target[:, 4],
                                    target[:, 2] - target[:, 5], target[:, 6] + target[:, 9],
                                    target[:, 7] - target[:, 10], target[:, 8] - target[:, 11]))
        results[name] = dict(height=h, synthetic_nominal_arm=nominal_arm,
            first_max_gap_deg=float(np.abs(gap[0]).max()),
            last_second_max_gap_deg=float(np.abs(gap[last]).max()),
            last_second_rms_gap_deg=float(np.sqrt(np.mean(gap[last] ** 2))),
            last_second_max_mirror_asymmetry_deg=float(np.rad2deg(np.abs(symmetry[last])).max()),
            mean_knee_targets_deg=np.rad2deg(target[last][:, [2, 5, 8, 11]].mean(axis=0)).tolist(),
            last_second_target_std_max_deg=float(np.rad2deg(target[last].std(axis=0)).max()))
        curves[name] = target
        np.savez(output / f"{name}.npz", targets=target, actions=actions, time=times)
    result = dict(cases=results, input_sha256=hashes, samples=len(rows),
                  original_logged_action_replay_max_error=replay_error,
                  height_only_last_second_max_target_change_deg=float(np.rad2deg(np.abs(
                      curves["initial_height"][last, :12] - curves["original_height"][last, :12])).max()),
                  initial_previous_actions="zeros, then preceding raw prediction in every case",
                  hardware_access=False, interpretation="Unexecuted state sequence; arm substitution is synthetic")
    live_targets = np.asarray([r["targets"] for r in rows])
    result["recorded_history_comparison"] = {
        "physical_observations_and_commands_identical": True,
        "final_second_max_target_difference_deg": float(np.rad2deg(np.abs(
            live_targets[last, :12] - curves["original_height"][last, :12])).max()),
        "recorded_mean_knee_targets_deg": np.rad2deg(
            live_targets[last][:, [2, 5, 8, 11]].mean(axis=0)).tolist(),
        "recorded_final_second_max_gap_deg": float(np.rad2deg(np.abs(
            live_targets[last, :12] - measured[last, :12])).max()),
        "scope": "Existing run history versus fresh zero-initialized replay; live history unchanged",
    }
    write_json(output / "result.json", result)
    write_json(output / "provenance.json", provenance("offline-replay", contract))
    return result


def initialize_captured(backend, initial, height, nominal_arm):
    q = np.asarray(initial["positions"], dtype=np.float32)
    q = q[[SDK_NAMES.index(n) for n in backend.contract.names]]
    if nominal_arm:
        q[backend.contract.arm_ids] = defaults(ARM)
    quat = np.asarray(initial["odom_quaternion_wxyz"], dtype=np.float32)
    if hasattr(backend, "data"):
        import mujoco as mj
        backend.data.qpos[:3] = [0, 0, height]
        backend.data.qpos[3:7] = quat
        backend.data.qpos[backend.qids] = q
        backend.data.qvel[:] = 0
        mj.mj_forward(backend.model, backend.data)
    else:
        import torch
        robot, device = backend.robot, backend.sim.device
        robot.write_root_pose_to_sim(torch.tensor([[0, 0, height, *quat]], device=device))
        robot.write_root_velocity_to_sim(torch.zeros((1, 6), device=device))
        robot.write_joint_state_to_sim(torch.tensor(q[None], device=device),
                                       torch.zeros((1, 19), device=device))
        backend.sim.forward()
        robot.update(.005)
    np.testing.assert_allclose(backend.state()["q"], q, atol=2e-5, rtol=0)
    np.testing.assert_allclose(backend.state()["position"][2], height, atol=2e-5, rtol=0)


def torchscript_parity(rows, output):
    import torch
    torch.set_num_threads(2)
    script_path = ASSET / "pretrained/policy.pt"
    model = torch.jit.load(str(script_path), map_location="cpu").eval()
    obs = np.asarray([r["observations"] for r in rows], dtype=np.float32)
    with torch.inference_mode():
        actions = model(torch.from_numpy(obs)).numpy()
    error = float(np.abs(actions - np.asarray([r["raw_actions"] for r in rows])).max())
    report = dict(torchscript_sha256=sha256(script_path), max_abs_raw_error=error,
                  passed=error < 2e-5, samples=len(rows))
    write_json(output / "torchscript-parity.json", report)
    assert report["passed"], report


def collision_audit(backend, output):
    from pxr import Usd, UsdGeom
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    report, meshes = [], []
    transforms = UsdGeom.XformCache()
    # The current URDF importer instances collider scopes even when the outer
    # articulation is not instanceable. Plain PrimRange misses these shapes.
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Spot"), Usd.TraverseInstanceProxies()):
        properties = {a.GetName(): str(a.Get()) for a in prim.GetAttributes()
                      if "collision" in a.GetName().lower() or "approximation" in a.GetName().lower()}
        relations = {r.GetName(): [str(t) for t in r.GetTargets()] for r in prim.GetRelationships()
                     if "filter" in r.GetName().lower()}
        if properties or relations or "/collisions/" in str(prim.GetPath()):
            report.append(dict(path=str(prim.GetPath()), type=prim.GetTypeName(),
                               instance_proxy=prim.IsInstanceProxy(),
                               schemas=list(prim.GetAppliedSchemas()),
                               geometric_primitive=prim.IsA(UsdGeom.Gprim),
                               properties=properties, relations=relations))
        if "/collisions/" in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            link = str(prim.GetPath()).split("/collisions/")[0]
            relative, _ = transforms.ComputeRelativeTransform(prim, stage.GetPrimAtPath(link))
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            meshes.append(dict(path=str(prim.GetPath()), link=link.rsplit("/", 1)[-1],
                vertices_link_frame=[list(relative.Transform(p)) for p in points]))
    write_json(output / "runtime-collision-properties.json", report)
    write_json(output / "collision-mesh-vertices.json", meshes)
    write_json(output / "compiled-collision-count.json", {
        "articulations": backend.robot.root_physx_view.count,
        "compiled_shapes_per_articulation": backend.robot.root_physx_view.max_shapes,
        "enabled_geometry_prims": [r["path"] for r in report
            if r["geometric_primitive"] and r["properties"].get("physics:collisionEnabled") == "True"],
        "enabled_non_geometry_prims": [r["path"] for r in report
            if not r["geometric_primitive"] and r["properties"].get("physics:collisionEnabled") == "True"],
    })
    assert any(r["properties"].get("physics:collisionEnabled") == "True" for r in report), \
        "Audit did not traverse enabled collision shapes"


def exclude_body_upper_legs(backend, output):
    """Diagnostic only: isolate the four observed 2 cm internal overlaps."""
    import xml.etree.ElementTree as ET
    import mujoco as mj
    tree = ET.parse(output / "model/spot.xml")
    contact = ET.SubElement(tree.getroot(), "contact")
    for leg in ("fl", "fr", "hl", "hr"):
        ET.SubElement(contact, "exclude", body1="body", body2=f"{leg}_uleg")
    path = output / "model/spot-excluded.xml"
    tree.write(path)
    backend.model = mj.MjModel.from_xml_path(str(path))
    backend.data = mj.MjData(backend.model)
    assert backend.model.nexclude == 4


def simulate(directory, output, engine, exclude=False, audit_only=False):
    from spot_relic_sim.runner import run
    rows, initial, height, hashes = inputs(directory)
    write_json(output / "inputs.json", hashes)
    held = tuple(initial["positions"][12:])
    cells = (("stowed_original_height", .55, False, False),
             ("stowed_initial_height", height, False, False),
             ("nominal_arm_initial_height", height, True, False),
             ("released_default_reset", .55, True, True))
    backend = None
    results = {}
    try:
        for name, requested, nominal_arm, default_reset in cells:
            s = StandingDiagnostic(name=name, duration=10, smoke=True, seed=101,
                requested_height=requested,
                held_arm=tuple(float(v) for v in defaults(ARM)) if nominal_arm else held)
            if backend is None:
                if engine == "mujoco":
                    from spot_relic_sim.mujoco_backend import MujocoBackend
                    contract = Contract.load(ROOT / "configs/articulation.json")
                    backend = MujocoBackend(contract, output / "model", s)
                    if exclude:
                        exclude_body_upper_legs(backend, output)
                        backend.reset(s)
                else:
                    from spot_relic_sim.isaac_backend import IsaacBackend
                    backend = IsaacBackend(output / "model", s, device="cpu", headless=True)
                    torchscript_parity(rows, output)
                    collision_audit(backend, output)
            else:
                backend.reset(s)
            if not default_reset:
                initialize_captured(backend, initial, height, nominal_arm)
            if audit_only:
                write_json(output / "result.json", {"collision_audit_only": True})
                return {"collision_audit_only": True}
            results[name] = run(backend, output / name, engine, s)
            print(json.dumps({name: results[name]}), flush=True)
        write_json(output / "result.json", results)
        return results
    finally:
        if backend:
            backend.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", required=True, choices=["replay", "mujoco", "isaaclab"])
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--exclude-body-upper-legs", action="store_true")
    parser.add_argument("--collision-audit-only", action="store_true")
    args = parser.parse_args()
    if args.exclude_body_upper_legs and args.backend != "mujoco":
        parser.error("body/upper-leg exclusions are a MuJoCo diagnostic only")
    if args.collision_audit_only and args.backend != "isaaclab":
        parser.error("runtime collision audit requires Isaac")
    args.output.mkdir(parents=True, exist_ok=False)
    app = None
    try:
        if args.backend == "isaaclab":
            from isaaclab.app import AppLauncher
            app = AppLauncher(headless=True, device="cpu",
                kit_args="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0").app
        result = (replay(args.input, args.output) if args.backend == "replay" else
                  simulate(args.input, args.output, args.backend,
                           args.exclude_body_upper_legs, args.collision_audit_only))
        write_json(args.output / "COMPLETE.json", {"status": "completed", "hardware_access": False})
        print(json.dumps(result), flush=True)
    except BaseException:
        import traceback
        (args.output / "ERROR.txt").write_text(traceback.format_exc())
        raise
    finally:
        if app:
            app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
