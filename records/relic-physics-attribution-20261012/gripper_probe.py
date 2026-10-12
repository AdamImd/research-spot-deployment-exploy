"""Registered 20 ms fixed-target air pulse, optional specific finger/jaw pairs."""

import argparse
import json
import sys
import hashlib
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "simulation"), str(ROOT / "simulation/tools")]
from spot_relic_sim.contract import Contract  # noqa: E402
from spot_relic_sim.scenarios import Scenario  # noqa: E402
from debug_shadow import initialize_captured, SDK_NAMES  # noqa: E402
from spot_deploy.records import atomic_json  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--backend", choices=["mujoco", "isaaclab"], required=True)
parser.add_argument("--pairs", action="store_true")
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
backend = app = sensor = None
scenario = Scenario(name="gripper-contact-probe", duration=0.02, smoke=True, seed=101)
try:
    if args.backend == "mujoco":
        import mujoco as mj
        import xml.etree.ElementTree as ET
        from spot_relic_sim.mujoco_backend import MujocoBackend

        backend = MujocoBackend(
            Contract.load(ROOT / "simulation/configs/articulation.json"),
            args.output / "model",
            scenario,
        )
        if args.pairs:
            tree = ET.parse(args.output / "model/spot.xml")
            named = {}
            for name in ["arm_link_fngr", "arm_link_jaw"]:
                body = tree.find(f".//body[@name='{name}']")
                names = []
                for i, geom in enumerate(body.findall("geom")):
                    if geom.get("contype", "1") == "0":
                        continue
                    geom.set("name", f"{name}_pair_{i}")
                    names.append(geom.get("name"))
                named[name] = names
            contact = ET.SubElement(tree.getroot(), "contact")
            for a in named["arm_link_fngr"]:
                for b in named["arm_link_jaw"]:
                    ET.SubElement(
                        contact, "pair", geom1=a, geom2=b, friction=".8 .8 .005 .0001 .0001"
                    )
            destination = args.output / "model/specific-pairs.xml"
            tree.write(destination)
            backend.model = mj.MjModel.from_xml_path(str(destination))
            backend.data = mj.MjData(backend.model)
            backend.reset(scenario)
    else:
        from isaaclab.app import AppLauncher

        app = AppLauncher(
            headless=True,
            device="cpu",
            kit_args="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0",
        ).app
        from spot_relic_sim.isaac_backend import IsaacBackend
        from isaaclab.sensors import ContactSensor, ContactSensorCfg

        backend = IsaacBackend(args.output / "model", scenario)
        sensor = ContactSensor(
            ContactSensorCfg(
                prim_path="/World/Spot/arm_link_fngr",
                filter_prim_paths_expr=["/World/Spot/arm_link_jaw"],
                update_period=0.0,
                history_length=0,
            )
        )
        backend.sim.reset()
        backend.reset(scenario)
    initial = json.loads(
        (ROOT / "local/relic-walk-inputs-20261009-001/capture/initial-state.json").read_text()
    )
    height = json.loads(
        (ROOT / "local/relic-walk-inputs-20261009-001/capture/initial-height.json").read_text()
    )["height_m"]
    initialize_captured(backend, initial, height + 2, False)
    first = json.loads(
        (ROOT / "runs/relic-unguarded-walk-20261012-001/mujoco/rollout.jsonl")
        .read_text()
        .splitlines()[0]
    )
    target = np.array(first["positions"])
    inverse = [SDK_NAMES.index(n) for n in backend.contract.names]
    target = target[inverse]
    result = []
    for tick in range(5):
        s = backend.state()
        row = dict(
            time_s=tick * 0.005,
            q=s["q"].tolist(),
            dq=s["dq"].tolist(),
            root_height_m=float(s["position"][2]),
        )
        if sensor and tick:
            row["finger_jaw_normal_force_w_N"] = sensor.data.force_matrix_w.cpu().numpy().tolist()
            row["finger_net_normal_force_w_N"] = sensor.data.net_forces_w.cpu().numpy().tolist()
        if hasattr(backend, "data"):
            row["contact_pairs"] = [
                dict(geom1=int(c.geom[0]), geom2=int(c.geom[1]), distance_m=float(c.dist))
                for c in backend.data.contact
            ]
        result.append(row)
        if tick < 4:
            backend.step(target)
            if sensor:
                sensor.update(0.005, force_recompute=True)
    assert np.isfinite([x for r in result for x in r["q"] + r["dq"]]).all()
    atomic_json(
        args.output / "result.json",
        dict(
            backend=args.backend,
            specific_pairs=args.pairs,
            hardware_access=False,
            policy_rollout=False,
            held_first_target_sdk=first["positions"],
            joint_names=backend.contract.names,
            rows=result,
        ),
    )
    atomic_json(
        args.output / "COMPLETE.json",
        dict(
            status="completed",
            hardware_access=False,
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            artifacts={
                "result.json": hashlib.sha256(
                    (args.output / "result.json").read_bytes()
                ).hexdigest()
            },
        ),
    )
except BaseException:
    import traceback

    (args.output / "ERROR.txt").write_text(traceback.format_exc())
    raise
finally:
    if backend:
        backend.close()
    if app:
        app.close(skip_cleanup=True)
