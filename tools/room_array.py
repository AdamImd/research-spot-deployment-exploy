"""Import the room twin's optical models, floor and archived Spot placement."""

import argparse
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / "local/room-cameras/array.json"
NAMES = ("flir-12mp", "flir-5mp", "kinect-1", "kinect-2", "d415-1083", "d415-8342", "l515")
PATHS = (
    "data/flir-12mp-repair-20261103/calibration-20261103/fleet-seven.json",
    "data/room-twin-20261106/fresh-room-4/scene.json",
    "data/room-twin-20261106/semantic-spot-proxy-1/proxy.json",
    "config/fleet-tracking-follow.json",
)


def import_array(base):
    inputs = {p: (base / p).read_bytes() for p in PATHS}
    hashes = {p: hashlib.sha256(data).hexdigest() for p, data in inputs.items()}
    cal, scene, proxy, cfg = [json.loads(inputs[p]) for p in PATHS]
    if scene["calibration_sha256"] != hashes[PATHS[0]] or cal["reference"] != "flir-12mp":
        raise ValueError("room and camera reference/calibration differ")
    cameras = {}
    for n in NAMES:
        c = cal["cameras"][n]
        cameras[n] = dict(
            **{k: c["intrinsics"][k] for k in ("model", "K", "distortion", "image_size")},
            T_reference_camera=c["T_reference_camera"],
            rotation=cfg["cameras"][n]["display_rotation_deg"],
        )
    return dict(
        reference=cal["reference"],
        verification=cal["status"],
        source_sha256=hashes,
        T_room_reference=scene["T_room_reference"],
        archived_body_pose=proxy["pose"],
        cameras=cameras,
    )


def load_array():
    path = Path(os.getenv("RELIC_ROOM_ARRAY", DEFAULT))
    if path.exists():
        return json.loads(path.read_text())
    raise FileNotFoundError(
        f"Missing room array {path}. Import the calibration with tools/room_array.py first."
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--fleet-repo", type=Path, required=True)
    p.add_argument("--output", type=Path, default=DEFAULT)
    a = p.parse_args()
    data = import_array(a.fleet_repo)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("x") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    print(a.output)
