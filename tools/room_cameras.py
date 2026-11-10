"""Render the calibrated room array from a copy of the simulated articulation."""

import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

SIDE = 448


def optical_map(camera, width=384):
    """Map native distorted pixels to the centered square perspective render."""
    w, h = camera["image_size"]
    height = round(width * h / w)
    y, x = np.indices((height, width), dtype=np.float64)
    pixels = np.stack(((x + 0.5) * w / width - 0.5, (y + 0.5) * h / height - 0.5), axis=-1)
    uv = pixels.reshape(-1, 1, 2)
    k, d = np.array(camera["K"]), np.array(camera["distortion"])
    if camera["model"] == "fisheye":
        rays = cv2.fisheye.undistortPoints(uv, k, d)
    else:
        rays = cv2.undistortPointsIter(
            uv, k, d, None, None, (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-10)
        )
    rays = rays.reshape(height, width, 2)
    if not np.isfinite(rays).all() or np.max(abs(rays)) > 10:
        raise ValueError("invalid calibrated field of view")
    span = np.max(abs(rays)) * 1.01
    return (rays * (SIDE / (2 * span)) + (SIDE - 1) / 2).astype(np.float32), math.degrees(
        2 * math.atan(span)
    )


def planar_frame(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)


def add_cameras(xml, cfg, start):
    model = ET.parse(xml)
    world = model.find("worldbody")
    prior = np.array(cfg["archived_body_pose"])
    room_start = planar_frame(*prior[:2, 3], math.atan2(prior[1, 0], prior[0, 0]))
    qw, qx, qy, qz = start[3:7]
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    sim_ref = planar_frame(*start[:2], yaw) @ np.linalg.inv(room_start) @ cfg["T_room_reference"]
    maps = {}
    for name, camera in cfg["cameras"].items():
        pose = sim_ref @ camera["T_reference_camera"]
        # OpenCV +x right, +y down, +z forward. MJCF camera looks along -z.
        axes = np.r_[pose[:3, 0], -pose[:3, 1]]
        maps[name], fov = optical_map(camera)
        ET.SubElement(
            world,
            "camera",
            name=name,
            pos=" ".join(map(str, pose[:3, 3])),
            xyaxes=" ".join(map(str, axes)),
            fovy=str(fov),
        )
    path = xml.with_name("room-display.xml")
    model.write(path)
    return path, maps


def render(run):
    cfg = json.loads((run / "room-array.json").read_text())
    folder = run / "views"
    folder.mkdir()
    state_file = run / "view-state.json"
    deadline = time.monotonic() + 40
    while not state_file.exists():
        if time.monotonic() > deadline or (run / "view-done").exists():
            raise ValueError("no simulated pose available")
        time.sleep(0.03)
    first = json.loads(state_file.read_text())
    xml, maps = add_cameras(run / "model/spot.xml", cfg, first["start_qpos"])
    model = mujoco.MjModel.from_xml_path(str(xml))
    model.vis.global_.offheight = model.vis.global_.offwidth = SIDE
    data = mujoco.MjData(model)
    options = mujoco.MjvOption()
    # Converted URDF uses group 1 for visuals and group 0 for contact geometry.
    # Hide duplicate contact meshes, while keeping the world floor visible.
    for i in range(model.ngeom):
        if model.geom_bodyid[i] and model.geom_contype[i]:
            model.geom_group[i] = 5
    options.geomgroup[5] = 0
    last = None
    with mujoco.Renderer(model, height=SIDE, width=SIDE) as renderer:
        while True:
            state = json.loads(state_file.read_text())
            if state["tick"] != last:
                data.qpos[:] = state["qpos"]
                mujoco.mj_forward(model, data)
                for name, coords in maps.items():
                    renderer.update_scene(data, camera=name, scene_option=options)
                    rgb = cv2.remap(renderer.render(), coords, None, cv2.INTER_LINEAR)
                    if cfg["cameras"][name]["rotation"] == 180:
                        rgb = cv2.rotate(rgb, cv2.ROTATE_180)
                    ok, jpeg = cv2.imencode(".jpg", rgb[:, :, ::-1])
                    if not ok:
                        raise OSError("JPEG encoding failed")
                    tmp = folder / (name + ".tmp")
                    tmp.write_bytes(jpeg.tobytes())
                    tmp.replace(folder / (name + ".jpg"))
                last = state["tick"]
                tmp = folder / "frame.tmp"
                tmp.write_text(
                    json.dumps(
                        dict(tick=last, seconds=state["seconds"], sampled_at=state["sampled_at"])
                    )
                )
                tmp.replace(folder / "frame.json")
            if (run / "view-done").is_file() and last == json.loads(state_file.read_text())["tick"]:
                break
            time.sleep(0.05)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    render(parser.parse_args().run.resolve())
