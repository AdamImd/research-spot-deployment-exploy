import copy
import http.client
import struct
import threading
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spot_deploy.contracts import ContractError
from spot_deploy.kinematics import from_urdf, schematic
from spot_deploy.viewer import DemoFeed, make_server
from spot_deploy.visual_assets import VisualAssets, load_visual_model, merge_visual_tree


def robot_file(tmp_path, visual=""):
    model = schematic()
    root = ET.Element("robot", name="fixture")
    for link in model["links"]:
        element = ET.SubElement(root, "link", name=link["name"])
        if link["name"] == "base" and visual:
            element.append(ET.fromstring(visual))
    for joint in model["joints"]:
        element = ET.SubElement(root, "joint", name=joint["name"], type=joint["type"])
        ET.SubElement(element, "parent", link=joint["parent"])
        ET.SubElement(element, "child", link=joint["child"])
        ET.SubElement(element, "origin", xyz=" ".join(map(str, joint["xyz"])))
        ET.SubElement(element, "axis", xyz=" ".join(map(str, joint["axis"])))
    path = tmp_path / "robot.urdf"
    path.write_text(ET.tostring(root, encoding="unicode"))
    return path


def test_visual_origin_scale_color_and_all_links(tmp_path):
    path = robot_file(tmp_path, '<visual><origin xyz="1 2 3" rpy=".1 .2 .3"/>'
                      '<geometry><mesh filename="body.obj" scale="1 2 3"/></geometry>'
                      '<material><color rgba="1 .8 0 1"/></material></visual>')
    model = from_urdf(path)
    assert len(model["links"]) == 20
    visual = model["links"][0]["visuals"][0]
    assert visual["xyz"] == [1, 2, 3] and visual["rpy"] == [.1, .2, .3]
    assert visual["geometry"]["scale"] == [1, 2, 3]
    assert visual["rgba"] == [1, .8, 0, 1]


def test_visual_template_keeps_measured_joints_and_adds_fixed_leaf(tmp_path):
    model = from_urdf(robot_file(tmp_path))
    visual = copy.deepcopy(model)
    visual["joints"].append(dict(name="foot_fixed", child="foot", parent="fl_kn",
                                 type="fixed", xyz=[0, 0, -.3], rpy=[0, 0, 0], axis=[1, 0, 0]))
    visual["links"].append(dict(name="foot", visuals=[]))
    visual["links"][0]["visuals"] = [dict(geometry=dict(kind="box", size=[1, 1, 1]))]
    before = copy.deepcopy(model["joints"])
    merge_visual_tree(model, visual)
    assert model["joints"][:19] == before
    assert len(model["links"]) == 21 and model["joints"][-1]["child"] == "visual:foot"
    assert model["links"][0]["visuals"][0]["geometry"]["kind"] == "box"


@pytest.mark.parametrize("field,value", [("xyz", [1, 2, 3]), ("axis", [-1, 0, 0]),
                                        ("type", "prismatic"), ("parent", "fr_hx")])
def test_visual_template_rejects_changed_kinematics(tmp_path, field, value):
    model = from_urdf(robot_file(tmp_path))
    visual = copy.deepcopy(model)
    visual["joints"][0][field] = value
    with pytest.raises((ContractError, KeyError)):
        merge_visual_tree(model, visual)


def test_obj_normals_uv_negative_indices_groups_and_local_materials(tmp_path):
    (tmp_path / "mesh.mtl").write_text("newmtl yellow\nKd 1 .8 0\n")
    (tmp_path / "mesh.obj").write_text(
        "mtllib mesh.mtl\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\nusemtl yellow\n"
        "f -4/1/1 -3/2/1 -2/3/1 -1/4/1\n")
    assets = VisualAssets()
    model = assets.mesh("mesh.obj", tmp_path, tmp_path)
    assert model["vertex_count"] == 6
    data = np.frombuffer(assets.data[model["url"]][0], dtype="<f4").reshape(-1, 8)
    np.testing.assert_array_equal(data[0], [0, 0, 0, 0, 0, 1, 0, 0])
    np.testing.assert_array_equal(data[-1], [0, 1, 0, 0, 0, 1, 0, 1])
    assert model["materials"]["yellow"]["rgba"] == [1, .8, 0, 1]
    assert model["groups"] == [dict(start=0, count=6, material="yellow")]


@pytest.mark.parametrize("filename", ["../outside.obj", "https://example.com/mesh.obj",
                                      "file:///etc/passwd", "escape.obj"])
def test_mesh_assets_cannot_escape_root(tmp_path, filename):
    root = tmp_path / "allowed"
    root.mkdir()
    (tmp_path / "outside.obj").write_text("v 0 0 0")
    (root / "escape.obj").symlink_to(tmp_path / "outside.obj")
    with pytest.raises(ContractError):
        VisualAssets().mesh(filename, root, root)


def test_obj_cannot_smuggle_remote_or_outside_material(tmp_path):
    (tmp_path / "mesh.obj").write_text("mtllib https://evil.example/secret\n")
    with pytest.raises(ContractError):
        VisualAssets().mesh("mesh.obj", tmp_path, tmp_path)


def test_derived_asset_budget_and_deduplication(monkeypatch):
    monkeypatch.setattr("spot_deploy.visual_assets.MAX_PUBLISHED_BYTES", 4)
    assets = VisualAssets()
    url = assets.publish(b"1234", "application/octet-stream", "bin")
    assert assets.publish(b"1234", "application/octet-stream", "bin") == url
    with pytest.raises(ContractError, match="total size limit"):
        assets.publish(b"5", "application/octet-stream", "bin")
    assert len(assets.data) == 1


@pytest.mark.parametrize("face", ["f 0 1 2", "f 1 2 9", "f 1//2 2 3"])
def test_malformed_obj_is_rejected(tmp_path, face):
    (tmp_path / "mesh.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\n" + face)
    with pytest.raises(ContractError):
        VisualAssets().mesh("mesh.obj", tmp_path, tmp_path)


def test_binary_stl_geometry(tmp_path):
    data = b"STL".ljust(80, b"\0") + struct.pack("<I12fH", 1, 0, 0, 1,
                                               0, 0, 0, 1, 0, 0, 0, 1, 0, 0)
    (tmp_path / "mesh.stl").write_bytes(data)
    assets = VisualAssets()
    mesh = assets.mesh("mesh.stl", tmp_path, tmp_path)
    assert mesh["vertex_count"] == 3
    array = np.frombuffer(assets.data[mesh["url"]][0], dtype="<f4").reshape(-1, 8)
    np.testing.assert_array_equal(array[:, 3:6], [[0, 0, 1]] * 3)


def test_missing_visual_is_reported_without_collision_substitution(tmp_path):
    path = robot_file(tmp_path, '<visual><geometry><mesh filename="missing.obj"/></geometry></visual>')
    model, assets = load_visual_model(path)
    assert model["missing_visuals"] == ["base"] and not assets.data
    assert model["visual_count"] == 0
    assert model["links"][0]["visuals"][0]["geometry"]["error"]


def test_http_assets_are_opaque_validated_and_read_only(tmp_path):
    path = robot_file(tmp_path, '<visual><geometry><mesh filename="mesh.obj"/></geometry></visual>')
    (tmp_path / "mesh.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    feed = DemoFeed()
    feed.model, feed.assets = load_visual_model(path)
    url = feed.model["links"][0]["visuals"][0]["geometry"]["url"]
    server = make_server(feed, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
    try:
        client.request("GET", url)
        response = client.getresponse()
        assert response.status == 200
        assert response.headers["Content-Type"] == "application/octet-stream"
        assert len(response.read()) == 3 * 32
        for bad in ("/assets/mesh.obj", "/assets/../robot.urdf", "/assets/nonexistent.bin"):
            client.request("GET", bad)
            response = client.getresponse()
            assert response.status == 404
            response.read()
        client.request("GET", url, headers={"Origin": "https://evil.example"})
        response = client.getresponse()
        assert response.status == 403
        response.read()
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        thread.join(2)
