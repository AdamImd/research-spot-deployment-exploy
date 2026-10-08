"""Bounded local visual assets. Only derived geometry and validated PNGs reach HTTP."""

import copy
import hashlib
import math
import shlex
import struct
from urllib.parse import urlsplit

import numpy as np

from .contracts import ContractError
from .kinematics import from_urdf

MAX_FILE = 16_000_000
MAX_TRIANGLES = 250_000
MAX_PUBLISHED_BYTES = 128_000_000
RELIC_ALIASES = {f"arm_{s}": f"arm0_{s}" for s in
                 ("sh0", "sh1", "el0", "el1", "wr0", "wr1", "f1x")}


class VisualAssets:
    def __init__(self):
        self.data, self.files, self.meshes = {}, {}, {}
        self.total_bytes = 0
        self.published_bytes = 0

    def read(self, filename, base, root):
        url = urlsplit(filename)
        if url.scheme == "package":
            path = root / url.netloc / url.path.lstrip("/")
        elif not url.scheme and not url.netloc and not url.query and not url.fragment:
            path = base / filename
        else:
            raise ContractError("visual asset must be a local file")
        path = path.resolve()
        if not path.is_relative_to(root.resolve()):
            raise ContractError("visual asset escapes its allowed mesh root")
        if path.stat().st_size > MAX_FILE:
            raise ContractError("visual asset exceeds size limit")
        data = path.read_bytes()
        if len(data) > MAX_FILE:
            raise ContractError("visual asset exceeds size limit")
        key = str(path)
        if key not in self.files:
            self.total_bytes += len(data)
            if self.total_bytes > 128_000_000:
                raise ContractError("visual assets exceed total size limit")
            self.files[key] = dict(name=str(path.relative_to(root.resolve())),
                                   sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))
        return path, data

    def publish(self, data, mime, extension):
        digest = hashlib.sha256(data).hexdigest()
        url = f"/assets/{digest}.{extension}"
        if url not in self.data:
            if self.published_bytes + len(data) > MAX_PUBLISHED_BYTES:
                raise ContractError("derived visual assets exceed total size limit")
            self.published_bytes += len(data)
        self.data[url] = (data, mime)
        return url

    def texture(self, filename, base, root):
        path, data = self.read(filename, base, root)
        if (path.suffix.lower() != ".png" or data[:8] != b"\x89PNG\r\n\x1a\n"
                or len(data) < 24 or data[12:16] != b"IHDR"):
            raise ContractError("visual textures must be PNG images")
        width, height = struct.unpack(">II", data[16:24])
        if not 0 < width <= 4096 or not 0 < height <= 4096:
            raise ContractError("visual texture dimensions exceed limit")
        return self.publish(data, "image/png", "png")

    def materials(self, filename, base, root):
        path, data = self.read(filename, base, root)
        if path.suffix.lower() != ".mtl":
            raise ContractError("expected a local MTL material")
        materials, current = {}, None
        for line in data.decode("utf-8").splitlines():
            fields = shlex.split(line, comments=True)
            if not fields:
                continue
            key, values = fields[0], fields[1:]
            if key == "newmtl":
                if len(materials) >= 128 or len(values) != 1:
                    raise ContractError("invalid material library")
                current = materials[values[0]] = dict(rgba=[1, 1, 1, 1])
            elif current is not None:
                if key == "Kd":
                    rgb = numbers(values, 3)
                    if any(x < 0 or x > 1 for x in rgb):
                        raise ContractError("invalid diffuse color")
                    current["rgba"][:3] = rgb
                elif key in ("d", "Tr"):
                    opacity = numbers(values, 1)[0]
                    if not 0 <= opacity <= 1:
                        raise ContractError("invalid material opacity")
                    current["rgba"][3] = 1 - opacity if key == "Tr" else opacity
                elif key == "map_Kd":
                    scale = [1, 1, 1]
                    if values[:1] == ["-s"]:
                        scale, values = numbers(values[1:4], 3), values[4:]
                    if len(values) != 1:
                        raise ContractError("unsupported diffuse texture options")
                    current.update(texture=self.texture(values[0], path.parent, root),
                                   texture_scale=scale[:2])
        return materials

    def mesh(self, filename, base, root):
        path, data = self.read(filename, base, root)
        cache_key = str(path)
        if cache_key in self.meshes:
            return copy.deepcopy(self.meshes[cache_key])
        if path.suffix.lower() == ".obj":
            vertices, normals, uv, groups, materials = self.obj(data, path.parent, root)
        elif path.suffix.lower() == ".stl":
            vertices, normals = stl(data)
            uv, groups, materials = np.zeros((len(vertices), 2)), [], {}
        else:
            raise ContractError("visual meshes support OBJ and STL")
        values = np.column_stack((vertices, normals, uv)).astype("<f4")
        if not np.isfinite(values).all():
            raise ContractError("non-finite visual mesh")
        result = dict(kind="mesh", url=self.publish(values.tobytes(),
                      "application/octet-stream", "bin"), vertex_count=len(values),
                      groups=groups, materials=materials, source_sha256=hashlib.sha256(data).hexdigest())
        self.meshes[cache_key] = result
        return copy.deepcopy(result)

    def obj(self, data, base, root):
        vertices, normals, uv, corners, groups, materials = [], [], [], [], [], {}
        material = None
        for line in data.decode("utf-8").splitlines():
            fields = line.split("#", 1)[0].split()
            if not fields:
                continue
            key, values = fields[0], fields[1:]
            if key in ("v", "vn", "vt"):
                target = {"v": vertices, "vn": normals, "vt": uv}[key]
                target.append(numbers(values[:2 if key == "vt" else 3],
                                      2 if key == "vt" else 3))
                if len(target) > MAX_TRIANGLES * 3:
                    raise ContractError("mesh vertex limit exceeded")
            elif key == "mtllib":
                for name in values:
                    materials.update(self.materials(name, base, root))
            elif key == "usemtl":
                material = " ".join(values)
            elif key == "f":
                if not 3 <= len(values) <= 64:
                    raise ContractError("invalid OBJ polygon")
                face = []
                for value in values:
                    indices = value.split("/")
                    if len(indices) > 3:
                        raise ContractError("invalid OBJ face")
                    indices += [""] * (3 - len(indices))
                    face.append([obj_index(index, len(target)) if index else -1
                                 for index, target in zip(indices, (vertices, uv, normals))])
                    if face[-1][0] < 0:
                        raise ContractError("missing OBJ vertex")
                if not groups or groups[-1]["material"] != material:
                    if len(groups) >= 4096:
                        raise ContractError("too many mesh material groups")
                    groups.append(dict(start=len(corners), count=0, material=material))
                for i in range(1, len(face) - 1):
                    corners.extend((face[0], face[i], face[i + 1]))
                    groups[-1]["count"] += 3
                if len(corners) > MAX_TRIANGLES * 3:
                    raise ContractError("mesh triangle limit exceeded")
        if not corners:
            raise ContractError("mesh contains no faces")
        indices = np.array(corners)
        points = np.array(vertices)[indices[:, 0]]
        n = face_normals(points)
        tex = np.zeros((len(points), 2))
        if normals:
            mask = indices[:, 2] >= 0
            n[mask] = np.array(normals)[indices[mask, 2]]
        if uv:
            mask = indices[:, 1] >= 0
            tex[mask] = np.array(uv)[indices[mask, 1]]
        return points, n, tex, groups, materials


def numbers(values, count):
    result = [float(value) for value in values]
    if len(result) != count or not all(math.isfinite(x) and abs(x) <= 100 for x in result):
        raise ContractError("invalid mesh coordinates")
    return result


def obj_index(text, count):
    value = int(text)
    index = value - 1 if value > 0 else count + value
    if value == 0 or not 0 <= index < count:
        raise ContractError("OBJ index outside geometry")
    return index


def face_normals(points):
    triangles = np.asarray(points).reshape(-1, 3, 3)
    n = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    return np.repeat(n / np.maximum(norm, 1e-12), 3, axis=0)


def stl(data):
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84 + count * 50:
        if count > MAX_TRIANGLES:
            raise ContractError("mesh triangle limit exceeded")
        dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        points = np.frombuffer(data, dtype=dtype, count=count, offset=84)["vertices"].reshape(-1, 3)
    else:
        points = np.array([numbers(line.split()[1:], 3) for line in data.decode("ascii").splitlines()
                           if line.strip().startswith("vertex ")])
    if (not len(points) or len(points) % 3 or len(points) > MAX_TRIANGLES * 3
            or not np.isfinite(points).all() or np.abs(points).max() > 100):
        raise ContractError("invalid STL geometry")
    return points, face_normals(points)


def merge_visual_tree(model, visual):
    """Attach a visual-only model only when its measured joint frames match exactly."""
    mapping = {visual["root"]: model["root"]}
    target_joints = {j["name"]: j for j in model["joints"]}
    target_links = {link["name"]: link for link in model["links"]}
    for joint in visual["joints"]:
        target = target_joints.get(joint["name"])
        if target is not None:
            if (target["parent"] != mapping[joint["parent"]] or target["type"] != joint["type"]
                    or any(not np.allclose(target[key], joint[key], atol=1e-9, rtol=0)
                           for key in ("xyz", "rpy", "axis"))):
                raise ContractError("visual URDF joint frames differ from measured URDF")
            mapping[joint["child"]] = target["child"]
        elif joint["type"] == "fixed":
            child = f"visual:{joint['child']}"
            if child in target_links:
                raise ContractError("visual URDF link name conflicts")
            mapping[joint["child"]] = child
            model["joints"].append(dict(joint, name=f"visual:{joint['name']}",
                                        parent=mapping[joint["parent"]], child=child))
            link = dict(name=child, visuals=[])
            model["links"].append(link)
            target_links[child] = link
        else:
            raise ContractError("unmatched movable visual joint")
    for link in visual["links"]:
        target_links[mapping[link["name"]]]["visuals"] = copy.deepcopy(link["visuals"])
    return set(mapping.values())


def load_visual_model(path, visual_urdf=None, mesh_root=None):
    model, assets = from_urdf(path), VisualAssets()
    visual_links = set()
    if visual_urdf:
        visual = from_urdf(visual_urdf, RELIC_ALIASES)
        visual_links = merge_visual_tree(model, visual)
        model["visual_source"] = dict(name=visual_urdf.name, sha256=visual["sha256"])
    missing, count = [], 0
    for link in model["links"]:
        asset_urdf = visual_urdf if link["name"] in visual_links else path
        root = (mesh_root or asset_urdf.parent).resolve()
        for visual in link["visuals"]:
            geometry = visual["geometry"]
            if geometry["kind"] == "mesh":
                filename = geometry.pop("filename")
                try:
                    geometry.update(assets.mesh(filename, asset_urdf.parent, root))
                except (OSError, ValueError, UnicodeError, IndexError) as exc:
                    # Missing/unsupported visuals are visible in the link list. Never
                    # silently substitute collision geometry or expose a raw local path.
                    geometry["error"] = (str(exc) if isinstance(exc, ContractError)
                                         else "mesh unavailable or invalid")
                    missing.append(link["name"])
            if not geometry.get("error"):
                count += 1
    model.update(visual_count=count, missing_visuals=missing,
                 asset_manifest=list(assets.files.values()),
                 label=f"{len(model['links'])} links · {count} visuals"
                       + (" · ReLIC visual model" if visual_urdf else " · URDF"))
    return model, assets
