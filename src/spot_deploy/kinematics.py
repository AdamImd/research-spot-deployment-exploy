"""URDF link tree and visual definitions; never used for control or limits."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .contracts import ContractError, JOINTS, sha256


def schematic():
    # Approximate Spot dimensions for an explicitly labeled visual aid, not a model.
    joints, tips = [], []
    for leg, x, y in (
        ("fl", 0.30, 0.06),
        ("fr", 0.30, -0.06),
        ("hl", -0.30, 0.06),
        ("hr", -0.30, -0.06),
    ):
        parent = "base"
        for suffix, xyz, axis in (
            ("hx", [x, y, 0], [1, 0, 0]),
            ("hy", [0, 0.11 if y > 0 else -0.11, 0], [0, 1, 0]),
            ("kn", [0.025, 0, -0.32], [0, 1, 0]),
        ):
            child = f"{leg}_{suffix}"
            joints.append(
                dict(
                    name=child,
                    parent=parent,
                    child=child,
                    xyz=xyz,
                    rpy=[0, 0, 0],
                    axis=axis,
                    type="revolute",
                )
            )
            parent = child
        tips.append(dict(parent=parent, xyz=[0, 0, -0.335]))
    parent = "base"
    for suffix, xyz, axis in (
        ("sh0", [0.29, 0, 0.188], [0, 0, 1]),
        ("sh1", [0, 0, 0], [0, 1, 0]),
        ("el0", [0.34, 0, 0], [0, 1, 0]),
        ("el1", [0.40, 0, 0.075], [1, 0, 0]),
        ("wr0", [0, 0, 0], [0, 1, 0]),
        ("wr1", [0, 0, 0], [1, 0, 0]),
        ("f1x", [0.117, 0, 0.015], [0, 1, 0]),
    ):
        child = f"arm0_{suffix}"
        joints.append(
            dict(
                name=child,
                parent=parent,
                child=child,
                xyz=xyz,
                rpy=[0, 0, 0],
                axis=axis,
                type="revolute",
            )
        )
        parent = child
    tips.append(dict(parent=parent, xyz=[0.1, 0, 0]))
    return dict(
        root="base",
        joints=joints,
        tips=tips,
        links=[dict(name=name, visuals=[]) for name in ["base", *[j["child"] for j in joints]]],
        label="Approximate schematic · not calibrated",
        sha256=None,
    )


def vector(text, default):
    values = [float(x) for x in text.split()] if text else default
    if len(values) != 3 or not all(math.isfinite(x) and abs(x) <= 100 for x in values):
        raise ContractError("invalid URDF vector")
    return values


def color(element):
    if element is None:
        return None
    rgba = [float(x) for x in element.attrib["rgba"].split()]
    if len(rgba) != 4 or not all(math.isfinite(x) and 0 <= x <= 1 for x in rgba):
        raise ContractError("invalid URDF visual color")
    return rgba


def visuals(link, materials):
    result = []
    for element in link.findall("visual"):
        if len(result) >= 32:
            raise ContractError("too many visuals on a link")
        origin, material = element.find("origin"), element.find("material")
        rgba = None
        if material is not None:
            rgba = color(material.find("color")) or materials.get(material.get("name"))
        geometry = element.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise ContractError("invalid URDF visual geometry")
        shape = geometry[0]
        data = dict(kind=shape.tag)
        if shape.tag == "mesh":
            data.update(filename=shape.attrib["filename"],
                        scale=vector(shape.get("scale"), [1, 1, 1]))
            if any(x == 0 for x in data["scale"]):
                raise ContractError("zero visual mesh scale")
        elif shape.tag == "box":
            data["size"] = vector(shape.attrib["size"], [0, 0, 0])
        elif shape.tag in ("sphere", "cylinder"):
            for field in ("radius", "length") if shape.tag == "cylinder" else ("radius",):
                data[field] = float(shape.attrib[field])
        else:
            raise ContractError("unsupported URDF visual geometry")
        for key in ("size", "radius", "length"):
            values = data.get(key, [])
            values = values if isinstance(values, list) else [values]
            if any(not math.isfinite(x) or not 0 < x <= 100 for x in values):
                raise ContractError("invalid URDF visual dimensions")
        result.append(dict(
            xyz=vector(origin.get("xyz") if origin is not None else None, [0, 0, 0]),
            rpy=vector(origin.get("rpy") if origin is not None else None, [0, 0, 0]),
            rgba=rgba, geometry=data,
        ))
    return result


def from_urdf(path: Path, joint_aliases=None):
    if path.stat().st_size > 4_000_000:
        raise ContractError("URDF exceeds viewer size limit")
    raw = path.read_text()
    if "<!" in raw:
        # Comments are normal; DTDs/entities are unnecessary for this input format.
        if "<!DOCTYPE" in raw.upper() or "<!ENTITY" in raw.upper():
            raise ContractError("URDF entities are unsupported")
    try:
        tree = ET.fromstring(raw)
        joints = []
        for element in tree.findall("joint"):
            kind = element.attrib["type"]
            if kind not in ("fixed", "revolute", "continuous", "prismatic"):
                raise ContractError("unsupported URDF joint type")
            origin, axis = element.find("origin"), element.find("axis")
            joints.append(
                dict(
                    name=(joint_aliases or {}).get(element.attrib["name"],
                                                  element.attrib["name"].replace(".", "_")),
                    parent=element.find("parent").attrib["link"],
                    child=element.find("child").attrib["link"],
                    type=kind,
                    xyz=vector(origin.get("xyz") if origin is not None else None, [0, 0, 0]),
                    rpy=vector(origin.get("rpy") if origin is not None else None, [0, 0, 0]),
                    axis=vector(axis.get("xyz") if axis is not None else None, [1, 0, 0]),
                )
            )
        names = [j["name"] for j in joints]
        children = [j["child"] for j in joints]
        roots = set(j["parent"] for j in joints) - set(children)
        if (
            len(joints) > 256
            or len(roots) != 1
            or len(names) != len(set(names))
            or len(children) != len(set(children))
        ):
            raise ContractError("URDF must be a unique bounded tree")
        if not set(JOINTS).issubset(names):
            raise ContractError("URDF does not contain all 19 Spot joints")
        if any(j["type"] != "fixed" and j["name"] not in JOINTS for j in joints):
            raise ContractError("URDF contains unmeasured movable joints")
        root = next(iter(roots))
        seen, ordered, remaining = {root}, [], joints.copy()
        while remaining:
            ready = [j for j in remaining if j["parent"] in seen]
            if not ready:
                raise ContractError("URDF is disconnected or cyclic")
            for joint in ready:
                if joint["type"] != "fixed" and sum(v * v for v in joint["axis"]) < 1e-12:
                    raise ContractError("URDF has a zero joint axis")
                ordered.append(joint)
                seen.add(joint["child"])
                remaining.remove(joint)
        declared = tree.findall("link")
        link_names = [link.attrib["name"] for link in declared]
        if declared and (len(set(link_names)) != len(link_names) or set(link_names) != seen):
            raise ContractError("URDF link declarations must match the complete joint tree")
        materials = {m.attrib["name"]: color(m.find("color")) for m in tree.findall("material")}
        links = [dict(name=link.attrib["name"], visuals=visuals(link, materials))
                 for link in declared] if declared else [dict(name=name, visuals=[])
                                                        for name in sorted(seen)]
        return dict(
            root=root,
            joints=ordered,
            tips=[],
            links=links,
            label=f"URDF · {len(links)} links",
            sha256=sha256(path),
        )
    except (ET.ParseError, KeyError, AttributeError, ValueError) as exc:
        if isinstance(exc, ContractError):
            raise
        raise ContractError("invalid URDF kinematic tree") from None
