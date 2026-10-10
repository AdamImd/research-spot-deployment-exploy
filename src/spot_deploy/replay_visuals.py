"""Restore all authored OBJ material groups in a replay-only MuJoCo model.

MuJoCo's OBJ importer retains one material subset from these ReLIC meshes.
Write each subset as an explicit MSH asset with its own material instead.
Never modify the simulation converter or advance dynamics here.
"""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from .contracts import ContractError, sha256
from .kinematics import from_urdf
from .records import atomic_json
from .visual_assets import RELIC_ALIASES, VisualAssets


def text(values):
    return ' '.join(str(float(v)) for v in values)


def rpy_quaternion(rpy):
    """URDF fixed-axis roll/pitch/yaw, quaternion wxyz."""
    cr, cp, cy = np.cos(np.asarray(rpy)/2)
    sr, sp, sy = np.sin(np.asarray(rpy)/2)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def write_mesh(path, corners, texture_scale):
    if not len(corners) or len(corners) % 3:
        raise ContractError('replay material group must contain complete triangles')
    corners = np.array(corners, dtype='<f4', copy=True)
    corners[:, 6:8] *= texture_scale
    vertices, inverse = np.unique(corners, axis=0, return_inverse=True)
    # MSH requires >=4 vertices, even when a material has only one triangle.
    if len(vertices) < 4:
        vertices = np.concatenate((vertices, np.repeat(vertices[:1], 4-len(vertices), axis=0)))
    count = len(vertices)
    with path.open('wb') as stream:
        stream.write(struct.pack('<4i', count, count, count, len(corners)//3))
        for block in (vertices[:, :3], vertices[:, 3:6], vertices[:, 6:8]):
            stream.write(block.astype('<f4').tobytes())
        stream.write(inverse.astype('<i4').tobytes())


def restore_visuals(canonical_path, urdf_path, output):
    """Return a separate MJCF path; preserve joints, collisions and inertials."""
    canonical_path, urdf_path, output = map(Path, (canonical_path, urdf_path, output))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    tree = ET.parse(canonical_path)
    root = tree.getroot()
    assets_xml = root.find('asset')
    if assets_xml is None:
        assets_xml = ET.SubElement(root, 'asset')
    bodies = {body.get('name'): body for body in root.findall('.//body')}
    old_meshes = set()
    for body in bodies.values():
        for geom in list(body.findall('geom')):
            if geom.get('contype') == '0' and geom.get('conaffinity') == '0':
                old_meshes.add(geom.get('mesh'))
                body.remove(geom)
    retained = {g.get('mesh') for g in root.findall('.//geom')}
    for mesh in list(assets_xml.findall('mesh')):
        if mesh.get('name') in old_meshes-retained:
            assets_xml.remove(mesh)
    visual_model = from_urdf(urdf_path, RELIC_ALIASES)
    assets = VisualAssets()
    groups, textures = [], {}
    total = 0
    for link in visual_model['links']:
        body = bodies.get(link['name'])
        if body is None:
            raise ContractError('replay visual link missing from kinematic model')
        for visual in link['visuals']:
            geometry = visual['geometry']
            if geometry['kind'] != 'mesh':
                raise ContractError('replay visual model currently requires authored meshes')
            spec = assets.mesh(geometry['filename'], urdf_path.parent, urdf_path.parent)
            corners = np.frombuffer(assets.data[spec['url']][0], dtype='<f4').reshape(-1, 8)
            for group in spec['groups']:
                key = f'replay_visual_{len(groups)}'
                material = spec['materials'].get(group['material'], {})
                rgba = visual['rgba'] or material.get('rgba', [.65, .68, .72, 1])
                attributes = dict(name=key, rgba=text(rgba), specular='.15', shininess='.15')
                url = material.get('texture') if not visual['rgba'] else None
                if url:
                    if url not in textures:
                        texture_name = f'replay_texture_{len(textures)}'
                        texture_path = output/f'{texture_name}.png'
                        texture_path.write_bytes(assets.data[url][0])
                        ET.SubElement(assets_xml, 'texture', name=texture_name, type='2d',
                                      file=str(texture_path))
                        textures[url] = texture_name
                    attributes['texture'] = textures[url]
                ET.SubElement(assets_xml, 'material', **attributes)
                mesh_path = output/f'{key}.msh'
                subset = corners[group['start']:group['start']+group['count']]
                scale = material.get('texture_scale', [1, 1]) if url else [1, 1]
                write_mesh(mesh_path, subset, scale)
                ET.SubElement(assets_xml, 'mesh', name=key, file=str(mesh_path),
                              scale=text(geometry['scale']), inertia='shell')
                ET.SubElement(body, 'geom', name=key, type='mesh', mesh=key, material=key,
                    pos=text(visual['xyz']), quat=text(rpy_quaternion(visual['rpy'])),
                    contype='0', conaffinity='0', group='1', mass='0')
                triangles = len(subset)//3
                total += triangles
                groups.append(dict(name=key, link=link['name'], material=group['material'],
                    triangles=triangles, source_sha256=spec['source_sha256'],
                    mesh_sha256=sha256(mesh_path), rgba=rgba, textured=bool(url)))
    # Preserve the original floor and key light, soften plastic surface lighting.
    visual_xml = root.find('visual')
    if visual_xml is None:
        visual_xml = ET.SubElement(root, 'visual')
    headlight = visual_xml.find('headlight')
    if headlight is None:
        headlight = ET.SubElement(visual_xml, 'headlight')
    headlight.attrib.update(ambient='.3 .3 .3', diffuse='.6 .6 .6', specular='.1 .1 .1')
    path = output/'replay.xml'
    tree.write(path)
    atomic_json(output/'visual-manifest.json', dict(
        kind='authored-material-groups', replay_only=True, new_dynamics=False,
        urdf_sha256=sha256(urdf_path), canonical_model_sha256=sha256(canonical_path),
        total_triangles=total, material_groups=groups, sources=list(assets.files.values()),
        links_without_authored_visuals=[link['name'] for link in visual_model['links']
                                       if not link['visuals']]))
    return path


def validate_visual_faces(model, manifest_path):
    """Fail rather than silently publish a replay with dropped material faces."""
    import json
    import mujoco as mj
    manifest = json.loads(Path(manifest_path).read_text())
    for group in manifest['material_groups']:
        mesh = mj.mj_name2id(model, mj.mjtObj.mjOBJ_MESH, group['name'])
        if mesh < 0 or model.mesh_facenum[mesh] != group['triangles']:
            raise ContractError('MuJoCo replay lost authored visual triangles')
