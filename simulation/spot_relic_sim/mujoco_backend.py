"""MuJoCo port of the released URDF and explicit delayed PD actuators."""
from collections import deque
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
from .contract import ASSET, ARM, LEGS, C, DT, defaults
from .collision_mesh import compact_obj


def convert_model(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    urdf = ET.parse(ASSET / "spot_with_arm.urdf")
    collision_meshes = {}
    for mesh in urdf.findall(".//collision/geometry/mesh"):
        relative = Path(mesh.attrib["filename"])
        if relative.suffix.lower() == ".obj":
            prepared = output / "collision_meshes" / relative
            if str(relative) not in collision_meshes:
                collision_meshes[str(relative)] = compact_obj(ASSET / relative, prepared)
            mesh.set("filename", str(prepared))
    (output / "collision-meshes.json").write_text(json.dumps(collision_meshes, indent=2) + "\n")
    for mesh in urdf.findall(".//mesh"):
        mesh.set("filename", str(ASSET / mesh.attrib["filename"]))
    extension = ET.SubElement(urdf.getroot(), "mujoco")
    ET.SubElement(extension, "compiler", fusestatic="false", discardvisual="false",
                  strippath="false", inertiafromgeom="false")
    urdf.write(output / "source.urdf")
    model = mj.MjModel.from_xml_path(str(output / "source.urdf"))
    mj.mj_saveLastXML(str(output / "imported.xml"), model)
    # Canonical MJCF recomputes the URDF's omitted inertials at default density.
    # Capture that geometry-derived tensor before rescaling it below.
    inferred = mj.MjModel.from_xml_path(str(output / "imported.xml"))
    xml = ET.parse(output / "imported.xml")
    root = xml.getroot()
    ET.SubElement(root, "option", timestep=str(DT), gravity="0 0 -9.81",
                  integrator="implicitfast", solver="Newton", iterations="100", tolerance="1e-8")
    world = root.find("worldbody")
    body = world.find("body[@name='body']")
    if body is None:
        raise ValueError("URDF importer did not preserve root body")
    body.set("pos", "0 0 .65")
    ET.SubElement(body, "freejoint", name="floating_base")
    for link in body.iter("body"):
        if link.find("inertial") is None:
            idx = mj.mj_name2id(inferred, mj.mjtObj.mjOBJ_BODY, link.attrib["name"])
            # Scale inferred inertials after decomposition; decomposing a
            # 1e-15 inertia tensor directly hits MuJoCo's eigenvalue tolerance.
            scale = 1e-8 / 1000
            ET.SubElement(link, "inertial",
                pos=" ".join(map(str, inferred.body_ipos[idx])),
                quat=" ".join(map(str, inferred.body_iquat[idx])),
                mass=str(inferred.body_mass[idx] * scale),
                diaginertia=" ".join(map(str, inferred.body_inertia[idx] * scale)))
    ET.SubElement(world, "geom", name="floor", type="plane", size="100 100 .1",
                  friction=".8 .005 .0001", rgba=".18 .22 .28 1")
    ET.SubElement(world, "light", pos="0 0 5", dir="0 0 -1", diffuse=".8 .8 .8")
    for joint in body.findall(".//joint"):
        joint.set("damping", "0")
        joint.set("frictionloss", "0")
        joint.set("armature", str(C.ARM_ARMATURE[ARM.index(joint.attrib["name"])])
                  if joint.attrib["name"] in ARM else "0")
    for geom in body.findall(".//geom"):
        # The URDF omits inertials for feet and the fixed jaw. Isaac imports
        # these with link_density=1e-8; MuJoCo otherwise adds ~0.87 kg at its
        # default density of 1000. Explicit URDF inertials still take priority.
        geom.set("density", "1e-8")
        geom.set("friction", ".8 .005 .0001")
        if geom.get("contype") == "0":
            geom.set("rgba", ".83 .66 .15 1")
    xml.write(output / "spot.xml")
    return output / "spot.xml"


class MujocoBackend:
    def __init__(self, contract, output, scenario, physics_dt=DT):
        if not np.isfinite(physics_dt) or not 0 < physics_dt <= DT:
            raise ValueError("MuJoCo physics step must be positive and at most 5 ms")
        self.contract = contract
        self.model = mj.MjModel.from_xml_path(str(convert_model(output)))
        self.model.opt.timestep = float(physics_dt)
        self.data = mj.MjData(self.model)
        self.root = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "body")
        self.feet = [mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, f"{leg}_foot") for leg in LEGS]
        self.joints = np.array([mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, n) for n in contract.names])
        self.qids = self.model.jnt_qposadr[self.joints]
        self.dids = self.model.jnt_dofadr[self.joints]
        if self.model.nq != 26 or self.model.nv != 25 or np.any(self.joints < 0):
            raise ValueError("Expected floating base and 19 one-DOF joints")
        self.kp = np.array([C.ARM_STIFFNESS[ARM.index(n)] if n in ARM else 60 for n in contract.names])
        self.kd = np.array([C.ARM_DAMPING[ARM.index(n)] if n in ARM else 1.5 for n in contract.names])
        self.limit = np.array([C.ARM_EFFORT_LIMIT[ARM.index(n)] if n in ARM else
                               np.inf if n.endswith("_kn") else 45 for n in contract.names])
        self.knees = np.array([i for i, n in enumerate(contract.names) if n.endswith("_kn")])
        self.table = np.array(C.JOINT_PARAMETER_LOOKUP_TABLE)
        self.pos_speed = np.array(C.POS_TORQUE_SPEED_LIMIT)
        self.neg_speed = np.array(C.NEG_TORQUE_SPEED_LIMIT)
        self.base_masses = self.model.body_mass.copy()
        self.viewer = None
        self.reset(scenario)

    def reset(self, scenario):
        self.model.body_mass[:] = self.base_masses
        self.model.body_mass[self.root] += scenario.mass_delta
        self.model.geom_friction[:, 0] = scenario.friction
        mj.mj_setConst(self.model, self.data)
        mj.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = [0, 0, .65]
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[self.qids] = defaults(self.contract.names)
        self.history = deque(maxlen=scenario.delay + 1)
        self.delay = scenario.delay
        mj.mj_forward(self.model, self.data)

    def state(self):
        # mjOBJ_BODY uses the body's center of mass. Rotate the world velocity
        # by the link orientation, matching Isaac root_com_lin_vel_b.
        velocity = np.empty(6)
        mj.mj_objectVelocity(self.model, self.data, mj.mjtObj.mjOBJ_BODY, self.root, velocity, 0)
        rotation = self.data.xmat[self.root].reshape(3, 3)
        return {key: np.asarray(value, dtype=np.float32).copy() for key, value in dict(
            position=self.data.xpos[self.root],
            linear_velocity=rotation.T @ velocity[3:], angular_velocity=rotation.T @ velocity[:3],
            gravity=rotation.T @ np.array([0, 0, -1]),
            q=self.data.qpos[self.qids], dq=self.data.qvel[self.dids], feet=self.data.xpos[self.feet],
        ).items()}

    @property
    def physics_time(self):
        return float(self.data.time)

    def torque(self, target):
        self.history.append(np.asarray(target).copy())
        target = self.history[max(0, len(self.history) - 1 - self.delay)]
        q, dq = self.data.qpos[self.qids], self.data.qvel[self.dids]
        torque = np.clip(self.kp * (target - q) - self.kd * dq, -self.limit, self.limit)
        k = self.knees
        limit = np.interp(q[k], self.table[:, 0], self.table[:, 2])
        torque[k] = np.clip(torque[k], -limit, limit)
        positive = np.interp(dq[k], self.pos_speed[:, 0], self.pos_speed[:, 1])
        negative = np.interp(dq[k], self.neg_speed[:, 0], self.neg_speed[:, 1])
        torque[k] = np.clip(torque[k], negative, positive)
        return torque

    def step(self, target):
        self.data.qfrc_applied[self.dids] = self.torque(target)
        mj.mj_step(self.model, self.data)
        # mj_step leaves Cartesian caches at the previous state.
        mj.mj_forward(self.model, self.data)

    def push(self, dvy):
        self.data.qvel[1] += dvy
        mj.mj_forward(self.model, self.data)

    def metadata(self):
        return dict(body_names=[mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
                                for i in range(1, self.model.nbody)],
                    body_mass=self.model.body_mass[1:].tolist(),
                    total_mass=float(self.model.body_mass.sum()),
                    joint_limits=self.model.jnt_range[self.joints].tolist(),
                    inertia=self.model.body_inertia[1:].tolist(),
                    center_of_mass=self.model.body_ipos[1:].tolist(),
                    state={k: v.tolist() for k, v in self.state().items()})

    def render(self):
        if self.viewer:
            self.viewer.sync()

    def close(self):
        if self.viewer:
            self.viewer.close()
