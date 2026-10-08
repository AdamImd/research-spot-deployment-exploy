"""The released ReLIC 84-D observation / 12-D action contract, by joint name."""
from dataclasses import dataclass, field
from pathlib import Path
import hashlib
import importlib.util
import json

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "source/relic/relic/assets/spot"
spec = importlib.util.spec_from_file_location("relic_constants", ASSET / "constants.py")
C = importlib.util.module_from_spec(spec)
spec.loader.exec_module(C)
ARM = tuple(C.ARM_JOINT_NAMES)
LEGS = tuple(C.LEG_JOINT_NAMES)
LEG_COMMAND_ORDER = tuple(n for leg in LEGS for n in C.LEG_JOINT_NAMES[leg])
POLICY_JOINTS = tuple(f"{leg}_{kind}" for kind in ("hx", "hy", "kn") for leg in LEGS)
DT = 0.005
DECIMATION = 4
POLICY_DT = DT * DECIMATION
POLICY_HASH = "039b542d7e833961b7602b4f933c10415589aa1ff1fa1a8295979f6a20c2f000"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def defaults(names):
    return np.array([C.SPOT_DEFAULT_JOINT_POS[n] for n in names], dtype=np.float32)


def vector(x, size, label):
    x = np.asarray(x, dtype=np.float32)
    if x.shape != (size,) or not np.isfinite(x).all():
        raise ValueError(f"Invalid {label}: expected finite ({size},), got {x.shape}")
    return x


@dataclass
class Command:
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    arm: np.ndarray = field(default_factory=lambda: defaults(ARM))
    leg: str | None = None
    leg_pose: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    torso: np.ndarray = field(default_factory=lambda: np.array([0, 0, .55], dtype=np.float32))

    def observation(self):
        legs = np.zeros(12, dtype=np.float32)
        if self.leg is not None:
            idx = LEGS.index(self.leg) * 3
            legs[idx:idx + 3] = vector(self.leg_pose, 3, "leg pose")
        return np.concatenate((vector(self.arm, 7, "arm"), legs, vector(self.torso, 3, "torso")))


class Contract:
    def __init__(self, joint_names):
        self.names = tuple(joint_names)
        if len(self.names) != 19 or set(self.names) != set(C.SPOT_DEFAULT_JOINT_POS):
            raise ValueError("Expected all 19 released Spot joints exactly once")
        if tuple(n for n in self.names if n in POLICY_JOINTS) != POLICY_JOINTS:
            raise ValueError("Isaac articulation leg order differs from the released policy order")
        self.q0 = defaults(self.names)
        self.leg_ids = np.array([self.names.index(n) for n in POLICY_JOINTS])
        self.arm_ids = np.array([self.names.index(n) for n in ARM])

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text())["joint_names"])

    def observation(self, state, command, last_action):
        # All velocities and gravity are expressed in the base frame. Joint
        # observations use the actual Isaac articulation order, including arm.
        return vector(np.concatenate((
            vector(state["linear_velocity"], 3, "linear velocity"),
            vector(state["angular_velocity"], 3, "angular velocity"),
            vector(state["gravity"], 3, "gravity"),
            vector(command.velocity, 3, "velocity command"), command.observation(),
            vector(state["q"], 19, "q") - self.q0,
            vector(state["dq"], 19, "dq"), vector(last_action, 12, "last raw action"),
        )), 84, "policy observation")

    def targets(self, action, command):
        target = self.q0.copy()
        target[self.leg_ids] += .2 * vector(action, 12, "policy action")
        target[self.arm_ids] = vector(command.arm, 7, "arm target")
        if command.leg is not None:
            ids = [self.names.index(n) for n in C.LEG_JOINT_NAMES[command.leg]]
            target[ids] = vector(command.leg_pose, 3, "commanded leg")
        return target


class Policy:
    def __init__(self):
        import onnxruntime as ort
        path = ASSET / "pretrained/policy.onnx"
        if sha256(path) != POLICY_HASH:
            raise ValueError("Released ONNX checkpoint hash mismatch")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])
        if self.session.get_inputs()[0].shape != [1, 84]:
            raise ValueError("Unexpected policy input shape")

    def __call__(self, observation):
        result = self.session.run(["actions"], {"obs": vector(observation, 84, "observation")[None]})[0][0]
        return vector(result, 12, "action")
