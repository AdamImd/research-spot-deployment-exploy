"""Released ReLIC inference for observation only; no robot command capability."""

import time

import numpy as np

from .contracts import ARM, JOINTS, ContractError, sha256
from .policy import rotation

CHECKPOINT_SHA256 = "039b542d7e833961b7602b4f933c10415589aa1ff1fa1a8295979f6a20c2f000"
POLICY_HZ = 50
ACTION_JOINTS = tuple(f"{leg}_{axis}" for axis in ("hx", "hy", "kn")
                      for leg in ("fl", "fr", "hl", "hr"))
OBS_JOINTS = ("arm0_sh0", *ACTION_JOINTS[:4], "arm0_sh1", *ACTION_JOINTS[4:8],
              "arm0_el0", *ACTION_JOINTS[8:], *ARM[3:])
OBS_IDS = [JOINTS.index(n) for n in OBS_JOINTS]
ACTION_IDS = [JOINTS.index(n) for n in ACTION_JOINTS]
DEFAULT_Q = np.array([.12, .5, -1, -.12, .5, -1, .12, .5, -1, -.12, .5, -1,
                      0, -.9, 1.8, 0, -.9, 0, -1.54], dtype=np.float32)
ROOT_COM_B = np.array([0, 0, -.00496172], dtype=np.float32)


def finite(value, size):
    value = np.asarray(value, dtype=np.float32)
    if value.shape != (size,) or not np.isfinite(value).all():
        raise ContractError(f"ReLIC requires {size} finite values")
    return value


def observation(state, arm_target, previous_action, body_height=.55):
    world_to_body = rotation(state).T
    angular = world_to_body @ state.angular_velocity_odom
    # Isaac observes root rigid-body COM velocity, SDK reports the body origin.
    linear = world_to_body @ state.linear_velocity_odom + np.cross(angular, ROOT_COM_B)
    q, dq = finite(state.positions, 19), finite(state.velocities, 19)
    return finite(np.concatenate((linear, angular, world_to_body @ [0, 0, -1],
        np.zeros(3), finite(arm_target, 7), np.zeros(12), [0, 0, body_height],
        q[OBS_IDS] - DEFAULT_Q[OBS_IDS], dq[OBS_IDS], finite(previous_action, 12))), 84)


def joint_targets(action, arm_target):
    targets = DEFAULT_Q.copy()
    targets[ACTION_IDS] += .2 * finite(action, 12)
    targets[12:] = finite(arm_target, 7)
    return finite(targets, 19)


class ReLICPolicy:
    def __init__(self, checkpoint, body_height=.55, *, device="cpu", profile_prefix=None):
        import onnxruntime as ort

        if sha256(checkpoint) != CHECKPOINT_SHA256:
            raise ContractError("released ReLIC checkpoint hash mismatch")
        if not np.isfinite(body_height) or not 0 < body_height <= 1.5:
            raise ContractError("ReLIC body height must be finite and in (0, 1.5] metres")
        self.body_height = float(body_height)
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        if device not in ("cpu", "cuda"):
            raise ContractError("ReLIC inference device must be cpu or cuda")
        if profile_prefix is not None:
            options.enable_profiling = True
            options.profile_file_prefix = str(profile_prefix)
        providers = ["CPUExecutionProvider"]
        if device == "cuda":
            if "CUDAExecutionProvider" not in ort.get_available_providers():
                raise ContractError("CUDA inference requested but CUDA provider is unavailable")
            # Preserve float32 parity and reject silent CPU placement/fallback.
            options.add_session_config_entry("session.disable_cpu_ep_fallback", "1")
            providers = [("CUDAExecutionProvider", {"device_id": 0, "use_tf32": 0})]
        self.session = ort.InferenceSession(str(checkpoint), sess_options=options,
                                           providers=providers)
        self.session.disable_fallback()
        expected = "CUDAExecutionProvider" if device == "cuda" else "CPUExecutionProvider"
        if self.session.get_providers()[0] != expected:
            raise ContractError("requested ReLIC inference provider did not initialize")
        self.device = device
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].name != "obs" or inputs[0].shape != [1, 84]
                or inputs[0].type != "tensor(float)" or len(outputs) != 1
                or outputs[0].name != "actions" or outputs[0].shape != [1, 12]):
            raise ContractError("unexpected ReLIC tensor contract")
        self.previous_action = np.zeros(12, dtype=np.float32)
        self.arm_target = None

    def predict(self, state):
        if self.arm_target is None:
            self.arm_target = finite(state.positions[12:], 7).copy()
        obs = observation(state, self.arm_target, self.previous_action, self.body_height)
        start = time.monotonic()
        action = finite(self.session.run(["actions"], {"obs": obs[None]})[0][0], 12)
        elapsed = time.monotonic() - start
        targets = joint_targets(action, self.arm_target)
        self.previous_action = action.copy()
        return targets, elapsed, obs, action
