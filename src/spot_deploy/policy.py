"""Pure observation/action conversion plus a CPU-only ONNX runner."""

from collections import deque
from time import monotonic

import numpy as np

from .contracts import ContractError, JOINTS, LEGS, Manifest, State


def rotation(state: State):
    # State rejects malformed quaternions; normalize accepted rounding drift.
    q = np.asarray(state.odom_quaternion_wxyz, dtype=np.float64)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class ObservationBuilder:
    def __init__(self, manifest: Manifest):
        self.manifest = manifest
        self.history = deque(maxlen=manifest.observations.history_length)
        self.previous_action = np.zeros(12, dtype=np.float32)

    def reset(self):
        self.history.clear()
        self.previous_action[:] = 0

    def build(self, state: State):
        spec = self.manifest.observations
        indices = [JOINTS.index(name) for name in spec.joint_order]
        defaults = dict(
            zip(self.manifest.actions.joint_order, self.manifest.actions.default_positions)
        )
        r = rotation(state).T
        data = {
            "body_linear_velocity": r @ state.linear_velocity_odom,
            "body_angular_velocity": r @ state.angular_velocity_odom,
            "projected_gravity": r @ [0, 0, -1],
            "joint_positions": np.array(state.positions)[indices]
            - np.array([defaults[n] for n in spec.joint_order]),
            "joint_velocities": np.array(state.velocities)[indices],
            # This field is in network output order, not observation joint order.
            "previous_action": self.previous_action,
            "velocity_command": np.zeros(3),
        }
        frame = np.concatenate(
            [(np.asarray(data[t.name]) - t.offset) * t.scale for t in spec.terms]
        )
        frame = (frame - spec.mean) / spec.std
        if not np.isfinite(frame).all():
            raise ContractError("non-finite observation")
        if spec.clip is not None:
            frame = np.clip(frame, -spec.clip, spec.clip)
        if not self.history:
            initial = (
                frame if spec.history_initialization == "repeat_first" else np.zeros_like(frame)
            )
            for _ in range(spec.history_length - 1):
                self.history.append(initial.copy())
        self.history.append(frame)
        with np.errstate(over="ignore"):
            obs = np.concatenate(self.history).astype(np.float32)[None, :]
        if not np.isfinite(obs).all():
            raise ContractError("observation exceeds float32 range")
        return obs

    def targets(self, output):
        spec = self.manifest.actions
        raw = np.asarray(output, dtype=np.float64)
        if raw.shape != (12,) or not np.isfinite(raw).all():
            raise ContractError("policy must produce twelve finite actions")
        # Exceeding the declared clip range is part of training semantics, not a safety fix.
        clipped = raw if spec.clip_min is None else np.clip(raw, spec.clip_min, spec.clip_max)
        legs = dict(zip(spec.joint_order, spec.default_positions + clipped * spec.scale))
        targets = np.array([legs[name] for name in LEGS] + self.manifest.arm_stowed_positions)
        if not np.isfinite(targets).all():
            raise ContractError("non-finite joint target")
        self.previous_action = (raw if spec.previous_action == "raw" else clipped).astype(
            np.float32
        )
        return targets


class OnnxPolicy:
    def __init__(self, path, manifest: Manifest):
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        self.manifest = manifest
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ContractError(
                "only one-input, one-output feed-forward ONNX policies are supported"
            )
        for actual, name, width in (
            (inputs[0], manifest.input_name, manifest.observations.input_size),
            (outputs[0], manifest.output_name, 12),
        ):
            if actual.name != name or actual.type != "tensor(float)" or actual.shape != [1, width]:
                raise ContractError(
                    "ONNX names, float32 type or fixed dimensions do not match manifest"
                )
        self.builder = ObservationBuilder(manifest)

    def reset(self):
        self.builder.reset()

    def predict(self, state):
        start = monotonic()
        obs = self.builder.build(state)
        action = self.session.run([self.manifest.output_name], {self.manifest.input_name: obs})[0]
        if action.shape != (1, 12):
            raise ContractError("invalid ONNX output shape")
        target = self.builder.targets(action[0])
        return target, monotonic() - start, obs, action[0]
