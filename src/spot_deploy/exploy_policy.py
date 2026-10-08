"""Native environment-exported ReLIC graph; no Torch, Isaac or Exploy dependency at runtime."""

import json
from time import monotonic
from types import SimpleNamespace
from typing import Literal

import numpy as np

from .contracts import JOINTS, ContractError, Hash, StrictModel, artifact_path, sha256
from .policy import rotation
from .relic_contract import ReLICDeploymentPolicy, ReLICManifest
from .relic_policy import ACTION_JOINTS, CHECKPOINT_SHA256, DEFAULT_Q, ROOT_COM_B, finite

EXPLOY_COMMIT = "05f9dc3b2e589abdae8de942e8c6749e7a123c88"
SCRIPT_SHA256 = "14ed0614f507e1330a5c66af1615f27558b58b6c999443b42d1bb6c3d45ae3b4"
META_KEY = "spot_deploy.relic_exploy"
TARGET = "output.joint_targets.robot.all.pos"
INPUT_SIZES = {"base_quaternion": 4, "base_linear_velocity": 3, "base_angular_velocity": 3,
               "joint_positions": 19, "joint_velocities": 19, "arm_command": 7,
               "torso_command": 3, "velocity_command": 3, "previous_actions": 12}
OUTPUT_SIZES = {"actions": 12, "obs": 84, TARGET: 19}


class ExploySettings(StrictModel):
    contract: Literal["relic-exploy-v1"]
    exporter_commit: Literal["05f9dc3b2e589abdae8de942e8c6749e7a123c88"]
    source_policy_sha256: Literal["039b542d7e833961b7602b4f933c10415589aa1ff1fa1a8295979f6a20c2f000"]
    source_script_sha256: Literal["14ed0614f507e1330a5c66af1615f27558b58b6c999443b42d1bb6c3d45ae3b4"]
    export_record_file: str
    export_record_sha256: Hash
    configuration_file: str
    configuration_sha256: Hash


class ExployReLICManifest(ReLICManifest):
    schema_version: Literal[3]
    adapter: Literal["relic-exploy"]
    exploy: ExploySettings

    def validate_policy_reference(self):
        if self.policy_sha256 == CHECKPOINT_SHA256:
            raise ValueError("Exploy needs the environment-exported graph, not the raw actor")
        if self.input_name != "named-state-v1" or self.output_name != TARGET:
            raise ValueError("Exploy requires named state inputs and all-joint output")


def validate_metadata(metadata, manifest):
    expected = {"contract": "relic-exploy-v1", "exploy_commit": EXPLOY_COMMIT,
                "source_onnx_sha256": CHECKPOINT_SHA256, "source_torchscript_sha256": SCRIPT_SHA256,
                "policy_hz": 50, "stream_hz": 200, "graph": "policy-step",
                "mode": "four-foot-standing", "target_output": TARGET,
                "history": "external-acknowledged-raw-actions; zero at activation"}
    if any(metadata.get(k) != v for k, v in expected.items()):
        raise ContractError("Exploy graph provenance or execution contract mismatch")
    names = metadata.get("joint_names", [])
    if len(names) != 19 or set(names) != set(JOINTS):
        raise ContractError("Exploy graph must name each of the 19 Spot joints exactly once")
    if metadata.get("action_joint_names") != list(ACTION_JOINTS):
        raise ContractError("Exploy raw action order differs from the released ReLIC contract")
    if not np.allclose(finite(metadata.get("root_com_b"), 3), ROOT_COM_B, rtol=0, atol=1e-7):
        raise ContractError("Exploy base COM differs from the reviewed SDK frame conversion")
    gains = metadata.get("gains", {})
    for i, name in enumerate(JOINTS):
        pair = gains.get(name, {})
        if (pair.get("stiffness") != manifest.gains.kp[i]
                or pair.get("damping") != manifest.gains.kd[i]):
            raise ContractError("Exploy graph gains differ from manifest")
    return names


class ExployReLICPolicy(ReLICDeploymentPolicy):
    def create_runner(self, path, manifest_path, device, profile_prefix):
        import onnxruntime as ort

        # CPU is the measured default for this small policy. Reject an untested
        # provider request rather than silently falling back or changing precision.
        if device != "cpu":
            raise ContractError("Exploy variant currently supports the validated CPU provider only")
        if sha256(path) != self.manifest.policy_sha256:
            raise ContractError("Exploy policy hash mismatch")
        spec = self.manifest.exploy
        for name, digest in ((spec.export_record_file, spec.export_record_sha256),
                             (spec.configuration_file, spec.configuration_sha256)):
            if sha256(artifact_path(manifest_path, name)) != digest:
                raise ContractError("Exploy export provenance hash mismatch")
        record = json.loads(artifact_path(manifest_path, spec.export_record_file).read_text())
        if (record.get("policy_sha256") != self.manifest.policy_sha256
                or record.get("configuration_sha256") != spec.configuration_sha256
                or record.get("source_policy_sha256") != CHECKPOINT_SHA256
                or record.get("source_script_sha256") != SCRIPT_SHA256):
            raise ContractError("Exploy export record does not describe this graph")
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        if profile_prefix is not None:
            options.enable_profiling = True
            options.profile_file_prefix = str(profile_prefix)
        session = ort.InferenceSession(str(path), sess_options=options,
                                       providers=["CPUExecutionProvider"])
        session.disable_fallback()
        for fields, sizes in ((session.get_inputs(), INPUT_SIZES),
                              (session.get_outputs(), OUTPUT_SIZES)):
            if {f.name for f in fields} != set(sizes) or any(
                f.shape != [1, sizes[f.name]] or f.type != "tensor(float)" for f in fields
            ):
                raise ContractError("Unexpected Exploy graph tensor contract")
        try:
            metadata = json.loads(session.get_modelmeta().custom_metadata_map[META_KEY])
        except (KeyError, TypeError, ValueError) as exc:
            raise ContractError("Missing or invalid Exploy deployment metadata") from exc
        names = validate_metadata(metadata, self.manifest)
        self.graph_from_sdk = [JOINTS.index(n) for n in names]
        self.sdk_from_graph = [names.index(n) for n in JOINTS]
        self.graph_metadata = metadata
        return SimpleNamespace(session=session)

    def inputs(self, state, previous):
        if self.body_height is None:
            raise ContractError("Initial ReLIC body height has not been latched")
        world_to_body = rotation(state).T
        angular = world_to_body @ state.angular_velocity_odom
        linear = world_to_body @ state.linear_velocity_odom + np.cross(angular, ROOT_COM_B)
        quat = np.asarray(state.odom_quaternion_wxyz, dtype=float)
        quat /= np.linalg.norm(quat)
        values = dict(base_quaternion=quat, base_linear_velocity=linear,
            base_angular_velocity=angular,
            joint_positions=finite(state.positions, 19)[self.graph_from_sdk],
            joint_velocities=finite(state.velocities, 19)[self.graph_from_sdk],
            arm_command=self.manifest.arm_stowed_positions,
            torso_command=[0., 0., self.body_height], velocity_command=[0., 0., 0.],
            previous_actions=previous)
        return {k: finite(v, INPUT_SIZES[k])[None] for k, v in values.items()}

    def evaluate(self, state, previous):
        start = monotonic()
        action, obs, target = self.runner.session.run(["actions", "obs", TARGET],
                                                     self.inputs(state, previous))
        action, obs = finite(action[0], 12), finite(obs[0], 84)
        target = finite(target[0], 19)[self.sdk_from_graph].astype(float)
        # Preserve the exact held-arm values in SDK packets, as the original
        # adapter does, after confirming the graph produces the same float32 pose.
        arm = np.asarray(self.manifest.arm_stowed_positions, dtype=np.float32)
        if not np.array_equal(target[12:].astype(np.float32), arm):
            raise ContractError("Exploy output changed the held arm command")
        target[12:] = self.manifest.arm_stowed_positions
        return target, monotonic() - start, obs[None], action

    def warmup(self):
        values = {k: np.zeros((1, n), dtype=np.float32) for k, n in INPUT_SIZES.items()}
        values["base_quaternion"][0, 0] = 1
        values["joint_positions"][0] = DEFAULT_Q[self.graph_from_sdk]
        values["arm_command"][0] = self.manifest.arm_stowed_positions
        values["torso_command"][0, 2] = self.manifest.relic.body_height_m or .55
        self.runner.session.run(["actions"], values)
