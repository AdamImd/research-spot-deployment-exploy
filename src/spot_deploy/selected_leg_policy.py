"""Explicit live selected-leg contract; never dispatch the simulation wrapper."""
import json
from typing import Literal

import numpy as np
import onnx
from pydantic import model_validator

from .contracts import JOINTS, ContractError, artifact_path, sha256
from .exploy_policy import ExploySettings, ExployReLICManifest, ExployReLICPolicy, INPUT_SIZES, META_KEY
from .leg_ik import LegPositionIK
from .simulation_demo import ThreeLegPlan, ThreeLegSequence, smoothstep, mix_entry, DemoTargetSlew

BASE_GRAPH_SHA = '1940df208bf9e30e6134eeb71d1c4e67d3a0d66778726507d4886c62eae662e8'


class SelectedLegSettings(ExploySettings):
    contract: Literal['relic-exploy-leg-v1']
    base_graph_file: str
    base_graph_sha256: Literal['1940df208bf9e30e6134eeb71d1c4e67d3a0d66778726507d4886c62eae662e8']


class SelectedLegManifest(ExployReLICManifest):
    schema_version: Literal[4]
    adapter: Literal['relic-exploy-leg']
    task: Literal['three-leg-demo']
    exploy: SelectedLegSettings
    demo: ThreeLegPlan

    @model_validator(mode='after')
    def selected_contract(self):
        if (self.demo.kind != 'front-left-cartesian-ik' or self.demo.return_blend_s != 2
                or self.demo.entry_blend_s != 0 or self.demo.target_slew_rad_s != 1
                or self.demo.duration_s >= 60 or self.relic.preparation.kind != 'direct'):
            raise ValueError('live demo requires front-left IK, native raw history,1rad/s target limiter and2s return blend')
        return self


class SelectedLegPolicy(ExployReLICPolicy):
    input_sizes = dict(INPUT_SIZES, leg_command=12)
    metadata_contract = 'relic-exploy-leg-v1'
    metadata_mode = 'front-left-selected-leg'

    def create_runner(self, path, manifest_path, device, profile_prefix):
        base = artifact_path(manifest_path, self.manifest.exploy.base_graph_file)
        if sha256(base) != BASE_GRAPH_SHA:
            raise ContractError('selected-leg observation source graph hash mismatch')
        model = onnx.load(base)
        self.base_metadata = json.loads(next(p.value for p in model.metadata_props if p.key == META_KEY))
        runner = super().create_runner(path, manifest_path, device, profile_prefix)
        meta = self.graph_metadata
        if (meta.get('base_graph_sha256') != BASE_GRAPH_SHA or meta.get('selected_leg') != 'fl'
                or meta.get('leg_command_joint_names') != list(JOINTS[:12])
                or meta['joint_names'] != list(JOINTS)):
            raise ContractError('selected-leg graph mapping or override contract mismatch')
        return runner

    def input_joint_names(self, metadata, output_names):
        # The new graph retains the pinned native input order; its OUTPUT is SDK
        # order. Do not conflate these two mappings using one names field.
        return self.base_metadata['joint_names']

    def reset(self):
        super().reset()
        self.ik = LegPositionIK(self.support, self.support_model, 'fl')
        self.sequence = ThreeLegSequence(self.manifest.demo, self.ik)
        self.command = np.zeros(12, dtype=np.float32)
        self.target_slew = DemoTargetSlew()
        self.proposed_targets = None

    def update_command(self, state, elapsed):
        self.command = self.sequence.update(state, elapsed)

    def inputs(self, state, previous, velocity_command=None):
        if np.any(self.command[3:]) or not np.all(np.isfinite(self.command)):
            raise ContractError('only the front-left leg can have an explicit command')
        result = super().inputs(state, previous, velocity_command)
        result['torso_command'] = np.asarray(self.sequence.torso_command(self.body_height),dtype=np.float32)[None]
        result['leg_command'] = self.command[None]
        return result

    def evaluate(self, state, previous, velocity_command=None):
        from time import monotonic
        start = monotonic()
        target, _, obs, action = super().evaluate(state, previous, velocity_command)
        if self.sequence.phase == 'three-leg-handover':
            values = super().inputs(state, previous, velocity_command)
            values['torso_command'] = np.asarray(self.sequence.torso_command(self.body_height),dtype=np.float32)[None]
            values['leg_command'] = np.zeros((1,12),dtype=np.float32)
            four_target, _, _, four_raw = self._evaluate_inputs(values, start)
            target, action = mix_entry(self.sequence, four_target, four_raw, target, action)
        if self.sequence.phase == 'four-foot-handover':
            p = self.manifest.demo
            begin = p.initial_hold_s+p.support_transfer_s+p.lift_s+p.hold_s+p.lower_s+p.cartesian_path.duration_s
            alpha = smoothstep((self.sequence.last_time-begin)/p.return_blend_s)
            ids = self.sequence.ids
            target[ids] = self.sequence.pose+alpha*(target[ids]-self.sequence.pose)
        self.proposed_targets = target.copy()
        target = self.target_slew.apply(self.sequence,target)
        return target, monotonic()-start, obs, action

    def demo_record(self, state):
        result = dict(phase=self.sequence.phase, leg_command=self.command.tolist(),
                      proposed_targets=self.proposed_targets.tolist(),
                      target_slew_rad_s=self.manifest.demo.target_slew_rad_s,
                      entry_blend_alpha=(self.sequence.entry_alpha
                                         if self.sequence.phase == 'three-leg-handover' else None))
        if self.sequence.leg:
            measured = self.ik.forward(np.asarray(state.positions)[self.sequence.ids])[0]
            result.update(ik_frame='body', ik_target_body_m=self.sequence.target_body_m.tolist(),
                ik_measured_body_m=measured.tolist(), ik_solve_residual_m=self.sequence.ik_residual_m,
                ik_tracking_error_m=float(np.linalg.norm(measured-self.sequence.target_body_m)))
        return result
