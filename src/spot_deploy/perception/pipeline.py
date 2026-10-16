"""Auditable capture, selection and current-frame cloud production."""
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from .cameras import PAIRS
from .geometry import masked_cloud, project_visible, voxel_fuse, write_ply
from .segmentation import projected_prompt


class Pipeline:
    def __init__(self, cameras, segmenter, output, voxel_m=.01):
        self.cameras, self.segmenter = cameras, segmenter
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.voxel_m = voxel_m
        self.frames, self.masks, self.views = {}, {}, {}
        self.cloud = np.empty((0, 3))
        self.colors = np.empty((0, 3), np.uint8)
        self.sources = np.empty(0, np.uint8)
        self.pending = None
        self.sequence = 0
        self.revision = 0
        self.last_valid_time = None
        self.status = 'select_chair'
        self.latency = 0.
        self.last_capture_local_s = None
        self.event('session', segmenter=segmenter.description, voxel_m=voxel_m,
                   frame='odom', units='m', max_tracking_gap_s=5,
                   tracking='current depth reprojection; no temporal accumulation')

    def event(self, name, **data):
        with (self.output/'events.jsonl').open('a') as out:
            out.write(json.dumps(dict(event=name, monotonic_s=time.monotonic(), **data))+'\n')

    def capture(self):
        self.sequence += 1
        self.directory = self.output/f'frame-{self.sequence:06d}'
        self.frames = self.cameras.capture(self.directory)
        self.last_capture_local_s = time.monotonic()
        self.pending = None
        self.masks, self.views = {}, {}
        self.revision += 1

    def initial_capture(self):
        try:
            self.capture()
            self.status = 'select_chair'
        except Exception:
            self.status = 'capture_error_try_again'
            raise
        finally:
            self.save_cloud()

    def preview(self, view, prompt, sequence):
        if sequence != self.sequence or view not in self.frames:
            raise ValueError('image changed; select again on the current image')
        self.pending = None
        start = time.monotonic()
        mask, score = self.segmenter.mask(self.frames[view].rgb, prompt)
        xyz, _, quality = masked_cloud(self.frames[view], mask)
        if len(xyz) < 20:
            raise ValueError('selected mask has fewer than 20 valid depth samples')
        self.pending = dict(view=view, mask=mask, prompt=prompt, score=score, quality=quality)
        self.revision += 1
        self.latency = time.monotonic()-start
        self.event('preview', frame=self.sequence, view=view, prompt=prompt, score=score,
                   quality=quality, latency_s=self.latency)
        return self.summary()

    def accept(self):
        start = time.monotonic()
        if self.pending is None:
            raise ValueError('preview a mask first')
        p = self.pending
        self.masks = {p['view']: p['mask']}
        self.views = {p['view']: dict(status='manually_selected', score=p['score'], **p['quality'])}
        self.pending = None
        self.fuse()
        anchor = self.cloud.copy()
        self.propagate(anchor, skip=p['view'])
        self.fuse()
        self.latency = time.monotonic()-start
        self.status = 'selected'
        self.last_valid_time = min(f.timestamp_s for f in self.frames.values())
        self.revision += 1
        self.save_cloud()
        self.event('accept', frame=self.sequence, view=p['view'], points=len(self.cloud))
        return self.summary()

    def propagate(self, anchor, skip=None):
        if self.segmenter.predictor is None:
            return
        for view, frame in self.frames.items():
            if view == skip:
                continue
            uv = project_visible(anchor, frame)
            try:
                prompt = projected_prompt(uv, frame.depth_m.shape)
                mask, score = self.segmenter.mask(frame.rgb, prompt)
                coverage = float(mask[uv[:, 1], uv[:, 0]].mean())
                xyz, _, quality = masked_cloud(frame, mask)
                if score < .6 or coverage < .6 or len(xyz) < 20:
                    raise ValueError('low model score, projected support or valid depth')
                # A known stationary chair must remain spatially associated. Reject leakage.
                lo, hi = anchor.min(axis=0)-.25, anchor.max(axis=0)+.25
                spatial = float(((xyz >= lo) & (xyz <= hi)).all(axis=1).mean())
                if spatial < .8:
                    raise ValueError('candidate cloud escaped the selected object region')
                self.masks[view] = mask
                self.views[view] = dict(status='propagated_unreviewed', score=score,
                                        projected_pixels=len(uv), support_fraction=coverage,
                                        spatial_fraction=spatial, **quality)
                self.event('propagation', frame=self.sequence, view=view, prompt=prompt,
                           **self.views[view])
            except ValueError as exc:
                self.views[view] = dict(status='needs_selection_or_not_visible', reason=str(exc),
                                       projected_pixels=len(uv))

    def step(self):
        start = time.monotonic()
        anchor = self.cloud.copy()
        previous_time = self.last_valid_time
        gap_limit = 300 if self.status == 'selected' else 5
        # Never retain a stale point cloud on capture, segmentation or association failure.
        self.cloud, self.colors, self.sources = np.empty((0, 3)), np.empty((0, 3), np.uint8), np.empty(0, np.uint8)
        self.masks, self.views, self.pending = {}, {}, None
        try:
            self.capture()
            now = min(f.timestamp_s for f in self.frames.values())
            if len(anchor) and previous_time is not None and 0 < now-previous_time <= gap_limit:
                self.propagate(anchor)
            self.fuse()
            self.status = 'tracking' if len(self.cloud) else 'lost_select_again'
            self.last_valid_time = now if len(self.cloud) else None
        except Exception:
            self.frames, self.masks, self.views = {}, {}, {}
            self.status = 'capture_or_segmentation_error'
            self.last_valid_time = None
            self.revision += 1
            raise
        finally:
            self.latency = time.monotonic()-start
            self.save_cloud()
        return self.summary()

    def fuse(self):
        points, colors, views = [], [], []
        for i, view in enumerate(PAIRS):
            if view not in self.masks:
                continue
            xyz, rgb, _ = masked_cloud(self.frames[view], self.masks[view])
            points.extend(xyz)
            colors.extend(rgb)
            views.extend([1 << i]*len(xyz))
        self.cloud, self.colors, self.sources = voxel_fuse(points, colors, views, self.voxel_m)

    def save_cloud(self):
        # A revision directory prevents selecting a new chair from overwriting earlier evidence.
        destination = self.output/f'revision-{self.revision:06d}'
        destination.mkdir(exist_ok=True)
        for view, mask in self.masks.items():
            Image.fromarray(mask.astype(np.uint8)*255).save(destination/(view+'-mask.png'))
        np.savez_compressed(destination/'chair.npz', xyz_odom=self.cloud.astype(np.float32),
                            rgb=self.colors, source_bits=self.sources,
                            timestamp_s=[f.timestamp_s for f in self.frames.values()])
        write_ply(destination/'chair.ply', self.cloud, self.colors)
        (destination/'summary.json').write_text(json.dumps(self.summary(), indent=2))
        temporary = self.output/'latest.json.tmp'
        temporary.write_text(json.dumps(dict(artifact_directory=destination.name,
                                              **self.summary()), indent=2))
        temporary.replace(self.output/'latest.json')

    def summary(self):
        return dict(sequence=self.sequence, revision=self.revision, status=self.status,
                    points=len(self.cloud), frame='odom', voxel_m=self.voxel_m,
                    inference_and_capture_s=self.latency, views=self.views,
                    snapshot_age_s=(None if self.last_capture_local_s is None else
                                    time.monotonic()-self.last_capture_local_s),
                    images={v: dict(width=f.rgb.shape[1], height=f.rgb.shape[0],
                                    timestamp_s=f.timestamp_s, **f.metadata)
                            for v, f in self.frames.items()},
                    pending=None if self.pending is None else {
                        k: v for k, v in self.pending.items() if k != 'mask'},
                    model=self.segmenter.description)
