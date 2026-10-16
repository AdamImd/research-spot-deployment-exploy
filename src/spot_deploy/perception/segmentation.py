"""SAM2 image prompting, with depth reprojection for view/time association.

This is not the paper's AOT tracker and assumes a static or slowly moving object.
"""
import hashlib
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


class Segmenter:
    def __init__(self, checkpoint=None, device='cuda:0'):
        self.predictor = None
        self.device = device
        self.description = {'backend': 'manual-polygon', 'checkpoint_sha256': None}
        if checkpoint:
            import torch
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            self.torch = torch
            path = Path(checkpoint)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != '7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69':
                raise ValueError('checkpoint must match the pinned official SAM2.1 tiny SHA256')
            torch.set_num_threads(2)
            self.predictor = SAM2ImagePredictor(build_sam2(
                'configs/sam2.1/sam2.1_hiera_t.yaml', str(path), device=device,
                apply_postprocessing=False))
            self.description = dict(backend='SAM2.1-hiera-tiny-image+depth-reprojection',
                                    device=device, torch_version=torch.__version__,
                                    checkpoint_sha256=digest)

    def mask(self, rgb, prompt):
        h, w = rgb.shape[:2]
        if prompt.get('polygon'):
            xy = np.asarray(prompt['polygon'], dtype=float)
            self.validate_xy(xy, w, h)
            if len(xy) < 3:
                raise ValueError('polygon needs at least three vertices')
            im = Image.new('1', (w, h))
            ImageDraw.Draw(im).polygon([tuple(x) for x in xy], fill=1)
            return np.array(im, dtype=bool), 1.
        if self.predictor is None:
            raise ValueError('SAM2 checkpoint not configured; use polygon mode')
        points = np.array(prompt.get('points', []), dtype=np.float32).reshape(-1, 2)
        labels = np.array(prompt.get('labels', []), dtype=np.int32)
        box = prompt.get('box')
        if len(points):
            self.validate_xy(points, w, h)
            if len(labels) != len(points) or not np.isin(labels, [0, 1]).all():
                raise ValueError('point labels must be 0/1 and match points')
        if box is not None:
            box = np.asarray(box, dtype=np.float32)
            if box.shape != (4,) or box[2] <= box[0] or box[3] <= box[1]:
                raise ValueError('box must be x0,y0,x1,y1 with positive area')
            self.validate_xy(box.reshape(2, 2), w, h)
        if not len(points) and box is None:
            raise ValueError('select a box or positive point first')
        with self.torch.inference_mode():
            self.predictor.set_image(rgb)
            masks, scores, _ = self.predictor.predict(
                point_coords=points if len(points) else None,
                point_labels=labels if len(points) else None, box=box,
                multimask_output=box is None)
        best = int(np.argmax(scores))
        return masks[best].astype(bool), float(scores[best])

    @staticmethod
    def validate_xy(points, w, h):
        if (points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all()
                or (points < 0).any() or (points[:, 0] >= w).any()
                or (points[:, 1] >= h).any()):
            raise ValueError('prompt coordinates must be finite image pixels')


def projected_prompt(uv, shape):
    """Spread positive prompts across actually depth-consistent visible pixels."""
    if len(uv) < 20:
        raise ValueError('too few depth-consistent projected pixels')
    h, w = shape
    lo, hi = uv.min(axis=0), uv.max(axis=0)
    if np.any(hi-lo < 3):
        raise ValueError('projected support is degenerate')
    candidates = [np.median(uv, axis=0), *uv[np.argmin(uv, axis=0)],
                  *uv[np.argmax(uv, axis=0)]]
    points = [uv[np.argmin(np.linalg.norm(uv-c, axis=1))].tolist() for c in candidates]
    return dict(points=points, labels=[1]*len(points),
                box=[max(0, int(lo[0])-15), max(0, int(lo[1])-15),
                     min(w-1, int(hi[0])+15), min(h-1, int(hi[1])+15)])
