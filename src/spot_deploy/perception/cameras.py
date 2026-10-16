"""Read-only image capability. No lease, power, command, gripper or E-stop writers."""
import io
import json
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image

from .geometry import Frame

PAIRS = {
    'frontleft': ('frontleft_fisheye_image', 'frontleft_depth_in_visual_frame'),
    'frontright': ('frontright_fisheye_image', 'frontright_depth_in_visual_frame'),
    'hand': ('hand_color_image', 'hand_depth_in_hand_color_frame'),
}


def seconds(t):
    return t.seconds+t.nanos/1e9


def decode(response):
    from bosdyn.api.image_pb2 import Image as I, ImageResponse
    if response.status != ImageResponse.STATUS_OK:
        raise ValueError('camera response not OK')
    im = response.shot.image
    if im.format == I.FORMAT_JPEG:
        return np.array(Image.open(io.BytesIO(im.data)).convert('RGB'))
    if im.format != I.FORMAT_RAW:
        raise ValueError('unsupported image encoding')
    if im.pixel_format == I.PIXEL_FORMAT_DEPTH_U16:
        return np.frombuffer(im.data, '<u2').reshape(im.rows, im.cols).copy()
    channels = {I.PIXEL_FORMAT_GREYSCALE_U8: 1, I.PIXEL_FORMAT_RGB_U8: 3,
                I.PIXEL_FORMAT_RGBA_U8: 4}.get(im.pixel_format)
    if channels is None:
        raise ValueError('unsupported camera pixel format')
    a = np.frombuffer(im.data, np.uint8).reshape(im.rows, im.cols, channels)
    return np.repeat(a, 3, axis=2) if channels == 1 else a[:, :, :3].copy()


def intrinsics(source):
    if source.WhichOneof('camera_models') != 'pinhole':
        raise ValueError('camera model requires explicit rectification; only pinhole supported')
    k = source.pinhole.intrinsics
    return np.array([[k.focal_length.x, 0, k.principal_point.x],
                     [0, k.focal_length.y, k.principal_point.y], [0, 0, 1.]])


def pair_frame(view, color, depth, max_pair_skew_s=.08):
    from bosdyn.client.frame_helpers import get_a_tform_b
    if (color.source.name, depth.source.name) != PAIRS[view]:
        raise ValueError('unexpected image sources')
    if color.shot.frame_name_image_sensor != depth.shot.frame_name_image_sensor:
        raise ValueError('depth must be registered into the color optical frame')
    tc, td = seconds(color.shot.acquisition_time), seconds(depth.shot.acquisition_time)
    if abs(tc-td) > max_pair_skew_s:
        raise ValueError('RGB/depth timestamp skew exceeds limit')
    K = intrinsics(depth.source)
    if not np.allclose(K, intrinsics(color.source), atol=1e-5):
        raise ValueError('aligned RGB/depth intrinsics differ')
    scale = depth.source.depth_scale
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('invalid SDK depth scale')
    rgb, raw = decode(color), decode(depth)
    if raw.ndim != 2 or raw.dtype != np.uint16:
        raise ValueError('expected uint16 depth')
    d = raw.astype(np.float32)/scale
    d[(raw == 0) | (raw == 65535)] = np.nan
    name = depth.shot.frame_name_image_sensor
    poses = [get_a_tform_b(x.shot.transforms_snapshot, 'odom', name) for x in (color, depth)]
    if any(p is None for p in poses):
        raise ValueError('missing capture-time odom/camera transform')
    a, b = [p.to_matrix() for p in poses]
    translation = float(np.linalg.norm(a[:3, 3]-b[:3, 3]))
    angle = float(np.arccos(np.clip((np.trace(a[:3, :3].T@b[:3, :3])-1)/2, -1, 1)))
    if translation > .02 or angle > .05:
        raise ValueError('camera moved too far between aligned color and depth exposures')
    # Native front camera streams are sideways. Rotate pixels AND projection matrix.
    if view.startswith('front'):
        h = rgb.shape[0]
        rgb, d = np.rot90(rgb, -1).copy(), np.rot90(d, -1).copy()
        K = np.array([[0, -1, h-1], [1, 0, 0], [0, 0, 1]]) @ K
    return Frame(view, rgb, d, K, b, td, dict(
        color_source=color.source.name, depth_source=depth.source.name,
        color_timestamp_s=tc, depth_timestamp_s=td, pair_skew_s=abs(tc-td),
        depth_scale=scale, camera_frame=name, capture_motion_m=translation,
        capture_motion_rad=angle, display_clockwise_rotation=view.startswith('front'),
    )).validate()


class SpotCameras:
    def __init__(self, config):
        import bosdyn.client
        from bosdyn.client.image import ImageClient
        from bosdyn.client.robot_id import RobotIdClient
        self.timeout = config.rpc_timeout_s
        self.robot = bosdyn.client.create_standard_sdk('spot-chair-read-only').create_robot(
            config.endpoint)
        self.robot.authenticate(os.environ['BOSDYN_CLIENT_USERNAME'],
                                os.environ['BOSDYN_CLIENT_PASSWORD'], timeout=self.timeout)
        identity = self.robot.ensure_client(RobotIdClient.default_service_name).get_id(
            timeout=self.timeout)
        if identity.serial_number != config.expected_serial:
            raise ValueError('robot identity differs from local configuration')
        self.robot.time_sync.wait_for_sync(timeout_sec=self.timeout)
        self.client = self.robot.ensure_client(ImageClient.default_service_name)
        names = {s.name for s in self.client.list_image_sources(timeout=self.timeout)}
        if not set(sum(PAIRS.values(), ())) <= names:
            raise ValueError('required front/hand aligned camera sources unavailable')
        self.last_timestamp = -float('inf')

    def capture(self, output=None):
        from bosdyn.api.image_pb2 import Image
        from bosdyn.client.image import build_image_request
        names = sum(PAIRS.values(), ())
        responses = self.client.get_image([build_image_request(n, image_format=Image.FORMAT_RAW)
                                          for n in names], timeout=self.timeout)
        # Measure age on receipt, before first-use imports/decoding or disk writes.
        received = time.monotonic()
        now = seconds(self.robot.time_sync.endpoint.robot_timestamp_from_local_secs(time.time()))
        if output:
            output = Path(output)
            output.mkdir(parents=True, exist_ok=False)
            for i, response in enumerate(responses):
                # Indexed filenames cannot be influenced by an unexpected SDK source name.
                (output/f'response-{i}.pb').write_bytes(response.SerializeToString())
            (output/'receipt.json').write_text(json.dumps(dict(robot_receive_time_s=now,
                sources=[dict(name=r.source.name, timestamp_s=seconds(r.shot.acquisition_time),
                              age_s=now-seconds(r.shot.acquisition_time)) for r in responses]),
                indent=2))
        by_name = {x.source.name: x for x in responses}
        if set(by_name) != set(names) or len(responses) != len(names):
            raise ValueError('incomplete or duplicate camera response set')
        frames = {v: pair_frame(v, by_name[c], by_name[d]) for v, (c, d) in PAIRS.items()}
        times = [f.timestamp_s for f in frames.values()]
        if max(times)-min(times) > .1 or max(now-t for t in times) > .5 or min(now-t for t in times) < -.05:
            raise ValueError(f'camera bundle skew={max(times)-min(times):.3f}s, '
                             f'oldest receipt age={max(now-t for t in times):.3f}s; '
                             'limits 0.100/0.500s')
        if min(times) <= self.last_timestamp:
            raise ValueError('repeated or reordered capture timestamp')
        self.last_timestamp = min(times)
        for frame in frames.values():
            frame.metadata['receive_age_s'] = now-frame.timestamp_s
            frame.metadata['decode_and_record_s'] = time.monotonic()-received
        if output:
            for view, frame in frames.items():
                frame.save(output/(view+'.npz'))
        return frames

    def close(self):
        self.robot.time_sync.stop()
