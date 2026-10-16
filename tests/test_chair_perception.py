"""Geometry, calibration integrity, masks, stale-output and read-only regression tests."""
import ast
from pathlib import Path

import numpy as np
import pytest

from spot_deploy.perception.geometry import Frame, masked_cloud, project_visible, voxel_fuse
from spot_deploy.perception.pipeline import Pipeline
from spot_deploy.perception.segmentation import Segmenter, projected_prompt


def frame(view='hand', t=1.):
    return Frame(view, np.full((8, 10, 3), 100, np.uint8), np.ones((8, 10), np.float32),
                 np.array([[2., 0, 3], [0, 4, 2], [0, 0, 1]]), np.eye(4), t, {})


def test_unprojection_and_transform():
    f = frame()
    f.odom_T_camera[:3, 3] = [10, 20, 30]
    m = np.zeros((8, 10), bool)
    m[6, 5] = True
    xyz, rgb, info = masked_cloud(f, m, erosion=0)
    np.testing.assert_allclose(xyz, [[11, 21, 31]])
    assert rgb.tolist() == [[100, 100, 100]]
    assert info['valid_depth_fraction'] == 1
    assert project_visible(xyz, f).tolist() == [[5, 6]]


def test_clockwise_pixel_rotation_preserves_geometry():
    f = frame()
    m = np.zeros((8, 10), bool)
    m[6, 5] = True
    expected, _, _ = masked_cloud(f, m, erosion=0)
    h = f.depth_m.shape[0]
    f.rgb, f.depth_m = np.rot90(f.rgb, -1), np.rot90(f.depth_m, -1)
    f.K = np.array([[0, -1, h-1], [1, 0, 0], [0, 0, 1]]) @ f.K
    actual, _, _ = masked_cloud(f, np.rot90(m, -1), erosion=0)
    np.testing.assert_allclose(actual, expected)


def test_invalid_depth_excluded_and_occlusion_respected():
    f = frame()
    f.depth_m[0, :4] = [np.nan, 0, .1, 5.]
    xyz, _, info = masked_cloud(f, np.ones((8, 10), bool), erosion=0)
    assert len(xyz) == 76 and info['valid_points'] == 76
    f.depth_m[:] = .5
    assert len(project_visible(xyz, f)) == 0


def test_voxel_provenance_and_empty():
    xyz, rgb, bits = voxel_fuse([[.001, 0, 1], [.003, 0, 1]], [[0, 10, 20], [100, 20, 30]], [1, 4])
    np.testing.assert_allclose(xyz, [[.002, 0, 1]])
    assert rgb.tolist() == [[50, 15, 25]] and bits.tolist() == [5]
    assert voxel_fuse([], [], [])[0].shape == (0, 3)


def test_rigid_transform_and_finite_validation():
    f = frame()
    f.odom_T_camera[0, 0] = 2
    with pytest.raises(ValueError, match='rigid'):
        f.validate()
    f = frame()
    f.K[:] = 0
    with pytest.raises(ValueError, match='singular'):
        f.validate()


def test_serialization_and_manual_mask(tmp_path):
    f = frame()
    f.save(tmp_path/'frame.npz')
    restored = Frame.load(tmp_path/'frame.npz')
    np.testing.assert_array_equal(restored.depth_m, f.depth_m)
    mask, score = Segmenter().mask(f.rgb, {'polygon': [[0, 0], [9, 0], [9, 7], [0, 7]]})
    assert mask.all() and score == 1
    with pytest.raises(ValueError):
        Segmenter().mask(f.rgb, {'polygon': [[-1, 0], [9, 0], [9, 7]]})
    with pytest.raises(ValueError):
        projected_prompt(np.array([[1, 1]]), (8, 10))


class Camera:
    def __init__(self):
        self.fail = False
        self.t = 0

    def capture(self, _):
        if self.fail:
            raise ValueError('camera disconnected')
        self.t += 1
        return {'hand': frame(t=self.t)}


def test_capture_error_clears_cloud_and_masks(tmp_path):
    camera = Camera()
    p = Pipeline(camera, Segmenter(), tmp_path/'record')
    p.initial_capture()
    p.preview('hand', {'polygon': [[0, 0], [9, 0], [9, 7], [0, 7]]}, p.sequence)
    p.accept()
    assert len(p.cloud) > 0
    camera.fail = True
    with pytest.raises(ValueError, match='disconnected'):
        p.step()
    assert len(p.cloud) == 0 and not p.masks and not p.frames
    assert p.status == 'capture_or_segmentation_error'
    saved = np.load(tmp_path/'record'/f'revision-{p.revision:06d}'/'chair.npz')
    assert not len(saved['xyz_odom'])


def test_stale_selection_rejected(tmp_path):
    p = Pipeline(Camera(), Segmenter(), tmp_path/'record')
    p.initial_capture()
    with pytest.raises(ValueError, match='image changed'):
        p.preview('hand', {}, 0)


def test_camera_module_has_no_control_capability():
    path = Path(__file__).parents[1]/'src/spot_deploy/perception/cameras.py'
    tree = ast.parse(path.read_text())
    imports = [n.module or '' for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
    assert not any(any(x in m for x in ('lease', 'robot_command', 'power', 'estop')) for m in imports)
    calls = [n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert not set(calls) & {'power_on', 'power_off', 'robot_command', 'take', 'acquire', 'allow'}


def sdk_pair(view='frontleft'):
    from bosdyn.api import image_pb2
    from spot_deploy.perception.cameras import PAIRS
    pair = []
    for name, depth in zip(PAIRS[view], (False, True), strict=True):
        r = image_pb2.ImageResponse(status=image_pb2.ImageResponse.STATUS_OK)
        r.source.name = name
        r.source.depth_scale = 999 if depth else 0
        k = r.source.pinhole.intrinsics
        k.focal_length.x, k.focal_length.y = 2, 4
        k.principal_point.x, k.principal_point.y = 3, 2
        r.shot.frame_name_image_sensor = 'camera'
        r.shot.acquisition_time.seconds = 100
        r.shot.transforms_snapshot.child_to_parent_edge_map['odom'].parent_tform_child.rotation.w = 1
        edge = r.shot.transforms_snapshot.child_to_parent_edge_map['camera']
        edge.parent_frame_name = 'odom'
        edge.parent_tform_child.rotation.w = 1
        im = r.shot.image
        im.rows, im.cols = 8, 10
        im.format = image_pb2.Image.FORMAT_RAW
        im.pixel_format = (image_pb2.Image.PIXEL_FORMAT_DEPTH_U16 if depth else
                           image_pb2.Image.PIXEL_FORMAT_RGB_U8)
        im.data = (np.full((8, 10), 999, '<u2') if depth else
                   np.full((8, 10, 3), 100, np.uint8)).tobytes()
        pair.append(r)
    return pair


def test_sdk_depth_units_alignment_and_rotation():
    from spot_deploy.perception.cameras import pair_frame
    a, b = sdk_pair()
    f = pair_frame('frontleft', a, b)
    assert f.rgb.shape == (10, 8, 3)
    np.testing.assert_allclose(f.depth_m, 1)
    assert f.metadata['depth_scale'] == 999
    b.shot.acquisition_time.nanos = 100000000
    with pytest.raises(ValueError, match='timestamp skew'):
        pair_frame('frontleft', a, b)


def test_sdk_rejects_mismatched_intrinsics_and_exposure_motion():
    from spot_deploy.perception.cameras import pair_frame
    a, b = sdk_pair()
    b.source.pinhole.intrinsics.focal_length.x += 1
    with pytest.raises(ValueError, match='intrinsics differ'):
        pair_frame('frontleft', a, b)
    a, b = sdk_pair()
    a.shot.transforms_snapshot.child_to_parent_edge_map['camera'].parent_tform_child.position.x = .1
    with pytest.raises(ValueError, match='camera moved'):
        pair_frame('frontleft', a, b)


def test_age_is_measured_at_receipt_and_rejected_raw_is_preserved(tmp_path, monkeypatch):
    from types import SimpleNamespace
    import spot_deploy.perception.cameras as module
    responses = []
    for v in module.PAIRS:
        responses.extend(sdk_pair(v))
    clock = [100.1]

    def timestamp(_):
        return SimpleNamespace(seconds=int(clock[0]), nanos=int((clock[0] % 1)*1e9))

    c = module.SpotCameras.__new__(module.SpotCameras)
    c.timeout, c.last_timestamp = 1, -float('inf')
    c.client = SimpleNamespace(get_image=lambda *a, **kw: responses)
    c.robot = SimpleNamespace(time_sync=SimpleNamespace(endpoint=SimpleNamespace(
        robot_timestamp_from_local_secs=timestamp)))
    original = module.pair_frame

    def delayed_decode(*args):
        clock[0] += 1  # Simulate costly first-use decoding without actually sleeping.
        return original(*args)

    monkeypatch.setattr(module, 'pair_frame', delayed_decode)
    frames = c.capture(tmp_path/'valid')
    assert len(frames) == 3
    assert frames['hand'].metadata['receive_age_s'] == pytest.approx(.1)
    c.last_timestamp = -float('inf')
    clock[0] = 101
    with pytest.raises(ValueError, match='receipt age'):
        c.capture(tmp_path/'rejected')
    assert len(list((tmp_path/'rejected').glob('response-*.pb'))) == 6
    assert (tmp_path/'rejected'/'receipt.json').is_file()
