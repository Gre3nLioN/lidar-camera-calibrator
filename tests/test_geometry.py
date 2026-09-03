from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from lidar_camera_calibrator.kitti import KittiAdapter
from lidar_camera_calibrator.models import CameraCalibration, LoadedCalibration
from lidar_camera_calibrator.overrides import CalibrationOverride
from lidar_camera_calibrator.projection import project_points
from lidar_camera_calibrator.transforms import as_homogeneous, compose_working_transform, make_transform


ROOT = Path(__file__).parents[2] / "kitti"
CALIB = ROOT / "2011_09_26_calib" / "2011_09_26"
SEQUENCE = ROOT / "2011_09_26_drive_0001_sync.zip"


def synthetic() -> LoadedCalibration:
    cam = CameraCalibration(
        "image_00", (100, 100), np.array([[10, 0, 50], [0, 10, 50], [0, 0, 1.]]),
        np.array([[10, 0, 50, 2], [0, 10, 50, 0], [0, 0, 1, 0.]]), np.eye(4), np.eye(3),
    )
    return LoadedCalibration(np.eye(4), {"image_00": cam})


def test_direction_depth_and_bounds():
    c = synthetic()
    points = np.array([[0, 0, 10], [5, 0, 10], [60, 0, 10], [0, 0, -1]])
    result = project_points(points, c, "image_00")
    np.testing.assert_array_equal(result.indices, [0, 1])
    np.testing.assert_allclose(result.pixels, [[50.2, 50], [55.2, 50]])


def test_full_kitti_projection_and_baselines():
    if not CALIB.exists() or not SEQUENCE.exists():
        pytest.skip("KITTI fixture unavailable")
    adapter = KittiAdapter(CALIB, SEQUENCE)
    frame = adapter.frame(0)
    points_h = np.c_[frame.lidar_points[:, :3], np.ones(len(frame.lidar_points))]
    t = adapter.calibration.t_camera_00_from_velodyne
    r00 = adapter.calibration.cameras["image_00"].rectification
    r00h = np.eye(4); r00h[:3, :3] = r00
    for cid in ("image_00", "image_01", "image_02", "image_03"):
        camera = adapter.calibration.cameras[cid]
        expected_h = (camera.projection_matrix @ r00h @ t @ points_h.T).T
        with np.errstate(divide="ignore", invalid="ignore"):
            expected = expected_h[:, :2] / expected_h[:, 2, None]
        valid = (expected_h[:, 2] > 0) & (expected[:, 0] >= 0) & (expected[:, 0] < camera.width) & (expected[:, 1] >= 0) & (expected[:, 1] < camera.height)
        actual = project_points(frame.lidar_points, adapter.calibration, cid)
        np.testing.assert_array_equal(actual.indices, np.flatnonzero(valid))
        np.testing.assert_allclose(actual.pixels, expected[valid], atol=1e-9)
    assert not np.allclose(adapter.calibration.cameras["image_02"].projection_matrix[:, 3], 0)


def test_intrinsic_edit_preserves_rectified_baseline_translation():
    c = synthetic()
    edited = np.array([[20., 0, 50], [0, 20., 50], [0, 0, 1.]])
    result = project_points(np.array([[0., 0., 10.]]), c, "image_00", intrinsics=edited)
    # Loaded P has t_rect=(0.2, 0, 0), therefore edited u is 50 + 20*0.2.
    np.testing.assert_allclose(result.pixels, [[50.4, 50.]])


def test_raw_xyzi_lifting_and_offset_euler_composition():
    np.testing.assert_array_equal(as_homogeneous(np.array([[1., 2., 3., .7]])), [[1., 2., 3., 1.]])
    loaded = make_transform(np.eye(3), [1., 2., 3.])
    working = compose_working_transform(loaded, [0.5, 0, 0], [0, 0, 90])
    np.testing.assert_allclose(working[:3, 3], [-1.5, 1., 3.])
    np.testing.assert_allclose(working[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-12)


def test_override_round_trip_exports_only_the_final_matrix():
    matrix = np.arange(16, dtype=float).reshape(4, 4)
    payload = json.loads(CalibrationOverride(matrix).to_json())
    assert payload == {"T_camera_00_from_velodyne": matrix.tolist()}
    np.testing.assert_array_equal(CalibrationOverride.from_dict(payload).matrix, matrix)


def test_override_rejects_nonminimal_save_files():
    with pytest.raises(ValueError, match="only"):
        CalibrationOverride.from_dict({"T_camera_00_from_velodyne": np.eye(4).tolist(), "intrinsics": {}})
