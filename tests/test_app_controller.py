from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from lidar_camera_calibrator.app import CalibrationCommand, WorkspaceController
from lidar_camera_calibrator.models import CameraCalibration, CameraFrame, FrameBundle, LoadedCalibration
from lidar_camera_calibrator.transforms import make_transform


class FakeAdapter:
    def __init__(self) -> None:
        camera = CameraCalibration(
            "image_00", (100, 80), np.array([[10, 0, 50], [0, 10, 40], [0, 0, 1.]]),
            np.array([[10, 0, 50, 2], [0, 10, 40, 0], [0, 0, 1, 0.]]), np.eye(4), np.eye(3),
        )
        camera2 = CameraCalibration(
            "image_02", (100, 80), np.array([[10, 0, 50], [0, 10, 40], [0, 0, 1.]]),
            np.array([[10, 0, 50, 2], [0, 10, 40, 0], [0, 0, 1, 0.]]), np.eye(4), np.eye(3),
        )
        self.calibration = LoadedCalibration(make_transform(np.eye(3), [1, 2, 3]), {"image_00": camera, "image_02": camera2}, sequence="test")
        self._frames = [
            FrameBundle(i, float(i), np.array([[1, 2, 10, .5]], dtype=np.float32), {
                "image_00": CameraFrame("image_00", float(i), np.zeros((2, 2, 3), dtype=np.uint8), i),
                "image_02": CameraFrame("image_02", float(i), np.zeros((2, 2, 3), dtype=np.uint8), i),
            }) for i in range(2)
        ]

    def __len__(self):
        return len(self._frames)

    def frame(self, index):
        return self._frames[index]


def test_controller_is_qt_free_and_snapshot_copies_frame_data():
    controller = WorkspaceController(FakeAdapter())
    first = controller.snapshot()
    assert first.projection_status == "ready"
    assert first.renderer_input.points_xyzi.shape == (1, 4)
    assert not first.renderer_input.points_xyzi.flags.writeable
    assert not first.frame_bundle.lidar_points.flags.writeable
    with pytest.raises(ValueError):
        first.renderer_input.points_xyzi[0, 0] = 99
    # Snapshot arrays are not aliases of the adapter's frame.
    assert first.frame_bundle is not controller.adapter._frames[0]


def test_semantic_commands_dirty_reset_and_independent_focal_lengths():
    controller = WorkspaceController(FakeAdapter())
    assert not controller.snapshot().dirty
    result = controller.dispatch(CalibrationCommand.make("adjust_translation", axis="x", value=.25))
    assert result.dirty
    controller.dispatch("set_intrinsics_enabled", enabled=True)
    original_fy = controller.snapshot().working_calibration.intrinsics["image_02"][1, 1]
    controller.dispatch("set_intrinsic", name="fx", value=20)
    k = controller.snapshot().working_calibration.intrinsics["image_02"]
    assert k[0, 0] == 20 and k[1, 1] == original_fy
    controller.dispatch("set_intrinsic", name="fy", value=30)
    k = controller.snapshot().working_calibration.intrinsics["image_02"]
    assert k[0, 0] == 20 and k[1, 1] == 30
    renderer_model = controller.snapshot().renderer_input.projection_model
    assert renderer_model["projection_matrix"].shape == (3, 4)
    assert renderer_model["projection_matrix"][0, 3] != 0  # selected-camera baseline retained
    np.testing.assert_allclose(renderer_model["original_transform"], controller.loaded.t_camera_00_from_velodyne)
    assert not np.array_equal(renderer_model["original_transform"], renderer_model["transform"])
    np.testing.assert_allclose(renderer_model["original_intrinsics"], controller.loaded.cameras["image_02"].intrinsics)
    assert set(renderer_model["camera_models"]) == set(controller.loaded.cameras)
    with pytest.raises(TypeError):
        renderer_model["camera_models"]["image_02"]["image_size"] = (1, 1)
    np.testing.assert_allclose(renderer_model["rectified_translation"], [.2, 0, 0])
    assert renderer_model["baseline_rule"]
    controller.dispatch("reset_all")
    assert not controller.snapshot().dirty


def test_undo_redo_coalesces_drag_and_restores_reset_steps():
    controller = WorkspaceController(FakeAdapter())
    controller.dispatch("begin_calibration_change")
    controller.dispatch("adjust_translation", axis="x", value=0.1)
    controller.dispatch("adjust_translation", axis="x", value=0.2)
    controller.dispatch("end_calibration_change")
    assert controller.snapshot().can_undo and not controller.snapshot().can_redo
    controller.dispatch("undo")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == 0
    assert controller.snapshot().can_redo
    controller.dispatch("redo")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == 0.2
    controller.dispatch("reset_extrinsics")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == 0
    controller.dispatch("undo")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == 0.2


def test_reset_parameter_only_resets_the_selected_calibration_axis():
    controller = WorkspaceController(FakeAdapter())
    controller.dispatch("adjust_rotation", axis="pitch", value=4.0)
    controller.dispatch("adjust_rotation", axis="yaw", value=-3.0)

    controller.dispatch("reset_parameter", name="yaw")
    rotation = controller.snapshot().working_calibration.rotation_offset_degrees
    assert rotation[1] == 4.0
    assert rotation[2] == 0.0


def test_unavailable_camera_has_no_renderer_input():
    adapter = FakeAdapter()
    adapter._frames[1] = FrameBundle(1, 1.0, adapter._frames[1].lidar_points, {"image_00": None, "image_02": None})
    controller = WorkspaceController(adapter)
    snapshot = controller.dispatch("seek", frame_index=1)
    assert snapshot.projection_status == "unavailable"
    assert snapshot.renderer_input is None


def test_timeline_camera_selection_and_strict_export(tmp_path: Path):
    controller = WorkspaceController(FakeAdapter())
    with pytest.raises(ValueError, match="confirmation"):
        controller.dispatch("confirm_export", path=str(tmp_path / "direct.json"))
    controller.dispatch("seek", frame_index=1)
    assert controller.snapshot().timeline.frame_index == 1
    controller.dispatch("select_camera", camera_id="image_00")
    controller.dispatch("adjust_rotation", axis="yaw", value=2)
    path = tmp_path / "override.json"
    controller.dispatch("request_export", suggested_path=str(path))
    assert controller.snapshot().export.status == "confirming"
    controller.dispatch("confirm_export")
    payload = json.loads(path.read_text())
    assert set(payload) == {"T_camera_00_from_velodyne"}
    assert np.asarray(payload["T_camera_00_from_velodyne"]).shape == (4, 4)
    assert not controller.snapshot().dirty
