from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("PySide6")
from lidar_camera_calibrator.ui.bridge import WorkspaceBridge


def _input(transform):
    return SimpleNamespace(
        frame_index=4,
        camera_id="image_02",
        calibration_revision=1,
        points_xyzi=np.array([[2.0, 3.0, 1.0, 0.5]], dtype=np.float32),
        projection_model={
            "transform": transform,
            "rectification_00": np.eye(3),
            "projection_matrix": np.array([[100.0, 0.0, 4.0, 0.0], [0.0, 100.0, 3.0, 0.0], [0.0, 0.0, 1.0, 0.0]]),
            "intrinsics": np.array([[100.0, 0.0, 4.0], [0.0, 100.0, 3.0], [0.0, 0.0, 1.0]]),
            "rectified_translation": np.zeros(3),
            "image_size": (8, 6),
        },
        overlay=SimpleNamespace(
            render_density="medium", voxel_size_metres=0.08, coloring="depth",
            point_size_px=1.0, opacity=1.0, depth_min_metres=0.0, depth_max_metres=100.0,
            only_points_in_image=True,
        ),
    )


def _snapshot(data, status="ready", message=None):
    return SimpleNamespace(renderer_input=data, projection_status=status,
                           projection_message=message, overlay=data.overlay)


def test_cpu_candidate_envelope_refreshes_after_large_rotation():
    bridge = WorkspaceBridge(cpu_overlay=True)
    bridge._cpu_prepared_transform = np.eye(4)
    bridge._cpu_prepared_intrinsics = _input(np.eye(4)).projection_model["intrinsics"]
    bridge._cpu_prepared_translation = np.zeros(3)
    small = np.eye(4)
    angle = np.deg2rad(6.0)
    large = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                      [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    assert bridge._cpu_out_of_envelope(_input(small)) is False
    assert bridge._cpu_out_of_envelope(_input(large)) is True
    bridge.shutdown()


def test_cpu_overlay_rekeys_populated_cache_and_preserves_controller_stale(monkeypatch):
    import lidar_camera_calibrator.rendering as rendering
    monkeypatch.setattr(rendering, "render_cpu_overlay", lambda *args, **kwargs: SimpleNamespace(
        rgba=np.zeros((6, 8, 4), dtype=np.uint8)))
    bridge = WorkspaceBridge(cpu_overlay=True)
    identity = _input(np.eye(4))
    bridge._push_cpu_overlay(_snapshot(identity))
    assert bridge._cpu_prepared_token == (4, "image_02", "medium", 0.08)
    assert {key.envelope_revision for key in bridge._prep_cache._items} == {0}

    angle = np.deg2rad(6.0)
    transform = np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                          [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    changed = _input(transform)
    bridge._push_cpu_overlay(_snapshot(changed))
    assert {key.envelope_revision for key in bridge._prep_cache._items} == {0, 1}
    assert bridge._cpu_envelope_revision == 1

    old_layer = bridge._cpu_overlay_image
    stale = _snapshot(changed, "stale", "controller refresh pending")
    bridge._push_cpu_overlay(stale)
    assert bridge._cpu_overlay_image is old_layer
    assert bridge._projection_status == "stale"
    assert bridge._projection_message == "controller refresh pending"
    bridge.shutdown()
