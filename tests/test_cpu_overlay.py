from types import SimpleNamespace

import numpy as np

from lidar_camera_calibrator.rendering import OverlaySettings, render_cpu_overlay


def _renderer_input():
    return SimpleNamespace(
        camera_id="image_02",
        projection_model={
            "transform": np.eye(4),
            "rectification_00": np.eye(3),
            "projection_matrix": np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]),
            "image_size": (8, 6),
        },
    )


def test_cpu_overlay_uses_canonical_projection_and_exact_clipping():
    points = np.array([
        [2.0, 3.0, 1.0, 0.5],  # pixel (2, 3)
        [9.0, 1.0, 1.0, 0.2],  # outside image
        [1.0, 1.0, -1.0, 0.8],  # behind camera
    ], dtype=np.float32)
    result = render_cpu_overlay(points, _renderer_input(), OverlaySettings(point_size_px=1.0))

    assert result.projected_count == 1
    assert result.rgba.shape == (6, 8, 4)
    assert result.rgba[3, 2, 3] == 255
    assert result.rgba[1, 7, 3] == 0


def test_cpu_overlay_applies_deterministic_display_budget_after_preparation():
    points = np.array([[float(index % 8), float(index % 6), 1.0, 0.5] for index in range(20)], dtype=np.float32)
    result = render_cpu_overlay(points, _renderer_input(), OverlaySettings(), max_points=5)

    assert result.input_count == 5
    assert result.projected_count == 5


def test_cpu_overlay_controls_change_rendered_rgba_pixels():
    points = np.array([[4.0, 3.0, 1.0, 0.2]], dtype=np.float32)
    small = render_cpu_overlay(points, _renderer_input(), OverlaySettings(point_size_px=1.0))
    large = render_cpu_overlay(points, _renderer_input(), OverlaySettings(point_size_px=4.0))
    intensity = render_cpu_overlay(points, _renderer_input(), OverlaySettings(coloring="intensity"))
    transparent = render_cpu_overlay(points, _renderer_input(), OverlaySettings(opacity=0.0))
    shallow_range = render_cpu_overlay(
        points, _renderer_input(), OverlaySettings(depth_min_metres=0.5, depth_max_metres=1.5)
    )

    assert np.count_nonzero(large.rgba[..., 3]) > np.count_nonzero(small.rgba[..., 3])
    assert not np.array_equal(small.rgba[..., :3], intensity.rgba[..., :3])
    assert not np.any(transparent.rgba[..., 3])
    assert not np.array_equal(small.rgba[..., :3], shallow_range.rgba[..., :3])
