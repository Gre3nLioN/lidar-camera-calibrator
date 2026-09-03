import numpy as np

from lidar_camera_calibrator.rendering import render_lidar_scene


def test_lidar_scene_renders_every_finite_point_and_green_ego_marker():
    points = np.array([
        [0.0, 0.0, 0.0, 1.0],
        [10.0, -2.0, 0.2, 0.5],
        [20.0, 4.0, 1.5, 0.2],
        [np.nan, 0.0, 0.0, 0.0],
    ], dtype=np.float32)
    result = render_lidar_scene(points, width=320, height=180)

    assert result.rgb.shape == (180, 320, 3)
    assert result.point_count == 3
    assert result.rendered_count == 3
    assert result.pick_indices is not None
    assert np.any(result.pick_indices >= 0)
    assert not result.pick_indices.flags.writeable
    assert not result.rgb.flags.writeable
    assert np.any(np.all(result.rgb == (50, 255, 120), axis=2))


def test_lidar_scene_navigation_changes_raster_without_dropping_points():
    x = np.linspace(-5.0, 30.0, 200, dtype=np.float32)
    points = np.column_stack((x, np.sin(x), np.cos(x), np.ones_like(x)))
    initial = render_lidar_scene(points, width=320, height=180)
    moved = render_lidar_scene(points, width=320, height=180, yaw_degrees=35, pitch_degrees=40, zoom=2)

    assert initial.point_count == moved.point_count == len(points)
    assert not np.array_equal(initial.rgb, moved.rgb)


def test_interactive_lod_preserves_near_points_and_bounds_render_work():
    count = 240_000
    x = np.linspace(0.0, 120.0, count, dtype=np.float32)
    points = np.column_stack((x, np.sin(x), np.zeros_like(x), np.ones_like(x)))
    result = render_lidar_scene(points, width=640, height=360, interactive=True, max_interactive_points=40_000)

    assert result.point_count == count
    assert 0 < result.rendered_count <= 40_000
    assert result.pick_indices is None
    assert np.any(np.all(result.rgb == (50, 255, 120), axis=2))


def test_pivot_projects_to_cursor_anchor_with_stable_scale():
    points = np.array([[0, 0, 0, 1], [10, 2, 1, 1], [20, -2, 0, 1]], dtype=np.float32)
    pivot = points[1, :3]
    result = render_lidar_scene(
        points, width=400, height=200, pivot_xyz=pivot,
        anchor_x=0.25, anchor_y=0.4, base_scale_ratio=0.02,
    )

    anchor_x, anchor_y = int(400 * 0.25), int(200 * 0.4)
    window = result.pick_indices[anchor_y - 2:anchor_y + 3, anchor_x - 2:anchor_x + 3]
    assert 1 in window
    assert result.base_scale_ratio == 0.02
