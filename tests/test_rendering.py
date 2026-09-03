from __future__ import annotations

import numpy as np

from lidar_camera_calibrator.rendering import (
    CandidateEnvelope,
    OverlaySettings,
    PointPreparationCache,
    ProjectionInputs,
    ProjectionStatus,
    ProjectionUniforms,
    RENDER_PROFILES,
    prepare_candidates,
    prepare_renderer_input,
    prepare_renderer_input_async,
    projection_inputs_from_mapping,
)


def projection() -> ProjectionInputs:
    return ProjectionInputs(
        "image_02", np.eye(4), np.eye(3),
        np.array([[10., 0., 50., 0.], [0., 10., 50., 0.], [0., 0., 1., 0.]]),
        (100, 100), intrinsics_revision=3, calibration_revision=4,
    )


def test_profiles_are_only_voxel_size() -> None:
    assert [RENDER_PROFILES[x].voxel_size_metres for x in ("light", "medium", "full")] == [.20, .08, .02]


def test_candidates_use_angular_margin_and_reject_invalid() -> None:
    # x=5.5 is outside the 26.565 degree exact right edge but within 5 degrees.
    points = np.array([[5.5, 0, 10, .7], [0, 0, -1, .2], [np.nan, 0, 2, .1]])
    result = prepare_candidates(points, projection(), profile="full")
    np.testing.assert_array_equal(result.source_indices, [0])
    np.testing.assert_allclose(result.points, [[5.5, 0, 10, .7]])
    assert result.candidate_count == result.retained_count == 1
    assert result.metadata["margin_degrees"] == 5.0


def test_xyz_input_becomes_raw_xyzi_and_voxel_choice_is_deterministic() -> None:
    points = np.array([[.01, 0, 10], [.09, 0, 10], [.11, 0, 10]])
    # 0.2m voxel: all points share a voxel; nearest centroid is the second point.
    result = prepare_candidates(points, projection(), profile="light")
    assert result.points.shape == (1, 4)
    np.testing.assert_array_equal(result.source_indices, [1])
    assert result.points[0, 3] == 0


def test_preparation_cache_reuses_and_bounds_buffers() -> None:
    cache = PointPreparationCache(max_entries=1, max_bytes=10_000)
    fake = type("Input", (), {
        "frame_index": 0, "camera_id": "image_02", "points_xyzi": np.array([[0., 0., 10., .5]]), "projection_model": {
            "transform": np.eye(4), "rectification_00": np.eye(3),
            "projection_matrix": projection().projection_matrix, "image_size": (100, 100),
        }, "overlay": type("Overlay", (), {"render_density": "full", "voxel_size_metres": .02})(),
        "calibration_revision": 1,
    })()
    first = prepare_renderer_input(fake, cache=cache)
    second = prepare_renderer_input(fake, cache=cache)
    assert first is second and cache.bytes_resident > 0
    assert prepare_renderer_input_async(fake, cache=cache).result().retained_count == 1


def test_dragging_keeps_one_degree_guard() -> None:
    env = CandidateEnvelope(margin_degrees=0.0, dragging_guard_degrees=1.0)
    result = prepare_candidates(np.array([[5.1, 0, 10, 1.]]), projection(), env, profile="full", dragging=True)
    assert result.candidate_count == 1


def test_working_intrinsics_rebuild_complete_projection_preserving_baseline() -> None:
    loaded = projection().projection_matrix.copy()
    loaded[:, 3] = [2., 0., 0.]
    edited_k = loaded[:, :3].copy(); edited_k[0, 0] = 20.0
    model = ProjectionInputs("image_02", np.eye(4), np.eye(3), loaded, (100, 100), intrinsics=edited_k)
    # t_rect=0.2 is retained while the working P uses edited K.
    np.testing.assert_allclose(model.rectified_translation, [.2, 0, 0])
    np.testing.assert_allclose(model.projection_matrix[:, :3], edited_k)
    np.testing.assert_allclose(model.projection_matrix[:, 3], [4., 0., 0.])


def test_controller_mapping_uses_canonical_rectification_and_working_baseline() -> None:
    loaded = projection().projection_matrix.copy(); loaded[:, 3] = [2., 0., 0.]
    mapping = {
        "transform": np.eye(4), "rectification_00": np.eye(3),
        "projection_matrix": loaded, "intrinsics": np.diag([20., 20., 1.]),
        "rectified_translation": np.array([.2, 0., 0.]), "image_size": (100, 100),
    }
    model = projection_inputs_from_mapping("image_02", mapping)
    np.testing.assert_allclose(model.projection_matrix[:, 3], [4., 0., 0.])


def test_uniform_payload_preserves_canonical_inputs() -> None:
    u = ProjectionUniforms(np.eye(4), np.eye(3), projection().projection_matrix, (100, 100))
    assert u.intrinsics.flags.writeable is False
    assert u.rectified_translation.flags.writeable is False
    assert OverlaySettings().coloring == "depth"
    assert ProjectionStatus.READY.value == "ready"
