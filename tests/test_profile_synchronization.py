from __future__ import annotations

import numpy as np
import pytest

from lidar_camera_calibrator.profile import (
    EgoPoseStreamSpec,
    SynchronizationConfig,
    SynchronizationError,
    TimedTransform,
    interpolate_imu,
    synchronize_camera,
)


def test_nearest_camera_matching_chooses_earlier_on_equal_distance():
    matches = synchronize_camera(
        (1.0,), (0.9, 1.1), SynchronizationConfig(max_delta_ms=101), "front"
    )
    assert matches[0].source_index == 0
    assert matches[0].source_timestamp == pytest.approx(0.9)
    assert matches[0].delta_seconds == pytest.approx(-0.1)


def test_previous_and_next_camera_matching_are_deterministic():
    previous = synchronize_camera((1.0,), (0.8, 1.2), SynchronizationConfig("previous", 500), "front")
    following = synchronize_camera((1.0,), (0.8, 1.2), SynchronizationConfig("next", 500), "front")
    assert previous[0].source_index == 0
    assert following[0].source_index == 1


def test_camera_match_outside_threshold_or_direction_fails_with_context():
    with pytest.raises(SynchronizationError) as threshold:
        synchronize_camera((1.0,), (0.8,), SynchronizationConfig(max_delta_ms=50), "front")
    assert threshold.value.code == "CAMERA_SYNC_MISSING"
    assert "front" in threshold.value.path
    assert threshold.value.actual["delta_ms"] == pytest.approx(-200)

    with pytest.raises(SynchronizationError, match="temporal direction"):
        synchronize_camera((1.0,), (1.1,), SynchronizationConfig("previous", 500), "front")


def test_imu_interpolation_uses_linear_translation_and_rotation_slerp():
    first = np.eye(4)
    second = np.eye(4)
    second[:3, :3] = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
    second[:3, 3] = [2, 4, 6]
    imu = EgoPoseStreamSpec((TimedTransform(0, first), TimedTransform(2, second)))

    interpolated = interpolate_imu((1.0,), imu)[0].matrix
    np.testing.assert_allclose(interpolated[:3, 3], [1, 2, 3])
    np.testing.assert_allclose(
        interpolated[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-7
    )
    np.testing.assert_allclose(interpolated[3], [0, 0, 0, 1])


def test_imu_exact_sample_is_preserved_and_extrapolation_is_rejected():
    first = np.eye(4)
    second = np.eye(4); second[1, 3] = 2
    imu = EgoPoseStreamSpec((TimedTransform(1, first), TimedTransform(2, second)))
    np.testing.assert_array_equal(interpolate_imu((1,), imu)[0].matrix, first)

    with pytest.raises(SynchronizationError) as error:
        interpolate_imu((0.5,), imu)
    assert error.value.code == "IMU_SYNC_MISSING"
    assert error.value.path == "imu.scene_frames[0]"
