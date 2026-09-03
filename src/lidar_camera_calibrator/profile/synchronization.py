"""Deterministic camera matching and IMU pose interpolation."""
from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .config import EgoPoseStreamSpec, SynchronizationConfig, TimedTransform
from .errors import SynchronizationError


@dataclass(frozen=True)
class CameraMatch:
    source_index: int
    source_timestamp: float
    delta_seconds: float


def synchronize_camera(
    lidar_timestamps: Sequence[float],
    camera_timestamps: Sequence[float],
    config: SynchronizationConfig,
    camera_name: str,
) -> tuple[CameraMatch, ...]:
    times = tuple(float(value) for value in camera_timestamps)
    matches = []
    for frame_index, timestamp in enumerate(lidar_timestamps):
        index = _camera_index(float(timestamp), times, config.method)
        if index is None:
            raise SynchronizationError(
                "CAMERA_SYNC_MISSING", f"cameras['{camera_name}'].frames[{frame_index}]",
                "no camera image satisfies the configured temporal direction",
                actual={"scene_timestamp": float(timestamp), "method": config.method},
            )
        delta = times[index] - float(timestamp)
        if abs(delta) * 1000 > config.max_delta_ms + 1e-9:
            raise SynchronizationError(
                "CAMERA_SYNC_MISSING", f"cameras['{camera_name}'].frames[{frame_index}]",
                "selected camera image exceeds the maximum synchronization delta",
                expected={"max_delta_ms": config.max_delta_ms},
                actual={
                    "scene_timestamp": float(timestamp), "source_timestamp": times[index],
                    "delta_ms": delta * 1000, "method": config.method,
                },
                hint="Increase synchronization.max_delta_ms only if the sensors are known to be synchronized.",
            )
        matches.append(CameraMatch(index, times[index], delta))
    return tuple(matches)


def interpolate_imu(
    lidar_timestamps: Sequence[float], imu: EgoPoseStreamSpec
) -> tuple[TimedTransform, ...]:
    source_times = tuple(sample.timestamp for sample in imu.samples)
    result = []
    for frame_index, value in enumerate(lidar_timestamps):
        timestamp = float(value)
        right = bisect_left(source_times, timestamp)
        if right < len(source_times) and source_times[right] == timestamp:
            matrix = imu.samples[right].matrix
        else:
            left = right - 1
            if left < 0 or right >= len(source_times):
                raise SynchronizationError(
                    "IMU_SYNC_MISSING", f"imu.scene_frames[{frame_index}]",
                    "LiDAR timestamp is not bracketed by IMU localization samples",
                    expected={"range": [source_times[0], source_times[-1]]}, actual=timestamp,
                    hint="Provide world-from-IMU samples covering the full LiDAR timeline.",
                )
            before, after = imu.samples[left], imu.samples[right]
            alpha = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
            matrix = _interpolate_matrix(before.matrix, after.matrix, alpha)
        result.append(TimedTransform(timestamp, matrix))
    return tuple(result)


def _camera_index(timestamp: float, times: tuple[float, ...], method: str) -> int | None:
    if not times:
        return None
    if method == "previous":
        index = bisect_right(times, timestamp) - 1
        return index if index >= 0 else None
    if method == "next":
        index = bisect_left(times, timestamp)
        return index if index < len(times) else None
    right = bisect_left(times, timestamp)
    candidates = []
    if right < len(times):
        candidates.append(right)
    if right > 0:
        candidates.append(right - 1)
    # Timestamp in the second tuple element makes equal-distance ties choose earlier.
    return min(candidates, key=lambda index: (abs(times[index] - timestamp), times[index]))


def _interpolate_matrix(before: np.ndarray, after: np.ndarray, alpha: float) -> np.ndarray:
    output = np.eye(4, dtype=np.float64)
    output[:3, 3] = (1.0 - alpha) * before[:3, 3] + alpha * after[:3, 3]
    q0 = _matrix_to_quaternion(before[:3, :3])
    q1 = _matrix_to_quaternion(after[:3, :3])
    output[:3, :3] = _quaternion_to_matrix(_slerp(q0, q1, alpha))
    output.setflags(write=False)
    return output


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        quaternion = np.array([
            (m[2, 1] - m[1, 2]) / scale,
            (m[0, 2] - m[2, 0]) / scale,
            (m[1, 0] - m[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(m)))
        if index == 0:
            scale = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            quaternion = np.array([0.25 * scale, (m[0, 1] + m[1, 0]) / scale, (m[0, 2] + m[2, 0]) / scale, (m[2, 1] - m[1, 2]) / scale])
        elif index == 1:
            scale = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            quaternion = np.array([(m[0, 1] + m[1, 0]) / scale, 0.25 * scale, (m[1, 2] + m[2, 1]) / scale, (m[0, 2] - m[2, 0]) / scale])
        else:
            scale = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            quaternion = np.array([(m[0, 2] + m[2, 0]) / scale, (m[1, 2] + m[2, 1]) / scale, 0.25 * scale, (m[1, 0] - m[0, 1]) / scale])
    return quaternion / np.linalg.norm(quaternion)


def _quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = q0 + alpha * (q1 - q0)
        return result / np.linalg.norm(result)
    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    return np.sin((1 - alpha) * theta) / sin_theta * q0 + np.sin(alpha * theta) / sin_theta * q1
