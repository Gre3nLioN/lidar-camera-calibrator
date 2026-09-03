from __future__ import annotations

import os
import subprocess
import sys
import time

import numpy as np

from lidar_camera_calibrator import (
    CameraInput, EgoPoseStreamSpec, FrameRef, PointCloudInput, SceneMcapReader,
    SourceAdapterConfig, TimedTransform, TransformSpec, write_profile_mcap,
)


def test_100_frame_profile_performance_and_clean_shutdown(tmp_path):
    count = 100
    timestamps = tuple(index * .1 for index in range(count))
    rng = np.random.default_rng(12)
    point_frame = rng.normal(size=(1_000, 4)).astype(np.float32)
    point_frame[:, 3] = np.abs(point_frame[:, 3])
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    identity = np.eye(4)
    camera_from_lidar = np.array([
        [0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1],
    ], dtype=float)
    config = SourceAdapterConfig(
        PointCloudInput(timestamps, (point_frame,) * count),
        (CameraInput(
            "front", timestamps, (image,) * count, "numpy-rgb",
            np.array([[50, 0, 32], [0, 50, 24], [0, 0, 1]], dtype=float), (64, 48),
        ),),
        (
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), identity),
            TransformSpec(FrameRef.camera("front"), FrameRef.lidar(), camera_from_lidar),
        ),
        EgoPoseStreamSpec(tuple(TimedTransform(timestamp, identity) for timestamp in timestamps)),
    )
    scene = tmp_path / "scene-100.mcap"
    started = time.perf_counter()
    write_profile_mcap(config, scene)
    reader = SceneMcapReader(scene)
    assert len(reader) == count
    np.testing.assert_array_equal(reader.frame(99).lidar_points, point_frame)
    # Generous regression ceiling: catches accidental quadratic/eager behavior, not machine speed.
    assert time.perf_counter() - started < 15

    code = f"""
from PySide6.QtCore import QTimer
from PySide6.QtGui import QGuiApplication
from lidar_camera_calibrator import CalibrationConfig, launch_calibrator
app = QGuiApplication([])
QTimer.singleShot(600, lambda: [window.close() for window in app.allWindows()])
result = launch_calibrator({str(scene)!r}, CalibrationConfig(renderer_mode='cpu', preload_count=2))
assert tuple(result.camera_edges) == ('front',)
print('CLEAN_SHUTDOWN')
"""
    environment = os.environ.copy()
    environment.update({
        "QT_QPA_PLATFORM": "offscreen", "QT_QUICK_BACKEND": "software",
        "LIDAR_CALIBRATOR_PREP_MODE": "sync",
    })
    completed = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True,
        env=environment, timeout=30,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "CLEAN_SHUTDOWN" in completed.stdout
    assert "ReferenceError" not in completed.stderr
    assert "Failed to get image" not in completed.stderr
