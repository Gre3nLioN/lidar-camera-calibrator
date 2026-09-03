#!/usr/bin/env python3
"""Minimal direct-NumPy canonical scene example."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from lidar_camera_calibrator import (
    CalibrationConfig, CameraInput, EgoPoseStreamSpec, FrameRef, PointCloudInput,
    SourceAdapterConfig, TimedTransform, TransformSpec, launch_calibrator,
    write_profile_mcap,
)


def source_config(frame_count: int = 10) -> SourceAdapterConfig:
    timestamps = tuple(index * 0.1 for index in range(frame_count))
    angles = np.linspace(-0.5, 0.5, 2_000, dtype=np.float32)
    points = tuple(np.column_stack((
        8 + np.cos(angles + index * .02),
        5 * np.sin(angles),
        np.linspace(-1, 2, angles.size),
        np.linspace(0, 1, angles.size),
    )).astype(np.float32) for index in range(frame_count))
    intrinsics = np.array([[500, 0, 320], [0, 500, 180], [0, 0, 1]], dtype=float)
    images = tuple(np.full((360, 640, 3), (20, 28, 38), np.uint8) for _ in timestamps)

    camera_left_from_lidar = np.array([
        [0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1],
    ], dtype=float)
    camera_right_from_lidar = camera_left_from_lidar.copy()
    camera_right_from_lidar[0, 3] = -.5
    identity = np.eye(4)
    poses = tuple(TimedTransform(timestamp, identity.copy()) for timestamp in timestamps)
    if len(poses) == 1:
        poses += (TimedTransform(timestamps[0] + .1, identity.copy()),)

    return SourceAdapterConfig(
        lidar=PointCloudInput(timestamps, points),
        cameras=(
            CameraInput("left", timestamps, images, "numpy-rgb", intrinsics, (640, 360)),
            CameraInput("right", timestamps, images, "numpy-rgb", intrinsics, (640, 360)),
        ),
        static_transforms=(
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), identity),
            TransformSpec(FrameRef.camera("left"), FrameRef.lidar(), camera_left_from_lidar),
            TransformSpec(FrameRef.camera("right"), FrameRef.lidar(), camera_right_from_lidar),
        ),
        imu=EgoPoseStreamSpec(poses),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("output/numpy-scene.mcap"))
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--launch", action="store_true")
    args = parser.parse_args()
    write_profile_mcap(source_config(args.frames), args.output)
    print(f"Wrote {args.output}")
    if args.launch:
        launch_calibrator(args.output, CalibrationConfig())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
