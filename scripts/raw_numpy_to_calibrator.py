#!/usr/bin/env python3
"""Load raw files into basic Python/NumPy lists, write scene MCAP, and launch.

This intentionally does not use ``kitti_source_config``. KITTI is only the
source of sample files; the integration boundary is the generic
``SourceAdapterConfig`` accepted by any application with arrays and timestamps.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from math import cos, log, pi, sin, tan
from pathlib import Path

import numpy as np
from PIL import Image

from lidar_camera_calibrator import (
    CalibrationConfig, CameraInput, EgoPoseStreamSpec, FrameRef, PointCloudInput,
    SourceAdapterConfig, TimedTransform, TransformSpec, launch_calibrator,
    write_profile_mcap,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
KITTI_ROOT = PROJECT_ROOT.parent / "kitti"
DEFAULT_CALIBRATION = KITTI_ROOT / "2011_09_26_calib" / "2011_09_26"
DEFAULT_SEQUENCE = (
    KITTI_ROOT / "2011_09_26_drive_0001_sync" / "2011_09_26"
    / "2011_09_26_drive_0001_sync"
)
EARTH_RADIUS_METRES = 6_378_137.0


def timestamps(path: Path, count: int) -> list[float]:
    values: list[float] = []
    for text in path.read_text(encoding="utf-8").splitlines()[:count]:
        values.append(datetime.fromisoformat(text).replace(tzinfo=timezone.utc).timestamp())
    return values


def calibration_values(path: Path) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line:
            continue
        key, text = line.split(":", 1)
        try:
            value = np.fromstring(text.strip(), sep=" ")
        except ValueError:  # textual calib_time line
            continue
        if value.size:
            result[key] = value
    return result


def rigid_transform(path: Path) -> np.ndarray:
    values = calibration_values(path)
    result = np.eye(4)
    result[:3, :3] = values["R"].reshape(3, 3)
    result[:3, 3] = values["T"]
    return result


def oxts_pose(packet: np.ndarray, scale: float) -> np.ndarray:
    latitude, longitude, altitude, roll, pitch, yaw = packet[:6]
    translation = np.array([
        scale * longitude * pi * EARTH_RADIUS_METRES / 180,
        scale * EARTH_RADIUS_METRES * log(tan((90 + latitude) * pi / 360)),
        altitude,
    ])
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = translation
    return result


def load_as_basic_lists(
    calibration_root: Path,
    sequence_root: Path,
    frame_count: int,
    camera_names: list[str],
) -> SourceAdapterConfig:
    """Materialize raw source data and construct the generic adapter contract."""
    if frame_count < 2:
        raise ValueError("frame_count must be at least 2 because IMU interpolation needs two samples")

    lidar_files = sorted((sequence_root / "velodyne_points" / "data").glob("*.bin"))[:frame_count]
    if len(lidar_files) != frame_count:
        raise ValueError(f"requested {frame_count} frames, found {len(lidar_files)}")
    lidar_timestamps: list[float] = timestamps(
        sequence_root / "velodyne_points" / "timestamps.txt", frame_count
    )
    # This is deliberately eager and basic: a list containing one NumPy N×4 array per frame.
    point_frames: list[np.ndarray] = [
        np.fromfile(path, dtype="<f4").reshape(-1, 4) for path in lidar_files
    ]

    camera_calibration = calibration_values(calibration_root / "calib_cam_to_cam.txt")
    camera00_from_lidar = rigid_transform(calibration_root / "calib_velo_to_cam.txt")
    lidar_from_imu = rigid_transform(calibration_root / "calib_imu_to_velo.txt")
    rectification00 = camera_calibration["R_rect_00"].reshape(3, 3)
    base_rotation = rectification00 @ camera00_from_lidar[:3, :3]
    base_translation = rectification00 @ camera00_from_lidar[:3, 3]

    camera_inputs: list[CameraInput] = []
    transforms: list[TransformSpec] = [
        TransformSpec(FrameRef.lidar(), FrameRef.imu(), lidar_from_imu)
    ]
    for camera_name in camera_names:
        suffix = camera_name.removeprefix("image_")
        image_files = sorted((sequence_root / camera_name / "data").glob("*.png"))[:frame_count]
        if len(image_files) != frame_count:
            raise ValueError(f"{camera_name}: requested {frame_count} images, found {len(image_files)}")
        camera_timestamps: list[float] = timestamps(
            sequence_root / camera_name / "timestamps.txt", frame_count
        )
        # Another ordinary list: each item is an H×W×3 uint8 NumPy array.
        image_frames: list[np.ndarray] = []
        for path in image_files:
            with Image.open(path) as image:
                image_frames.append(np.asarray(image.convert("RGB")).copy())

        projection = camera_calibration[f"P_rect_{suffix}"].reshape(3, 4)
        intrinsics = projection[:, :3]
        width, height = image_frames[0].shape[1], image_frames[0].shape[0]
        camera_inputs.append(CameraInput(
            name=camera_name,
            timestamps=camera_timestamps,
            images=image_frames,
            image_format="numpy-rgb",
            intrinsics=intrinsics,
            image_size=(width, height),
        ))
        camera_from_lidar = np.eye(4)
        camera_from_lidar[:3, :3] = base_rotation
        camera_from_lidar[:3, 3] = base_translation + np.linalg.solve(
            intrinsics, projection[:, 3]
        )
        transforms.append(TransformSpec(
            FrameRef.camera(camera_name), FrameRef.lidar(), camera_from_lidar
        ))

    oxts_files = sorted((sequence_root / "oxts" / "data").glob("*.txt"))[:frame_count]
    packets: list[np.ndarray] = [
        np.fromstring(path.read_text(encoding="utf-8"), sep=" ") for path in oxts_files
    ]
    if len(packets) != frame_count:
        raise ValueError(f"requested {frame_count} OXTS packets, found {len(packets)}")
    scale = cos(float(packets[0][0]) * pi / 180)
    global_poses: list[np.ndarray] = [oxts_pose(packet, scale) for packet in packets]
    origin_from_world = np.linalg.inv(global_poses[0])
    world_from_imu: list[np.ndarray] = [origin_from_world @ pose for pose in global_poses]

    # SourceAdapterConfig is the generic adapter boundary. It has no KITTI knowledge.
    return SourceAdapterConfig(
        lidar=PointCloudInput(lidar_timestamps, point_frames),
        cameras=tuple(camera_inputs),
        static_transforms=tuple(transforms),
        imu=EgoPoseStreamSpec(tuple(
            TimedTransform(timestamp, pose)
            for timestamp, pose in zip(lidar_timestamps, world_from_imu)
        )),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Direct raw-file → Python lists/NumPy arrays → canonical MCAP example"
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "output/raw-numpy-scene.mcap")
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--cameras", nargs="+", default=["image_02", "image_03"])
    parser.add_argument("--no-launch", action="store_true")
    args = parser.parse_args(argv)

    print("Loading raw files into plain Python lists of NumPy arrays...")
    source = load_as_basic_lists(
        args.calibration.resolve(), args.sequence.resolve(), args.frames, args.cameras
    )
    print(
        f"Loaded {len(source.lidar.frames)} point arrays and "
        f"{sum(len(camera.images) for camera in source.cameras)} image arrays"
    )
    scene = write_profile_mcap(source, args.output)
    print(f"Wrote canonical profile: {scene}")
    if not args.no_launch:
        launch_calibrator(scene, CalibrationConfig(renderer_mode="cpu"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
