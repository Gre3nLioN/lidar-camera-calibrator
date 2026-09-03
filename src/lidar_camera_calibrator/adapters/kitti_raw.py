"""Strict, lazy adapter from a raw KITTI sync drive to SourceAdapterConfig."""
from __future__ import annotations

from datetime import datetime, timezone
from math import cos, log, pi, sin, tan
from pathlib import Path
import re
from typing import Any, Sequence

import numpy as np

from ..profile import (
    CameraInput, ConfigurationError, EgoPoseStreamSpec, FrameRef, PointCloudInput,
    SourceAdapterConfig, SourceDataError, SynchronizationConfig, TimedTransform,
    TransformSpec,
)

_EARTH_RADIUS_METRES = 6_378_137.0


class _PointFileSequence(Sequence[np.ndarray]):
    def __init__(self, files: Sequence[Path]) -> None:
        self._files = tuple(files)

    def __len__(self) -> int:
        return len(self._files)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return type(self)(self._files[index])
        path = self._files[index]
        try:
            values = np.fromfile(path, dtype="<f4")
        except OSError as exc:
            raise SourceDataError(
                "KITTI_POINT_READ_FAILED", f"lidar.frames[{index}]",
                "KITTI Velodyne frame could not be read", actual=str(path), hint=str(exc),
            ) from exc
        if values.size % 4:
            raise SourceDataError(
                "POINT_SHAPE_INVALID", f"lidar.frames[{index}]",
                "KITTI Velodyne byte count is not divisible by four float32 fields",
                expected="N×4 float32", actual={"float_count": int(values.size), "path": str(path)},
            )
        return values.reshape(-1, 4)


class _ByteFileSequence(Sequence[bytes]):
    def __init__(self, files: Sequence[Path]) -> None:
        self._files = tuple(files)

    def __len__(self) -> int:
        return len(self._files)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return type(self)(self._files[index])
        path = self._files[index]
        try:
            return path.read_bytes()
        except OSError as exc:
            raise SourceDataError(
                "KITTI_IMAGE_READ_FAILED", f"cameras.images[{index}]",
                "KITTI camera image could not be read", actual=str(path), hint=str(exc),
            ) from exc


def kitti_source_config(
    calibration_path: str | Path,
    sequence_path: str | Path,
    *,
    camera_names: Sequence[str] | None = None,
    frame_limit: int | None = None,
    max_delta_ms: float = 50.0,
) -> SourceAdapterConfig:
    """Build a lazy canonical source configuration from a KITTI raw sync drive."""
    calibration_root = Path(calibration_path).expanduser().resolve()
    sequence_root = Path(sequence_path).expanduser().resolve()
    if not calibration_root.is_dir():
        raise ConfigurationError(
            "KITTI_CALIBRATION_PATH_INVALID", "calibration_path",
            "KITTI calibration path must be a directory", actual=str(calibration_root),
        )
    if not sequence_root.is_dir():
        raise ConfigurationError(
            "KITTI_SEQUENCE_PATH_INVALID", "sequence_path",
            "KITTI sequence path must be a directory", actual=str(sequence_root),
        )
    if frame_limit is not None and (
        isinstance(frame_limit, bool) or int(frame_limit) != frame_limit or frame_limit <= 0
    ):
        raise ConfigurationError(
            "KITTI_FRAME_LIMIT_INVALID", "frame_limit",
            "frame_limit must be None or a positive integer", actual=frame_limit,
        )

    camera_calibration = _numeric_calibration(calibration_root / "calib_cam_to_cam.txt")
    lidar_from_imu = _rigid_calibration(calibration_root / "calib_imu_to_velo.txt")
    camera00_from_lidar = _rigid_calibration(calibration_root / "calib_velo_to_cam.txt")

    available = tuple(sorted(
        path.name for path in sequence_root.glob("image_[0-9][0-9]")
        if path.is_dir() and (path / "data").is_dir()
    ))
    selected = tuple(available if camera_names is None else camera_names)
    if not selected:
        raise ConfigurationError(
            "KITTI_CAMERAS_EMPTY", "camera_names", "no KITTI camera directories were selected",
        )
    unknown = [name for name in selected if name not in available]
    if unknown:
        raise ConfigurationError(
            "KITTI_CAMERA_NOT_FOUND", "camera_names",
            "selected KITTI camera directory does not exist",
            expected=available, actual=unknown,
        )
    if len(selected) != len(set(selected)):
        raise ConfigurationError(
            "CAMERA_NAME_DUPLICATE", "camera_names", "KITTI camera selection contains duplicates",
            actual=selected,
        )

    all_lidar_files = _numbered_files(sequence_root / "velodyne_points" / "data", ".bin")
    all_lidar_timestamps = _timestamps(sequence_root / "velodyne_points" / "timestamps.txt")
    _require_equal_count("lidar", all_lidar_files, all_lidar_timestamps)
    limit = len(all_lidar_files) if frame_limit is None else min(int(frame_limit), len(all_lidar_files))
    lidar_files = all_lidar_files[:limit]
    lidar_timestamps = all_lidar_timestamps[:limit]
    if not lidar_files:
        raise SourceDataError("LIDAR_EMPTY", "lidar.frames", "KITTI sequence has no Velodyne frames")

    rectification00 = _matrix(camera_calibration, "R_rect_00", 3, 3)
    base_rotation = rectification00 @ camera00_from_lidar[:3, :3]
    base_translation = rectification00 @ camera00_from_lidar[:3, 3]
    cameras = []
    camera_transforms = []
    for camera_name in selected:
        suffix = camera_name.removeprefix("image_")
        projection = _matrix(camera_calibration, f"P_rect_{suffix}", 3, 4)
        size_values = _required(camera_calibration, f"S_rect_{suffix}")
        if size_values.shape != (2,):
            raise SourceDataError(
                "KITTI_CALIBRATION_SHAPE_INVALID", f"calibration.S_rect_{suffix}",
                "rectified image size must contain width and height", expected=2,
                actual=int(size_values.size),
            )
        intrinsics = projection[:, :3]
        rectified_offset = np.linalg.solve(intrinsics, projection[:, 3])
        camera_from_lidar = np.eye(4)
        camera_from_lidar[:3, :3] = base_rotation
        camera_from_lidar[:3, 3] = base_translation + rectified_offset

        image_files = _numbered_files(sequence_root / camera_name / "data", ".png")
        image_timestamps = _timestamps(sequence_root / camera_name / "timestamps.txt")
        _require_equal_count(f"cameras['{camera_name}']", image_files, image_timestamps)
        cameras.append(CameraInput(
            name=camera_name,
            timestamps=image_timestamps,
            images=_ByteFileSequence(image_files),
            image_format="png",
            intrinsics=intrinsics,
            image_size=(int(size_values[0]), int(size_values[1])),
        ))
        camera_transforms.append(TransformSpec(
            FrameRef.camera(camera_name), FrameRef.lidar(), camera_from_lidar
        ))

    oxts_files = _numbered_files(sequence_root / "oxts" / "data", ".txt")
    pose_count = max(2, len(lidar_timestamps))
    if len(oxts_files) < pose_count or len(all_lidar_timestamps) < pose_count:
        raise SourceDataError(
            "KITTI_OXTS_COUNT_INVALID", "imu.samples",
            "KITTI OXTS packets must cover every selected LiDAR frame",
            expected=pose_count, actual=len(oxts_files),
        )
    # KITTI sync packet indexes represent the same synchronized scene frame.
    # Associate OXTS localization with the LiDAR timeline rather than treating
    # sensor acquisition timestamps as an inter-sensor clock offset.
    poses = _oxts_poses(oxts_files[:pose_count])
    imu = EgoPoseStreamSpec(tuple(
        TimedTransform(timestamp, pose)
        for timestamp, pose in zip(all_lidar_timestamps[:pose_count], poses)
    ))

    return SourceAdapterConfig(
        lidar=PointCloudInput(lidar_timestamps, _PointFileSequence(lidar_files)),
        cameras=tuple(cameras),
        static_transforms=(
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), lidar_from_imu),
            *camera_transforms,
        ),
        imu=imu,
        synchronization=SynchronizationConfig("nearest", max_delta_ms),
    )


def _numeric_calibration(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise SourceDataError(
            "KITTI_CALIBRATION_FILE_MISSING", "calibration",
            "required KITTI calibration file is missing", actual=str(path),
        )
    result = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise SourceDataError(
            "KITTI_CALIBRATION_READ_FAILED", "calibration",
            "KITTI calibration file could not be read", actual=str(path), hint=str(exc),
        ) from exc
    for line in lines:
        if ":" not in line:
            continue
        key, text = line.split(":", 1)
        try:
            values = np.fromstring(text.strip(), sep=" ")
        except ValueError:
            continue
        if values.size:
            result[key.strip()] = values
    return result


def _rigid_calibration(path: Path) -> np.ndarray:
    values = _numeric_calibration(path)
    rotation = _matrix(values, "R", 3, 3)
    translation = _required(values, "T")
    if translation.shape != (3,):
        raise SourceDataError(
            "KITTI_CALIBRATION_SHAPE_INVALID", f"calibration.{path.name}.T",
            "translation must contain three values", expected=3, actual=int(translation.size),
        )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def _required(values: dict[str, np.ndarray], key: str) -> np.ndarray:
    if key not in values:
        raise SourceDataError(
            "KITTI_CALIBRATION_FIELD_MISSING", f"calibration.{key}",
            "required KITTI calibration field is missing", actual=key,
        )
    return values[key]


def _matrix(values: dict[str, np.ndarray], key: str, rows: int, columns: int) -> np.ndarray:
    value = _required(values, key)
    if value.size != rows * columns:
        raise SourceDataError(
            "KITTI_CALIBRATION_SHAPE_INVALID", f"calibration.{key}",
            "KITTI calibration matrix has the wrong number of values",
            expected=rows * columns, actual=int(value.size),
        )
    return value.reshape(rows, columns)


def _timestamps(path: Path) -> tuple[float, ...]:
    if not path.is_file():
        raise SourceDataError(
            "KITTI_TIMESTAMPS_MISSING", "timestamps", "KITTI timestamps file is missing",
            actual=str(path),
        )
    result = []
    for index, line in enumerate(path.read_text(encoding="utf-8").splitlines()):
        text = line.strip()
        if not text:
            continue
        try:
            result.append(datetime.fromisoformat(text).replace(tzinfo=timezone.utc).timestamp())
        except ValueError as exc:
            raise SourceDataError(
                "KITTI_TIMESTAMP_INVALID", f"{path}.lines[{index}]",
                "KITTI timestamp is malformed", actual=text,
            ) from exc
    return tuple(result)


def _numbered_files(directory: Path, suffix: str) -> tuple[Path, ...]:
    if not directory.is_dir():
        raise SourceDataError(
            "KITTI_DATA_DIRECTORY_MISSING", "source", "KITTI data directory is missing",
            actual=str(directory),
        )
    files = tuple(sorted(
        (path for path in directory.iterdir() if path.is_file() and path.suffix.lower() == suffix),
        key=lambda path: (0, int(path.stem)) if re.fullmatch(r"\d+", path.stem) else (1, path.name),
    ))
    return files


def _require_equal_count(path: str, payloads: Sequence[Any], timestamps: Sequence[float]) -> None:
    if len(payloads) != len(timestamps):
        raise SourceDataError(
            "KITTI_SOURCE_COUNT_MISMATCH", path,
            "KITTI payload and timestamp counts differ",
            expected={"timestamps": len(timestamps)}, actual={"payloads": len(payloads)},
        )


def _oxts_poses(files: Sequence[Path]) -> tuple[np.ndarray, ...]:
    packets = []
    for index, path in enumerate(files):
        try:
            values = np.fromstring(path.read_text(encoding="utf-8"), sep=" ")
        except OSError as exc:
            raise SourceDataError(
                "KITTI_OXTS_READ_FAILED", f"imu.samples[{index}]",
                "KITTI OXTS packet could not be read", actual=str(path), hint=str(exc),
            ) from exc
        if values.size < 6 or not np.isfinite(values[:6]).all():
            raise SourceDataError(
                "KITTI_OXTS_INVALID", f"imu.samples[{index}]",
                "KITTI OXTS packet requires finite lat, lon, alt, roll, pitch, yaw",
                expected=6, actual=int(values.size),
            )
        packets.append(values[:6])
    if not packets:
        raise SourceDataError("KITTI_OXTS_EMPTY", "imu.samples", "KITTI OXTS stream is empty")

    scale = cos(float(packets[0][0]) * pi / 180.0)
    global_poses = []
    for latitude, longitude, altitude, roll, pitch, yaw in packets:
        translation = np.array([
            scale * longitude * pi * _EARTH_RADIUS_METRES / 180.0,
            scale * _EARTH_RADIUS_METRES * log(tan((90.0 + latitude) * pi / 360.0)),
            altitude,
        ])
        cr, sr = cos(roll), sin(roll)
        cp, sp = cos(pitch), sin(pitch)
        cy, sy = cos(yaw), sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        pose = np.eye(4)
        pose[:3, :3] = rz @ ry @ rx
        pose[:3, 3] = translation
        global_poses.append(pose)
    origin_from_world = np.linalg.inv(global_poses[0])
    return tuple(origin_from_world @ pose for pose in global_poses)
