"""Immutable geometry and frame data models."""
from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

import numpy as np


def _readonly(value: np.ndarray, *, shape: tuple[int, ...] | None = None) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if shape is not None and array.shape != shape:
        raise ValueError(f"expected shape {shape}, got {array.shape}")
    array = np.array(array, copy=True)
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class CameraCalibration:
    """Calibration for one KITTI rectified pinhole camera."""

    camera_id: str
    image_size: tuple[int, int]  # width, height
    intrinsics: np.ndarray
    projection_matrix: np.ndarray  # KITTI P_rect (3x4), including baseline
    camera_from_camera00: np.ndarray
    rectification: np.ndarray
    distortion: np.ndarray = field(default_factory=lambda: np.zeros(5))

    def __post_init__(self) -> None:
        object.__setattr__(self, "intrinsics", _readonly(self.intrinsics, shape=(3, 3)))
        object.__setattr__(self, "projection_matrix", _readonly(self.projection_matrix, shape=(3, 4)))
        object.__setattr__(self, "camera_from_camera00", _readonly(self.camera_from_camera00, shape=(4, 4)))
        object.__setattr__(self, "rectification", _readonly(self.rectification, shape=(3, 3)))
        object.__setattr__(self, "distortion", _readonly(self.distortion))
        if len(self.image_size) != 2 or any(int(v) <= 0 for v in self.image_size):
            raise ValueError("image_size must be positive (width, height)")

    @property
    def width(self) -> int:
        return int(self.image_size[0])

    @property
    def height(self) -> int:
        return int(self.image_size[1])


@dataclass(frozen=True)
class LoadedCalibration:
    """Dataset calibration; arrays are made read-only and never edited in place."""

    t_camera_00_from_velodyne: np.ndarray
    cameras: Mapping[str, CameraCalibration]
    dataset: str = "KITTI"
    sequence: str | None = None
    lidar_to_cameras: Mapping[str, np.ndarray] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "t_camera_00_from_velodyne", _readonly(self.t_camera_00_from_velodyne, shape=(4, 4)))
        cameras = dict(self.cameras)
        object.__setattr__(self, "cameras", MappingProxyType(cameras))
        transforms = {
            camera_id: _readonly(matrix, shape=(4, 4))
            for camera_id, matrix in self.lidar_to_cameras.items()
        }
        unknown = set(transforms) - set(cameras)
        if unknown:
            raise ValueError(f"lidar_to_cameras contains unknown cameras: {sorted(unknown)}")
        object.__setattr__(self, "lidar_to_cameras", MappingProxyType(transforms))

    @property
    def T_camera_00_from_velodyne(self) -> np.ndarray:
        return self.t_camera_00_from_velodyne


@dataclass
class WorkingCalibration:
    """Mutable offsets over an immutable :class:`LoadedCalibration`."""

    loaded: LoadedCalibration
    translation_offset_metres: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation_offset_degrees: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll,pitch,yaw
    intrinsic_overrides: dict[str, np.ndarray] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.translation_offset_metres = np.asarray(self.translation_offset_metres, dtype=float).reshape(3)
        self.rotation_offset_degrees = np.asarray(self.rotation_offset_degrees, dtype=float).reshape(3)
        self.intrinsic_overrides = {k: np.asarray(v, dtype=float).reshape(3, 3) for k, v in self.intrinsic_overrides.items()}

    @property
    def t_camera_00_from_velodyne(self) -> np.ndarray:
        from .transforms import compose_working_transform
        return compose_working_transform(
            self.loaded.t_camera_00_from_velodyne,
            self.translation_offset_metres,
            self.rotation_offset_degrees,
        )

    def intrinsics(self, camera_id: str) -> np.ndarray:
        return self.intrinsic_overrides.get(camera_id, self.loaded.cameras[camera_id].intrinsics)

    def reset_extrinsics(self) -> None:
        self.translation_offset_metres[:] = 0
        self.rotation_offset_degrees[:] = 0

    def reset_intrinsics(self) -> None:
        self.intrinsic_overrides.clear()

    def to_override(self, validation_camera: str = "image_02", dataset: Mapping[str, object] | None = None):
        from .overrides import CalibrationOverride
        return CalibrationOverride(self.t_camera_00_from_velodyne)


@dataclass(frozen=True)
class CameraFrame:
    camera_id: str
    timestamp: float
    image: np.ndarray
    frame_index: int

    def __post_init__(self) -> None:
        image = np.asarray(self.image)
        image.setflags(write=False)
        object.__setattr__(self, "image", image)


@dataclass(frozen=True)
class FrameBundle:
    """LiDAR frame and nearest synchronized images selected at one timeline timestamp."""

    frame_index: int
    timestamp: float
    lidar_points: np.ndarray
    cameras: Mapping[str, CameraFrame | None]

    def __post_init__(self) -> None:
        points = np.asarray(self.lidar_points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] not in (3, 4):
            raise ValueError("lidar_points must have shape (N, 3) or (N, 4)")
        points = np.array(points, copy=True)
        points.setflags(write=False)
        object.__setattr__(self, "lidar_points", points)
        object.__setattr__(self, "cameras", MappingProxyType(dict(self.cameras)))

    @property
    def camera_frames(self) -> Mapping[str, CameraFrame | None]:
        return self.cameras
