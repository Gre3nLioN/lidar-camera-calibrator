"""Strict source-adapter configuration for canonical scene MCAP generation."""
from __future__ import annotations

from dataclasses import dataclass, field
from difflib import get_close_matches
from enum import Enum
from io import BytesIO
import re
from typing import Any, Mapping, Sequence

import numpy as np

from .errors import ConfigurationError, SourceDataError

_CAMERA_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_POINT_FORMATS = frozenset({"xyzi-f32", "xyz-f32"})
_IMAGE_FORMATS = frozenset({"numpy-rgb", "png", "jpeg"})
_SYNC_METHODS = frozenset({"nearest", "previous", "next"})


class FrameRole(str, Enum):
    IMU = "imu"
    LIDAR = "lidar"
    CAMERA = "camera"


@dataclass(frozen=True, order=True)
class FrameRef:
    role: FrameRole
    camera_name: str | None = None

    def __post_init__(self) -> None:
        try:
            role = self.role if isinstance(self.role, FrameRole) else FrameRole(str(self.role))
        except ValueError as exc:
            raise ConfigurationError(
                "FRAME_ROLE_INVALID", "frame.role", "unknown semantic frame role",
                expected=sorted(role.value for role in FrameRole), actual=self.role,
            ) from exc
        object.__setattr__(self, "role", role)
        if role is FrameRole.CAMERA:
            _validate_camera_name(self.camera_name, "frame.camera_name")
        elif self.camera_name is not None:
            raise ConfigurationError(
                "FRAME_CAMERA_NAME_UNEXPECTED", "frame.camera_name",
                "camera_name is valid only for the CAMERA role", actual=self.camera_name,
            )

    @classmethod
    def imu(cls) -> "FrameRef":
        return cls(FrameRole.IMU)

    @classmethod
    def lidar(cls) -> "FrameRef":
        return cls(FrameRole.LIDAR)

    @classmethod
    def camera(cls, name: str) -> "FrameRef":
        return cls(FrameRole.CAMERA, name)

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any], path: str = "frame") -> "FrameRef":
        return _frame_ref(value, path)

    @property
    def label(self) -> str:
        return self.role.value if self.camera_name is None else f"camera:{self.camera_name}"


@dataclass(frozen=True)
class TransformSpec:
    target: FrameRef
    source: FrameRef
    matrix: np.ndarray

    def __post_init__(self) -> None:
        if not isinstance(self.target, FrameRef) or not isinstance(self.source, FrameRef):
            raise ConfigurationError(
                "TRANSFORM_FRAME_REF_INVALID", "static_transforms",
                "target and source must be FrameRef values",
                expected="FrameRef", actual={"target": type(self.target).__name__, "source": type(self.source).__name__},
            )
        object.__setattr__(self, "matrix", rigid_matrix(self.matrix, "static_transforms[].matrix"))
        if self.target == self.source:
            raise ConfigurationError(
                "TRANSFORM_SELF_EDGE", "static_transforms",
                "a transform cannot connect a semantic frame to itself", actual=self.target.label,
            )


@dataclass(frozen=True)
class TimedTransform:
    timestamp: float
    matrix: np.ndarray

    def __post_init__(self) -> None:
        timestamp = float(self.timestamp)
        if not np.isfinite(timestamp) or timestamp < 0:
            raise ConfigurationError(
                "TIMESTAMP_INVALID", "imu.samples[].timestamp",
                "timestamp must be finite and non-negative", actual=self.timestamp,
            )
        object.__setattr__(self, "timestamp", timestamp)
        object.__setattr__(self, "matrix", rigid_matrix(self.matrix, "imu.samples[].matrix"))


@dataclass(frozen=True)
class EgoPoseStreamSpec:
    samples: Sequence[TimedTransform]

    def __post_init__(self) -> None:
        if not _is_sequence_like(self.samples):
            raise ConfigurationError(
                "CONFIG_SEQUENCE_REQUIRED", "imu.samples", "IMU samples must be a sequence",
                actual=type(self.samples).__name__,
            )
        samples_list = []
        for index, item in enumerate(self.samples):
            if isinstance(item, TimedTransform):
                samples_list.append(item)
            elif isinstance(item, Mapping):
                try:
                    samples_list.append(TimedTransform(**item))
                except TypeError as exc:
                    raise ConfigurationError(
                        "CONFIG_TIMED_TRANSFORM_INVALID", f"imu.samples[{index}]",
                        "IMU sample requires only timestamp and matrix",
                        actual=sorted(item),
                    ) from exc
            else:
                raise ConfigurationError(
                    "CONFIG_TIMED_TRANSFORM_INVALID", f"imu.samples[{index}]",
                    "IMU sample must be TimedTransform or a mapping", actual=type(item).__name__,
                )
        samples = tuple(samples_list)
        if len(samples) < 2:
            raise ConfigurationError(
                "IMU_SAMPLES_INSUFFICIENT", "imu.samples",
                "at least two world-from-IMU samples are required", expected=">= 2", actual=len(samples),
            )
        _validate_increasing([item.timestamp for item in samples], "imu.samples")
        object.__setattr__(self, "samples", samples)


@dataclass(frozen=True)
class PointCloudInput:
    timestamps: Sequence[float]
    frames: Sequence[Any]
    format: str = "xyzi-f32"

    def __post_init__(self) -> None:
        timestamps = _timestamps(self.timestamps, "lidar.timestamps")
        if not _is_sequence_like(self.frames):
            raise ConfigurationError(
                "CONFIG_SEQUENCE_REQUIRED", "lidar.frames", "point-cloud frames must be a sequence",
                actual=type(self.frames).__name__,
            )
        if self.format not in _POINT_FORMATS:
            raise ConfigurationError(
                "POINT_FORMAT_INVALID", "lidar.format", "unsupported point-cloud format",
                expected=sorted(_POINT_FORMATS), actual=self.format,
            )
        if len(self.frames) != len(timestamps):
            raise ConfigurationError(
                "SOURCE_LENGTH_MISMATCH", "lidar.frames",
                "point-cloud frame count must equal timestamp count",
                expected=len(timestamps), actual=len(self.frames),
            )
        if not timestamps:
            raise ConfigurationError("LIDAR_EMPTY", "lidar.timestamps", "at least one LiDAR frame is required")
        object.__setattr__(self, "timestamps", timestamps)

    def validated_frame(self, index: int) -> np.ndarray:
        return validate_point_frame(self.frames[index], self.format, index)


@dataclass(frozen=True)
class CameraInput:
    name: str
    timestamps: Sequence[float]
    images: Sequence[Any]
    image_format: str
    intrinsics: np.ndarray
    image_size: tuple[int, int]

    def __post_init__(self) -> None:
        _validate_camera_name(self.name, "cameras[].name")
        timestamps = _timestamps(self.timestamps, f"cameras['{self.name}'].timestamps")
        if not _is_sequence_like(self.images):
            raise ConfigurationError(
                "CONFIG_SEQUENCE_REQUIRED", f"cameras['{self.name}'].images",
                "camera images must be a sequence", actual=type(self.images).__name__,
            )
        if self.image_format not in _IMAGE_FORMATS:
            raise ConfigurationError(
                "IMAGE_FORMAT_INVALID", f"cameras['{self.name}'].image_format",
                "unsupported camera image format", expected=sorted(_IMAGE_FORMATS), actual=self.image_format,
            )
        if len(self.images) != len(timestamps):
            raise ConfigurationError(
                "SOURCE_LENGTH_MISMATCH", f"cameras['{self.name}'].images",
                "image count must equal timestamp count", expected=len(timestamps), actual=len(self.images),
            )
        if not timestamps:
            raise ConfigurationError(
                "CAMERA_EMPTY", f"cameras['{self.name}'].timestamps",
                "each camera requires at least one image",
            )
        size = tuple(int(value) for value in self.image_size)
        if len(size) != 2 or size[0] <= 0 or size[1] <= 0:
            raise ConfigurationError(
                "IMAGE_SIZE_INVALID", f"cameras['{self.name}'].image_size",
                "image_size must be positive (width, height)", actual=self.image_size,
            )
        k = numeric_matrix(self.intrinsics, (3, 3), f"cameras['{self.name}'].intrinsics")
        if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1], atol=1e-9):
            raise ConfigurationError(
                "INTRINSICS_INVALID", f"cameras['{self.name}'].intrinsics",
                "intrinsics require positive fx/fy and homogeneous row [0, 0, 1]",
            )
        object.__setattr__(self, "timestamps", timestamps)
        object.__setattr__(self, "image_size", size)
        object.__setattr__(self, "intrinsics", k)

    def validated_image(self, index: int) -> Any:
        return validate_image_frame(self.images[index], self, index)


@dataclass(frozen=True)
class SynchronizationConfig:
    method: str = "nearest"
    max_delta_ms: float = 50.0

    def __post_init__(self) -> None:
        if self.method not in _SYNC_METHODS:
            raise ConfigurationError(
                "SYNC_METHOD_INVALID", "synchronization.method",
                "unsupported camera synchronization method",
                expected=sorted(_SYNC_METHODS), actual=self.method,
            )
        delta = float(self.max_delta_ms)
        if not np.isfinite(delta) or delta < 0:
            raise ConfigurationError(
                "SYNC_DELTA_INVALID", "synchronization.max_delta_ms",
                "maximum camera synchronization delta must be finite and non-negative",
                actual=self.max_delta_ms,
            )
        object.__setattr__(self, "max_delta_ms", delta)


@dataclass(frozen=True)
class SourceAdapterConfig:
    lidar: PointCloudInput
    cameras: Sequence[CameraInput]
    static_transforms: Sequence[TransformSpec]
    imu: EgoPoseStreamSpec
    synchronization: SynchronizationConfig = field(default_factory=SynchronizationConfig)

    def __post_init__(self) -> None:
        if not isinstance(self.lidar, PointCloudInput):
            raise ConfigurationError(
                "CONFIG_TYPE_INVALID", "lidar", "lidar must be PointCloudInput",
                expected="PointCloudInput", actual=type(self.lidar).__name__,
            )
        if not isinstance(self.imu, EgoPoseStreamSpec):
            raise ConfigurationError(
                "CONFIG_TYPE_INVALID", "imu", "imu must be EgoPoseStreamSpec",
                expected="EgoPoseStreamSpec", actual=type(self.imu).__name__,
            )
        if not isinstance(self.synchronization, SynchronizationConfig):
            raise ConfigurationError(
                "CONFIG_TYPE_INVALID", "synchronization", "synchronization must be SynchronizationConfig",
                expected="SynchronizationConfig", actual=type(self.synchronization).__name__,
            )
        if not _is_sequence_like(self.cameras):
            raise ConfigurationError("CONFIG_SEQUENCE_REQUIRED", "cameras", "cameras must be a sequence")
        if not _is_sequence_like(self.static_transforms):
            raise ConfigurationError("CONFIG_SEQUENCE_REQUIRED", "static_transforms", "static_transforms must be a sequence")
        cameras = tuple(self.cameras)
        transforms = tuple(self.static_transforms)
        for index, camera in enumerate(cameras):
            if not isinstance(camera, CameraInput):
                raise ConfigurationError(
                    "CONFIG_TYPE_INVALID", f"cameras[{index}]", "camera must be CameraInput",
                    expected="CameraInput", actual=type(camera).__name__,
                )
        for index, transform in enumerate(transforms):
            if not isinstance(transform, TransformSpec):
                raise ConfigurationError(
                    "CONFIG_TYPE_INVALID", f"static_transforms[{index}]", "transform must be TransformSpec",
                    expected="TransformSpec", actual=type(transform).__name__,
                )
        if not cameras:
            raise ConfigurationError("CAMERAS_EMPTY", "cameras", "at least one camera is required")
        names = [camera.name for camera in cameras]
        duplicates = sorted({name for name in names if names.count(name) > 1})
        if duplicates:
            indexes = {name: [i for i, value in enumerate(names) if value == name] for name in duplicates}
            raise ConfigurationError(
                "CAMERA_NAME_DUPLICATE", "cameras", "camera names must be unique", actual=indexes,
            )
        object.__setattr__(self, "cameras", cameras)
        object.__setattr__(self, "static_transforms", transforms)
        from .transforms import validate_transform_tree
        validate_transform_tree(transforms, names)

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SourceAdapterConfig":
        data = _strict_mapping(value, {"lidar", "cameras", "static_transforms", "imu", "synchronization"}, "config")
        _require(data, {"lidar", "cameras", "static_transforms", "imu"}, "config")
        lidar_data = _strict_mapping(data["lidar"], {"timestamps", "frames", "format"}, "lidar")
        _require(lidar_data, {"timestamps", "frames"}, "lidar")
        cameras = []
        for index, raw in enumerate(_sequence(data["cameras"], "cameras")):
            path = f"cameras[{index}]"
            camera_data = _strict_mapping(raw, {"name", "timestamps", "images", "image_format", "intrinsics", "image_size"}, path)
            _require(camera_data, {"name", "timestamps", "images", "image_format", "intrinsics", "image_size"}, path)
            cameras.append(CameraInput(**camera_data))
        transforms = []
        for index, raw in enumerate(_sequence(data["static_transforms"], "static_transforms")):
            path = f"static_transforms[{index}]"
            transform_data = _strict_mapping(raw, {"target", "source", "matrix"}, path)
            _require(transform_data, {"target", "source", "matrix"}, path)
            transforms.append(TransformSpec(
                target=_frame_ref(transform_data["target"], f"{path}.target"),
                source=_frame_ref(transform_data["source"], f"{path}.source"),
                matrix=transform_data["matrix"],
            ))
        imu_data = _strict_mapping(data["imu"], {"samples"}, "imu")
        _require(imu_data, {"samples"}, "imu")
        samples = []
        for index, raw in enumerate(_sequence(imu_data["samples"], "imu.samples")):
            path = f"imu.samples[{index}]"
            sample_data = _strict_mapping(raw, {"timestamp", "matrix"}, path)
            _require(sample_data, {"timestamp", "matrix"}, path)
            samples.append(TimedTransform(**sample_data))
        sync_data = _strict_mapping(data.get("synchronization", {}), {"method", "max_delta_ms"}, "synchronization")
        return cls(
            lidar=PointCloudInput(**lidar_data), cameras=cameras,
            static_transforms=transforms, imu=EgoPoseStreamSpec(samples),
            synchronization=SynchronizationConfig(**sync_data),
        )


def numeric_matrix(value: Any, shape: tuple[int, int], path: str) -> np.ndarray:
    try:
        matrix = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(
            "MATRIX_TYPE_INVALID", path, "matrix must contain numeric values", actual=type(value).__name__,
        ) from exc
    if matrix.shape != shape:
        raise ConfigurationError(
            "MATRIX_SHAPE_INVALID", path, "matrix has the wrong shape", expected=shape, actual=matrix.shape,
        )
    if not np.isfinite(matrix).all():
        raise ConfigurationError("MATRIX_NONFINITE", path, "matrix values must all be finite")
    matrix = np.array(matrix, copy=True)
    matrix.setflags(write=False)
    return matrix


def rigid_matrix(value: Any, path: str) -> np.ndarray:
    matrix = numeric_matrix(value, (4, 4), path)
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9):
        raise ConfigurationError(
            "RIGID_HOMOGENEOUS_ROW_INVALID", path,
            "rigid transform must end with [0, 0, 0, 1]", actual=matrix[3].tolist(),
        )
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ConfigurationError(
            "RIGID_ROTATION_INVALID", path,
            "rotation must be orthonormal with determinant +1",
            actual={"determinant": float(np.linalg.det(rotation))},
        )
    return matrix


def validate_point_frame(value: Any, point_format: str, index: int) -> np.ndarray:
    path = f"lidar.frames[{index}]"
    try:
        points = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise SourceDataError(
            "POINT_TYPE_INVALID", path, "point frame must be a numeric array or nested list",
            actual=type(value).__name__,
        ) from exc
    columns = 4 if point_format == "xyzi-f32" else 3
    if points.ndim != 2 or points.shape[1] != columns:
        raise SourceDataError(
            "POINT_SHAPE_INVALID", path, "point frame has the wrong shape",
            expected=f"N×{columns}", actual=points.shape,
        )
    if not len(points):
        raise SourceDataError("POINT_FRAME_EMPTY", path, "point frame must contain at least one point")
    if not np.isfinite(points).all():
        raise SourceDataError("POINT_NONFINITE", path, "point frame contains NaN or infinity")
    if columns == 3:
        points = np.column_stack((points, np.zeros(len(points), dtype=np.float32)))
    points = np.ascontiguousarray(points, dtype=np.float32)
    points.setflags(write=False)
    return points


def validate_image_frame(value: Any, camera: CameraInput, index: int) -> Any:
    path = f"cameras['{camera.name}'].images[{index}]"
    if camera.image_format == "numpy-rgb":
        image = np.asarray(value)
        expected = (camera.image_size[1], camera.image_size[0], 3)
        if image.shape != expected:
            raise SourceDataError(
                "IMAGE_SHAPE_INVALID", path, "RGB image shape does not match camera image_size",
                expected=expected, actual=image.shape,
            )
        if image.dtype != np.uint8:
            raise SourceDataError(
                "IMAGE_DTYPE_INVALID", path, "RGB images must use uint8", expected="uint8", actual=str(image.dtype),
            )
        result = np.array(image, copy=True)
        result.setflags(write=False)
        return result
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise SourceDataError(
            "IMAGE_BYTES_INVALID", path, "compressed image must be bytes-like", actual=type(value).__name__,
        )
    payload = bytes(value)
    expected_magic = b"\x89PNG\r\n\x1a\n" if camera.image_format == "png" else b"\xff\xd8"
    if not payload.startswith(expected_magic):
        raise SourceDataError(
            "IMAGE_ENCODING_INVALID", path, f"payload is not a valid {camera.image_format} header",
        )
    try:
        from PIL import Image
        with Image.open(BytesIO(payload)) as decoded:
            size = decoded.size
            detected = decoded.format.lower()
    except Exception as exc:
        raise SourceDataError("IMAGE_DECODE_FAILED", path, "compressed image could not be decoded") from exc
    if size != camera.image_size:
        raise SourceDataError(
            "IMAGE_SIZE_MISMATCH", path, "decoded image dimensions do not match camera image_size",
            expected=camera.image_size, actual=size,
        )
    if detected != camera.image_format:
        raise SourceDataError(
            "IMAGE_ENCODING_INVALID", path, "decoded image format differs from configured format",
            expected=camera.image_format, actual=detected,
        )
    return payload


def _timestamps(values: Sequence[float], path: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError("TIMESTAMP_TYPE_INVALID", path, "timestamps must be numeric") from exc
    if not all(np.isfinite(value) and value >= 0 for value in result):
        raise ConfigurationError("TIMESTAMP_INVALID", path, "timestamps must all be finite and non-negative")
    _validate_increasing(result, path)
    return result


def _validate_increasing(values: Sequence[float], path: str) -> None:
    for index in range(1, len(values)):
        if values[index] <= values[index - 1]:
            raise ConfigurationError(
                "TIMESTAMP_ORDER_INVALID", f"{path}[{index}]",
                "timestamps must be strictly increasing",
                expected=f"> {values[index - 1]}", actual=values[index],
            )


def _validate_camera_name(value: Any, path: str) -> None:
    if not isinstance(value, str) or not _CAMERA_NAME.fullmatch(value):
        raise ConfigurationError(
            "CAMERA_NAME_INVALID", path,
            "camera name must match ^[A-Za-z][A-Za-z0-9_-]*$", actual=value,
        )


def _strict_mapping(value: Any, allowed: set[str], path: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigurationError(
            "CONFIG_MAPPING_REQUIRED", path, "configuration value must be a mapping",
            actual=type(value).__name__,
        )
    result = dict(value)
    unknown = sorted(set(result) - allowed)
    if unknown:
        name = unknown[0]
        suggestion = get_close_matches(name, sorted(allowed), n=1, cutoff=0.6)
        raise ConfigurationError(
            "CONFIG_UNKNOWN_FIELD", f"{path}.{name}", f"unknown field {name!r}",
            expected=sorted(allowed), hint=f"Did you mean {suggestion[0]!r}?" if suggestion else None,
        )
    return result


def _require(value: Mapping[str, Any], required: set[str], path: str) -> None:
    missing = sorted(required - set(value))
    if missing:
        raise ConfigurationError(
            "CONFIG_REQUIRED_FIELD_MISSING", f"{path}.{missing[0]}",
            f"required field {missing[0]!r} is missing", expected=sorted(required),
        )


def _is_sequence_like(value: Any) -> bool:
    return (
        not isinstance(value, (str, bytes, bytearray))
        and hasattr(value, "__len__")
        and hasattr(value, "__getitem__")
    )


def _sequence(value: Any, path: str) -> Sequence[Any]:
    if not _is_sequence_like(value):
        raise ConfigurationError(
            "CONFIG_SEQUENCE_REQUIRED", path, "configuration value must be a sequence",
            actual=type(value).__name__,
        )
    return value


def _frame_ref(value: Any, path: str) -> FrameRef:
    data = _strict_mapping(value, {"role", "camera_name"}, path)
    _require(data, {"role"}, path)
    try:
        return FrameRef(FrameRole(str(data["role"])), data.get("camera_name"))
    except ValueError as exc:
        raise ConfigurationError(
            "FRAME_ROLE_INVALID", f"{path}.role", "unknown semantic frame role",
            expected=sorted(role.value for role in FrameRole), actual=data["role"],
        ) from exc
