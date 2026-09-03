"""Canonical scene profile configuration, validation, and synchronization."""

from .config import (
    CameraInput,
    EgoPoseStreamSpec,
    FrameRef,
    FrameRole,
    PointCloudInput,
    SourceAdapterConfig,
    SynchronizationConfig,
    TimedTransform,
    TransformSpec,
    validate_image_frame,
    validate_point_frame,
)
from .errors import (
    ConfigurationError,
    ProfileError,
    ProfileValidationError,
    ProfileVersionError,
    ProfileWriteError,
    SourceDataError,
    SynchronizationError,
    TransformGraphError,
)
from .reader import SceneFrameRecord, SceneMcapReader
from .schemas import PROFILE_NAME, PROFILE_VERSION, SCHEMA_NAMES, load_schema
from .synchronization import CameraMatch, interpolate_imu, synchronize_camera
from .transforms import calibration_edge_for_camera, validate_transform_tree
from .writer import write_profile_mcap

__all__ = [
    "CameraInput", "CameraMatch", "ConfigurationError", "EgoPoseStreamSpec",
    "FrameRef", "FrameRole", "PointCloudInput", "PROFILE_NAME", "PROFILE_VERSION",
    "ProfileError", "ProfileValidationError", "ProfileVersionError", "ProfileWriteError", "SCHEMA_NAMES",
    "SceneFrameRecord", "SceneMcapReader",
    "SourceAdapterConfig", "SourceDataError", "SynchronizationConfig",
    "SynchronizationError", "TimedTransform", "TransformGraphError", "TransformSpec",
    "calibration_edge_for_camera", "interpolate_imu", "load_schema",
    "synchronize_camera", "validate_image_frame", "validate_point_frame",
    "validate_transform_tree", "write_profile_mcap",
]
