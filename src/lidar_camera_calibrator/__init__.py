"""Data-source-agnostic manual LiDAR-to-camera calibration package."""

from .adapters import foxglove_source_config, kitti_source_config
from .api import (
    CalibrationConfig, CalibrationResult, CameraEdgeCalibration, launch_calibrator,
)
from .kitti import KittiAdapter, load_kitti
from .models import CameraCalibration, CameraFrame, FrameBundle, LoadedCalibration, WorkingCalibration
from .overrides import (
    CalibrationOverride, CalibrationOverrideError, CameraEdgeOverride,
    SceneCalibrationOverride,
)
from .profile import (
    CameraInput, ConfigurationError, EgoPoseStreamSpec, FrameRef, FrameRole,
    PointCloudInput, ProfileError, ProfileValidationError, ProfileVersionError,
    ProfileWriteError, SceneMcapReader, SourceAdapterConfig, SourceDataError,
    SynchronizationConfig, SynchronizationError, TimedTransform,
    TransformGraphError, TransformSpec, write_profile_mcap,
)
from .projection import ProjectionResult, project_points, project_points_reference

__all__ = [
    "CalibrationOverride", "CalibrationOverrideError", "CameraEdgeOverride",
    "SceneCalibrationOverride", "CameraCalibration", "CameraFrame", "FrameBundle",
    "CalibrationConfig", "CalibrationResult", "CameraEdgeCalibration", "CameraInput",
    "ConfigurationError", "EgoPoseStreamSpec", "FrameRef", "FrameRole",
    "KittiAdapter", "LoadedCalibration", "PointCloudInput", "ProfileError",
    "ProfileValidationError", "ProfileVersionError", "ProfileWriteError", "SceneMcapReader",
    "ProjectionResult", "SourceAdapterConfig", "SourceDataError", "SynchronizationConfig",
    "SynchronizationError", "TimedTransform", "TransformGraphError", "TransformSpec",
    "WorkingCalibration", "foxglove_source_config", "kitti_source_config", "launch_calibrator", "load_kitti", "project_points",
    "project_points_reference", "write_profile_mcap",
]
