"""Immutable application snapshots and renderer input contracts (no Qt)."""
from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np

from ..models import CameraFrame, FrameBundle, LoadedCalibration


def _ro(value: Any, dtype=float) -> np.ndarray:
    array = np.array(value, dtype=dtype, copy=True)
    array.setflags(write=False)
    return array


def _freeze_projection_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _ro(value)
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze_projection_value(item) for key, item in value.items()})
    return value


def _copy_bundle(bundle: FrameBundle | None) -> FrameBundle | None:
    """Copy frame arrays at the controller/view boundary (no live adapter alias)."""
    if bundle is None:
        return None
    cameras: dict[str, CameraFrame | None] = {}
    for camera_id, frame in bundle.cameras.items():
        cameras[camera_id] = None if frame is None else CameraFrame(
            frame.camera_id, frame.timestamp, np.array(frame.image, copy=True), frame.frame_index
        )
    return FrameBundle(bundle.frame_index, bundle.timestamp, np.array(bundle.lidar_points, copy=True), cameras)


@dataclass(frozen=True)
class TimelineSnapshot:
    frame_index: int
    frame_count: int
    timestamp: float
    is_playing: bool = False
    playback_rate: float = 1.0


@dataclass(frozen=True)
class WorkingCalibrationSnapshot:
    translation_offset_metres: np.ndarray
    rotation_offset_degrees: np.ndarray
    intrinsics: Mapping[str, np.ndarray]
    intrinsics_enabled: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "translation_offset_metres", _ro(self.translation_offset_metres))
        object.__setattr__(self, "rotation_offset_degrees", _ro(self.rotation_offset_degrees))
        copied = {key: _ro(value) for key, value in self.intrinsics.items()}
        object.__setattr__(self, "intrinsics", MappingProxyType(copied))


@dataclass(frozen=True)
class OverlaySettings:
    coloring: str = "depth"
    point_size_px: float = 2.0
    opacity: float = 1.0
    depth_min_metres: float = 0.0
    depth_max_metres: float = 100.0
    render_density: str = "medium"
    voxel_size_metres: float = 0.08
    only_points_in_image: bool = False


@dataclass(frozen=True)
class RendererReadyInput:
    """Raw candidate input for a renderer; projection remains renderer-owned."""

    frame_index: int
    camera_id: str
    points_xyzi: np.ndarray
    projection_model: Mapping[str, Any]
    overlay: OverlaySettings
    calibration_revision: int

    def __post_init__(self) -> None:
        points = _ro(self.points_xyzi, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] not in (3, 4):
            raise ValueError("points_xyzi must have shape (N, 3) or (N, 4)")
        object.__setattr__(self, "points_xyzi", points)
        # Keep matrices immutable while leaving the model intentionally opaque to UI.
        model = {key: _freeze_projection_value(value) for key, value in self.projection_model.items()}
        object.__setattr__(self, "projection_model", MappingProxyType(model))


@dataclass(frozen=True)
class ExportSnapshot:
    status: str = "idle"
    suggested_path: str = "calibration_override.json"
    error_message: str | None = None


@dataclass(frozen=True)
class WorkspaceSnapshot:
    dataset_label: str
    sequence_id: str | None
    selected_camera: str
    timeline: TimelineSnapshot
    frame_bundle: FrameBundle | None
    displayed_bundle: FrameBundle | None
    loaded_calibration: LoadedCalibration
    working_calibration: WorkingCalibrationSnapshot
    dirty: bool
    projection_status: str
    projection_message: str | None
    renderer_input: RendererReadyInput | None
    overlay: OverlaySettings
    can_undo: bool = False
    can_redo: bool = False
    export: ExportSnapshot = field(default_factory=ExportSnapshot)

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_bundle", _copy_bundle(self.frame_bundle))
        object.__setattr__(self, "displayed_bundle", _copy_bundle(self.displayed_bundle))
