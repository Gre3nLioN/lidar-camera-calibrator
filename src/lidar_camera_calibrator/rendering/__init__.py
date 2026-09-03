"""Point preparation and GPU overlay rendering boundary.

The preparation helpers are Qt-free and deliberately consume the canonical
projection inputs produced by :mod:`lidar_camera_calibrator.projection`.
"""
from .point_preparation import (
    CandidateEnvelope,
    PointPreparationCache,
    PreparationKey,
    PreparedPointBuffer,
    ProjectionInputs,
    RenderProfile,
    RENDER_PROFILES,
    prepare_candidates,
    prepare_renderer_input,
    prepare_renderer_input_async,
    projection_inputs_from_mapping,
)
from .contracts import OverlayRenderError, OverlaySettings, ProjectionStatus, ProjectionUniforms
from .cpu_overlay import CpuOverlayResult, render_cpu_overlay
from .cpu_lidar_scene import LidarSceneResult, render_lidar_scene
from .scenegraph_overlay import SceneGraphOverlayItem, register_qml_type

__all__ = [
    "CandidateEnvelope", "PointPreparationCache", "PreparationKey", "PreparedPointBuffer", "ProjectionInputs", "RenderProfile",
    "RENDER_PROFILES", "prepare_candidates", "prepare_renderer_input", "prepare_renderer_input_async", "projection_inputs_from_mapping",
    "OverlayRenderError", "OverlaySettings", "CpuOverlayResult", "render_cpu_overlay", "LidarSceneResult", "render_lidar_scene",
    "SceneGraphOverlayItem", "ProjectionStatus", "ProjectionUniforms", "register_qml_type",
]
