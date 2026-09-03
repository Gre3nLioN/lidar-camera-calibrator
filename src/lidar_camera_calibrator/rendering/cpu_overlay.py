"""Bounded CPU reference rasterizer for the temporary software overlay mode."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .contracts import OverlaySettings
from .point_preparation import ProjectionInputs, projection_inputs_from_mapping


@dataclass(frozen=True)
class CpuOverlayResult:
    """Transparent image-space point layer and its exact accepted-point count."""

    rgba: np.ndarray
    projected_count: int
    input_count: int


def _project_xyzi(points: np.ndarray, projection: ProjectionInputs) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project raw Velodyne XYZI with the canonical T -> R_rect -> P chain."""
    xyzi = np.asarray(points, dtype=np.float32)
    if xyzi.ndim != 2 or xyzi.shape[1] != 4:
        raise ValueError("prepared points must have shape (N, 4) XYZI")
    xyz = xyzi[:, :3]
    finite = np.isfinite(xyzi).all(axis=1)
    homogeneous = np.column_stack((xyz, np.ones(len(xyz), dtype=np.float32)))
    cam00 = (projection.transform @ homogeneous.T).T
    rectification = np.eye(4, dtype=np.float64)
    rectification[:3, :3] = projection.rectification
    rectified = (rectification @ cam00.T).T
    pixels_h = (projection.projection_matrix @ rectified.T).T
    depth = pixels_h[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        pixels = pixels_h[:, :2] / depth[:, None]
    width, height = projection.image_size
    valid = (
        finite
        & np.isfinite(pixels_h).all(axis=1)
        & (depth > 0)
        & (pixels[:, 0] >= 0) & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    )
    return pixels[valid], depth[valid], xyzi[valid, 3]


def render_cpu_overlay(
    points_xyzi: np.ndarray,
    projection_model: object,
    settings: OverlaySettings,
    *,
    max_points: int = 150_000,
) -> CpuOverlayResult:
    """Rasterize a bounded prepared XYZI buffer into a transparent RGBA layer.

    This is deliberately a correctness/reference path, not a 5M-point renderer.
    The caller supplies the existing FOV/voxel-prepared buffer; a deterministic
    stride enforces the CPU display budget without changing calibration math.
    """
    if max_points < 1:
        raise ValueError("max_points must be positive")
    projection = projection_inputs_from_mapping(str(projection_model.camera_id), projection_model.projection_model)
    points = np.asarray(points_xyzi, dtype=np.float32)
    if len(points) > max_points:
        stride = int(np.ceil(len(points) / max_points))
        points = points[::stride]
    pixels, depths, intensities = _project_xyzi(points, projection)
    width, height = projection.image_size
    rgba = np.zeros((height, width, 4), dtype=np.uint8)
    if len(pixels) == 0:
        rgba.setflags(write=False)
        return CpuOverlayResult(rgba, 0, len(points))

    # Far-to-near writes leave the closest point visible where pixels overlap.
    order = np.argsort(depths)[::-1]
    x = np.rint(pixels[order, 0]).astype(np.intp)
    y = np.rint(pixels[order, 1]).astype(np.intp)
    depths = depths[order]
    intensities = intensities[order]
    t = np.clip(
        (depths - settings.depth_min_metres)
        / max(settings.depth_max_metres - settings.depth_min_metres, 1e-6),
        0.0,
        1.0,
    )
    value = 1.0 - t if settings.coloring == "depth" else np.clip(intensities, 0.0, 1.0)
    color = np.column_stack((value, 1.0 - value, np.full_like(value, 0.1)))
    alpha = int(round(255 * settings.opacity))
    # Rasterize the requested integer diameter exactly. The previous symmetric
    # radius mapping made several adjacent UI values look identical (for
    # example 2–4 px), hiding valid control changes from the operator.
    diameter = max(1, int(round(settings.point_size_px)))
    start = -(diameter // 2)
    stop = start + diameter
    for dx in range(start, stop):
        for dy in range(start, stop):
            xx, yy = x + dx, y + dy
            inside = (xx >= 0) & (xx < width) & (yy >= 0) & (yy < height)
            rgba[yy[inside], xx[inside], :3] = np.rint(255 * color[inside]).astype(np.uint8)
            rgba[yy[inside], xx[inside], 3] = alpha
    rgba.setflags(write=False)
    return CpuOverlayResult(rgba, len(pixels), len(points))
