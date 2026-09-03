"""Software-safe full-frame LiDAR scene rasterization for the workspace overview."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LidarSceneResult:
    rgb: np.ndarray
    point_count: int
    rendered_count: int
    ego_pixel: tuple[int, int]
    base_scale_ratio: float
    pick_indices: np.ndarray | None = None


def render_lidar_scene(
    points_xyzi: np.ndarray,
    *,
    width: int = 1280,
    height: int = 720,
    yaw_degrees: float = -18.0,
    pitch_degrees: float = 58.0,
    zoom: float = 1.0,
    pan_x: float = 0.0,
    pan_y: float = 0.0,
    interactive: bool = False,
    max_interactive_points: int = 80_000,
    pivot_xyz: np.ndarray | None = None,
    anchor_x: float = 0.5,
    anchor_y: float = 0.5,
    base_scale_ratio: float | None = None,
) -> LidarSceneResult:
    """Render a frame around a stable world pivot and optionally publish picking."""
    if width < 2 or height < 2:
        raise ValueError("scene dimensions must be at least 2x2")
    points = np.asarray(points_xyzi, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError("points must have shape (N, 3) or (N, 4)")
    finite = np.isfinite(points[:, :3]).all(axis=1)
    xyz_all = points[finite, :3]
    source_indices_all = np.flatnonzero(finite).astype(np.int32)
    background = np.empty((height, width, 3), dtype=np.uint8)
    background[:] = (14, 22, 30)
    if not len(xyz_all):
        background.setflags(write=False)
        return LidarSceneResult(background, 0, 0, (width // 2, height // 2), 1.0 / min(width, height))

    pivot = np.zeros(3, dtype=np.float32) if pivot_xyz is None else np.asarray(pivot_xyz, dtype=np.float32).reshape(3)
    total_count = len(xyz_all)
    xyz = xyz_all
    source_indices = source_indices_all
    if interactive:
        indices = np.arange(total_count)
        radial_distance = np.linalg.norm(xyz_all[:, :2], axis=1)
        shell_keep = (
            (radial_distance < 20.0)
            | ((radial_distance < 50.0) & (indices % 2 == 0))
            | ((radial_distance >= 50.0) & (indices % 4 == 0))
        )
        selected = indices[shell_keep]
        budget = max(10_000, int(max_interactive_points * min(1.0, max(0.5, zoom) ** 0.5)))
        if len(selected) > budget:
            selected = selected[::int(np.ceil(len(selected) / budget))]
        xyz = xyz_all[selected]
        source_indices = source_indices_all[selected]

    yaw = np.deg2rad(float(yaw_degrees))
    pitch = np.deg2rad(float(pitch_degrees))
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)

    def project_coordinates(values: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        relative = values - pivot
        forward = cy * relative[:, 0] - sy * relative[:, 1]
        lateral = sy * relative[:, 0] + cy * relative[:, 1]
        vertical = cp * relative[:, 2] + sp * forward
        depth = cp * forward - sp * relative[:, 2]
        return -lateral, vertical, depth

    horizontal, vertical, depth = project_coordinates(xyz)
    if base_scale_ratio is None:
        # This executes on the initial settled frame only. Subsequent orbit/LOD
        # renders reuse the ratio, eliminating apparent recenter/refit jumps.
        fit_horizontal, fit_vertical, _ = project_coordinates(xyz_all)
        span_u = max(float(np.ptp(fit_horizontal)), 1e-6)
        span_v = max(float(np.ptp(fit_vertical)), 1e-6)
        fitted_scale = min(width * 0.92 / span_u, height * 0.88 / span_v)
        base_scale_ratio = fitted_scale / min(width, height)
    base_scale_ratio = float(max(base_scale_ratio, 1e-9))
    scale = base_scale_ratio * min(width, height) * float(np.clip(zoom, 0.5, 20.0))
    center_x = width * (float(anchor_x) + float(pan_x))
    center_y = height * (float(anchor_y) + float(pan_y))
    px = np.rint(center_x + horizontal * scale).astype(np.intp)
    py = np.rint(center_y - vertical * scale).astype(np.intp)
    visible = (px >= 0) & (px < width) & (py >= 0) & (py < height)

    z = xyz[:, 2]
    t = np.clip((z + 2.5) / 5.0, 0.0, 1.0)
    colors = np.column_stack((
        35 + 205 * t,
        105 + 125 * (1.0 - np.abs(2.0 * t - 1.0)),
        235 - 170 * t,
    )).astype(np.uint8)
    order = np.arange(len(depth)) if interactive else np.argsort(depth)
    footprints = ((0, 0),) if interactive else ((0, 0), (1, 0), (0, 1), (1, 1))
    pick_indices = None if interactive else np.full((height, width), -1, dtype=np.int32)
    for dx, dy in footprints:
        xx, yy = px[order] + dx, py[order] + dy
        inside = visible[order] & (xx >= 0) & (xx < width) & (yy >= 0) & (yy < height)
        background[yy[inside], xx[inside]] = colors[order][inside]
        if pick_indices is not None:
            pick_indices[yy[inside], xx[inside]] = source_indices[order][inside]

    ego_horizontal, ego_vertical, _ = project_coordinates(np.zeros((1, 3), dtype=np.float32))
    ego_x = int(round(center_x + ego_horizontal[0] * scale))
    ego_y = int(round(center_y - ego_vertical[0] * scale))
    radius = max(4, int(round(min(width, height) * 0.009)))
    yy, xx = np.ogrid[:height, :width]
    ego = (xx - ego_x) ** 2 + (yy - ego_y) ** 2 <= radius ** 2
    background[ego] = (50, 255, 120)
    background.setflags(write=False)
    if pick_indices is not None:
        pick_indices.setflags(write=False)
    return LidarSceneResult(
        background, total_count, len(xyz), (ego_x, ego_y), base_scale_ratio, pick_indices
    )
