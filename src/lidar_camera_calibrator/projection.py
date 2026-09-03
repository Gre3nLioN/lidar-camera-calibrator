"""CPU/Numpy reference implementation of the KITTI projection chain."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .models import CameraCalibration, LoadedCalibration, WorkingCalibration
from .transforms import as_homogeneous, compose_working_transform


@dataclass(frozen=True)
class ProjectionResult:
    pixels: np.ndarray
    depths: np.ndarray
    indices: np.ndarray
    mask: np.ndarray


def _projection_matrix(camera: CameraCalibration, intrinsics: np.ndarray | None) -> np.ndarray:
    """Build ``[K_working | K_working @ t_rect]`` without dropping baseline."""
    loaded_k = camera.projection_matrix[:, :3]
    t_rect = np.linalg.solve(loaded_k, camera.projection_matrix[:, 3])
    k = loaded_k if intrinsics is None else np.asarray(intrinsics, dtype=float)
    return np.column_stack((k, k @ t_rect))


def project_points(
    points: np.ndarray,
    calibration: LoadedCalibration | WorkingCalibration,
    camera_id: str,
    *,
    transform: np.ndarray | None = None,
    intrinsics: np.ndarray | None = None,
    reject_outside: bool = True,
) -> ProjectionResult:
    """Project Velodyne points into a rectified camera.

    The returned ``mask`` is indexed against the input points; ``pixels``,
    ``depths`` and ``indices`` contain only points passing depth and (by
    default) image-bound checks.
    """
    camera = calibration.loaded.cameras[camera_id] if isinstance(calibration, WorkingCalibration) else calibration.cameras[camera_id]
    if transform is None:
        transform = (
            calibration.t_camera_00_from_velodyne
            if isinstance(calibration, LoadedCalibration)
            else calibration.t_camera_00_from_velodyne
        )
    if intrinsics is None and isinstance(calibration, WorkingCalibration):
        intrinsics = calibration.intrinsics(camera_id)

    xyz = np.asarray(points, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("points must have shape (N, >=3)")
    # KITTI bins are XYZ+reflectance; synthetic callers may provide XYZ+homogeneous-w.
    if xyz.shape[1] == 4 and np.allclose(xyz[:, 3], 1.0):
        points_h = xyz[:, :4]
    else:
        points_h = np.concatenate([xyz[:, :3], np.ones((len(xyz), 1))], axis=1)
    # Canonical raw-KITTI path: P_rect_i already includes camera-i baseline,
    # camera relationship, and rectification. R_rect_00 is applied exactly once.
    cam00 = (np.asarray(transform) @ points_h.T).T
    rect00 = calibration.loaded.cameras["image_00"].rectification if isinstance(calibration, WorkingCalibration) else calibration.cameras["image_00"].rectification
    rectified00 = (np.block([[rect00, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]) @ cam00.T).T
    projected_h = (_projection_matrix(camera, intrinsics) @ rectified00.T).T
    depths = projected_h[:, 2]
    valid = np.isfinite(projected_h).all(axis=1) & (depths > 0)
    pixels_all = np.full((len(xyz), 2), np.nan, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        pixels_all = projected_h[:, :2] / depths[:, None]
    if reject_outside:
        valid &= (pixels_all[:, 0] >= 0) & (pixels_all[:, 0] < camera.width)
        valid &= (pixels_all[:, 1] >= 0) & (pixels_all[:, 1] < camera.height)
    indices = np.flatnonzero(valid)
    return ProjectionResult(pixels_all[indices], depths[indices], indices, valid)


def project_points_reference(*args, **kwargs) -> ProjectionResult:
    """Alias kept explicit for renderer-vs-reference comparisons."""
    return project_points(*args, **kwargs)
