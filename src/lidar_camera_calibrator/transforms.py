"""Direction-explicit homogeneous transform helpers."""
from __future__ import annotations

import numpy as np


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=float)
    out[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    out[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return out


def euler_xyz_degrees(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the extrinsic XYZ (roll/pitch/yaw) rotation, applied as Rz Ry Rx."""
    r, p, y = np.deg2rad([roll, pitch, yaw])
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    cz, sz = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rotation_matrix_to_euler_xyz_degrees(rotation: np.ndarray) -> np.ndarray:
    """Invert :func:`euler_xyz_degrees` for a proper 3×3 rotation matrix."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    pitch = np.arcsin(np.clip(-matrix[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-8:
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        # At gimbal lock choose roll=0 and preserve the represented rotation.
        roll = 0.0
        yaw = np.arctan2(-matrix[0, 1], matrix[1, 1])
    return np.rad2deg([roll, pitch, yaw])


def compose_working_transform(
    loaded: np.ndarray,
    translation_offset_metres: np.ndarray | None = None,
    rotation_offset_degrees: np.ndarray | None = None,
) -> np.ndarray:
    """Compose camera-frame offsets on the loaded Velodyne->camera transform.

    Offsets are represented by ``delta @ loaded``; translation is metres and
    rotation order is documented as Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    translation = np.zeros(3) if translation_offset_metres is None else translation_offset_metres
    rotation = np.zeros(3) if rotation_offset_degrees is None else rotation_offset_degrees
    delta = make_transform(euler_xyz_degrees(*np.asarray(rotation, dtype=float)), translation)
    return delta @ np.asarray(loaded, dtype=float)


def as_homogeneous(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError("points must have shape (N, 3) or (N, 4)")
    # KITTI point rows are XYZ+reflectance, not homogeneous coordinates.
    # Always synthesize w=1; callers needing arbitrary homogeneous rows can
    # multiply their 4x4 transform directly before calling this helper.
    return np.concatenate([points[:, :3].astype(float, copy=False), np.ones((len(points), 1))], axis=1)
