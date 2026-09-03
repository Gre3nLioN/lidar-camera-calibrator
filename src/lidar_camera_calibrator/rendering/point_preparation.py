"""Conservative camera candidates and deterministic voxel representations.

This module never computes image pixels.  It prepares source XYZ[I] points for
an overlay which performs exact clipping/projection from the canonical KITTI
projection model in a shader (or in :func:`project_points` for a CPU reference).
"""
from __future__ import annotations

from collections import OrderedDict
from concurrent.futures import Executor, Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from threading import RLock
from types import MappingProxyType
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class RenderProfile:
    """A v1 density profile; profiles differ *only* in voxel size."""

    name: str
    voxel_size_metres: float

    def __post_init__(self) -> None:
        if self.voxel_size_metres <= 0 or not np.isfinite(self.voxel_size_metres):
            raise ValueError("voxel_size_metres must be finite and positive")


RENDER_PROFILES: Mapping[str, RenderProfile] = MappingProxyType({
    "light": RenderProfile("light", 0.20),
    "medium": RenderProfile("medium", 0.08),
    "full": RenderProfile("full", 0.02),
})


@dataclass(frozen=True)
class ProjectionInputs:
    """Canonical projection inputs, supplied by geometry/controller.

    ``transform`` is the working ``T_camera_00_from_velodyne``. ``rectification``
    is ``R_rect_00`` and ``projection_matrix`` is the complete selected-camera
    ``P_rect_i`` (including camera relationship/baseline). No alternate chain is
    composed here.
    """

    camera_id: str
    transform: np.ndarray
    rectification: np.ndarray
    projection_matrix: np.ndarray
    image_size: tuple[int, int]
    # Working K may differ from the loaded P_rect K during opt-in intrinsic
    # editing. The complete P_rect still carries the authoritative baseline.
    intrinsics: np.ndarray | None = None
    rectified_translation: np.ndarray | None = None
    intrinsics_revision: int = 0
    calibration_revision: int = 0

    def __post_init__(self) -> None:
        t = np.asarray(self.transform, dtype=float)
        r = np.asarray(self.rectification, dtype=float)
        p = np.asarray(self.projection_matrix, dtype=float)
        if t.shape != (4, 4) or r.shape != (3, 3) or p.shape != (3, 4):
            raise ValueError("invalid canonical projection input shapes")
        if len(self.image_size) != 2 or any(int(v) <= 0 for v in self.image_size):
            raise ValueError("image_size must be positive (width, height)")
        if not np.isfinite(t).all() or not np.isfinite(r).all() or not np.isfinite(p).all():
            raise ValueError("projection inputs must be finite")
        k = np.asarray(p[:, :3] if self.intrinsics is None else self.intrinsics, dtype=float)
        if k.shape != (3, 3) or not np.isfinite(k).all():
            raise ValueError("intrinsics must be finite with shape (3, 3)")
        baseline = (np.linalg.solve(p[:, :3], p[:, 3])
                    if self.rectified_translation is None else np.asarray(self.rectified_translation, dtype=float))
        if baseline.shape != (3,) or not np.isfinite(baseline).all():
            raise ValueError("rectified_translation must be finite with shape (3,)")
        # Rebuild complete working P_rect while preserving loaded baseline.
        working_p = np.column_stack((k, k @ baseline))
        for name, value in (("transform", t), ("rectification", r), ("projection_matrix", working_p),
                            ("intrinsics", k), ("rectified_translation", baseline)):
            value = np.array(value, copy=True)
            value.setflags(write=False)
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class CandidateEnvelope:
    """Angular envelope around current rectified pinhole FOV."""

    margin_degrees: float = 5.0
    revision: int = 0
    dragging_guard_degrees: float = 1.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.margin_degrees) or self.margin_degrees < 0:
            raise ValueError("margin_degrees must be finite and non-negative")
        if not np.isfinite(self.dragging_guard_degrees) or self.dragging_guard_degrees < 0:
            raise ValueError("dragging_guard_degrees must be finite and non-negative")

    def effective_margin(self, dragging: bool = False) -> float:
        return max(float(self.margin_degrees), float(self.dragging_guard_degrees)) if dragging else float(self.margin_degrees)


@dataclass(frozen=True)
class PreparationKey:
    """Stable CPU representation cache key."""

    frame_index: int
    camera_id: str
    envelope_revision: int
    voxel_size_metres: float
    calibration_revision: int = 0


class PointPreparationCache:
    """Bounded LRU cache for prepared CPU buffers.

    Cache ownership is independent of the GL VBO cache. A UI/controller may run
    ``prepare_renderer_input`` in a worker and publish the resulting buffer when
    ready; cache hits are safe to use immediately while scrubbing.
    """

    def __init__(self, max_entries: int = 8, max_bytes: int = 512 * 1024 * 1024) -> None:
        if max_entries <= 0 or max_bytes <= 0:
            raise ValueError("cache limits must be positive")
        self.max_entries, self.max_bytes = int(max_entries), int(max_bytes)
        self._items: OrderedDict[PreparationKey, PreparedPointBuffer] = OrderedDict()
        self._bytes = 0
        self._lock = RLock()

    @property
    def bytes_resident(self) -> int:
        with self._lock:
            return self._bytes

    def get(self, key: PreparationKey) -> PreparedPointBuffer | None:
        with self._lock:
            value = self._items.get(key)
            if value is not None:
                self._items.move_to_end(key)
            return value

    def put(self, key: PreparationKey, value: PreparedPointBuffer) -> None:
        size = int(value.points.nbytes + value.source_indices.nbytes)
        if size > self.max_bytes:
            return
        with self._lock:
            old = self._items.pop(key, None)
            if old is not None:
                self._bytes -= int(old.points.nbytes + old.source_indices.nbytes)
            self._items[key] = value
            self._bytes += size
            while len(self._items) > self.max_entries or self._bytes > self.max_bytes:
                _, removed = self._items.popitem(last=False)
                self._bytes -= int(removed.points.nbytes + removed.source_indices.nbytes)


@dataclass(frozen=True)
class PreparedPointBuffer:
    """Immutable candidate/voxel result suitable for upload to a VBO."""

    points: np.ndarray
    source_indices: np.ndarray
    candidate_count: int
    retained_count: int
    profile: RenderProfile
    envelope: CandidateEnvelope
    camera_id: str
    metadata: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        points = np.asarray(self.points, dtype=np.float32)
        indices = np.asarray(self.source_indices, dtype=np.int64)
        if points.ndim != 2 or points.shape[1] not in (3, 4):
            raise ValueError("points must have shape (N, 3) or (N, 4)")
        if len(points) != len(indices):
            raise ValueError("source_indices must match points")
        for name, value in (("points", points), ("source_indices", indices)):
            value = np.array(value, copy=True)
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))


def projection_inputs_from_mapping(camera_id: str, model: Mapping[str, object]) -> ProjectionInputs:
    """Adapt the controller's opaque ``projection_model`` mapping.

    The mapping is intentionally copied at this boundary; no camera-i transform
    is inferred or composed. ``rectification_00`` is the controller's canonical
    spelling (``rectification`` is accepted for direct callers).
    """
    try:
        rectification = model.get("rectification_00")
        if rectification is None:
            rectification = model["rectification"]
        return ProjectionInputs(
            camera_id,
            np.asarray(model["transform"]),
            np.asarray(rectification),
            np.asarray(model["projection_matrix"]),
            tuple(model["image_size"]),
            intrinsics=None if model.get("intrinsics") is None else np.asarray(model["intrinsics"]),
            rectified_translation=None if model.get("rectified_translation") is None else np.asarray(model["rectified_translation"]),
            calibration_revision=int(model.get("calibration_revision", 0)),
            intrinsics_revision=int(model.get("intrinsics_revision", 0)),
        )
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
        raise ValueError("incomplete canonical projection model") from exc


def prepare_renderer_input_async(renderer_input: object, envelope: CandidateEnvelope | None = None, *, dragging: bool = False, cache: PointPreparationCache | None = None, executor: Executor | None = None) -> Future[PreparedPointBuffer]:
    """Schedule preparation off the UI thread and return a publishable future.

    The caller owns the executor when supplied. With no executor a short-lived
    worker is used and shuts down after the job completes.
    """
    if executor is not None:
        return executor.submit(prepare_renderer_input, renderer_input, envelope, dragging=dragging, cache=cache)
    pool = ThreadPoolExecutor(max_workers=1)
    future = pool.submit(prepare_renderer_input, renderer_input, envelope, dragging=dragging, cache=cache)
    future.add_done_callback(lambda _: pool.shutdown(wait=False))
    return future


def prepare_renderer_input(renderer_input: object, envelope: CandidateEnvelope | None = None, *, dragging: bool = False, cache: PointPreparationCache | None = None) -> PreparedPointBuffer:
    """Prepare a controller ``RendererReadyInput`` without leaking projection math.

    This adapter intentionally reads only the documented public fields and
    delegates to :func:`prepare_candidates`; callers may run it in a worker.
    """
    inputs = projection_inputs_from_mapping(renderer_input.camera_id, renderer_input.projection_model)
    overlay = renderer_input.overlay
    envelope = envelope or CandidateEnvelope()
    voxel_size = float(getattr(overlay, "voxel_size_metres", 0.08))
    key = PreparationKey(int(renderer_input.frame_index), str(renderer_input.camera_id), envelope.revision, voxel_size, int(getattr(renderer_input, "calibration_revision", 0)))
    if cache is not None:
        cached = cache.get(key)
        if cached is not None:
            return cached
    result = prepare_candidates(
        renderer_input.points_xyzi,
        inputs,
        envelope,
        profile=str(getattr(overlay, "render_density", "medium")),
        voxel_size_metres=float(getattr(overlay, "voxel_size_metres", 0.08)),
        dragging=dragging,
    )
    if cache is not None:
        cache.put(key, result)
    return result


def _as_xyzi(points: np.ndarray) -> np.ndarray:
    p = np.asarray(points)
    if p.ndim != 2 or p.shape[1] not in (3, 4):
        raise ValueError("points must have shape (N, 3) or (N, 4)")
    # The renderer boundary is always XYZI. For XYZ-only synthetic input use a
    # zero reflectance attribute; the fourth value is never homogeneous w.
    if p.shape[1] == 3:
        p = np.column_stack((p, np.zeros(len(p), dtype=p.dtype)))
    return np.asarray(p, dtype=np.float32)


def _rectified_camera_points(points: np.ndarray, inputs: ProjectionInputs) -> np.ndarray:
    xyz = np.asarray(points[:, :3], dtype=float)
    finite = np.isfinite(xyz).all(axis=1)
    homogeneous = np.concatenate((xyz, np.ones((len(xyz), 1))), axis=1)
    cam00 = (inputs.transform @ homogeneous.T).T
    rect_h = np.eye(4, dtype=float)
    rect_h[:3, :3] = inputs.rectification
    rectified = (rect_h @ cam00.T).T[:, :3]
    # P_rect=[K | K*t] means projection is K @ (rectified + t).
    k = inputs.projection_matrix[:, :3]
    result = rectified + inputs.rectified_translation
    result[~finite] = np.nan
    return result


def _angular_mask(camera_points: np.ndarray, inputs: ProjectionInputs, margin_degrees: float) -> np.ndarray:
    width, height = map(float, inputs.image_size)
    k = inputs.intrinsics
    fx, fy, cx, cy = float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
    if fx <= 0 or fy <= 0 or not np.isfinite([fx, fy, cx, cy]).all():
        raise ValueError("projection intrinsics must have positive focal lengths")
    margin = np.deg2rad(margin_degrees)
    # Bounds are explicitly built from the current pinhole intrinsics/image size.
    left, right = np.arctan2(-cx, fx) - margin, np.arctan2(width - cx, fx) + margin
    top, bottom = np.arctan2(-cy, fy) - margin, np.arctan2(height - cy, fy) + margin
    x, y, z = camera_points.T
    with np.errstate(divide="ignore", invalid="ignore"):
        horizontal = np.arctan2(x, z)
        vertical = np.arctan2(y, z)
    return (
        np.isfinite(camera_points).all(axis=1)
        & (z > 0)
        & (horizontal >= left) & (horizontal <= right)
        & (vertical >= top) & (vertical <= bottom)
    )


def _voxel_downsample(points: np.ndarray, source_indices: np.ndarray, voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    if len(points) == 0:
        return points, source_indices
    xyz = np.asarray(points[:, :3], dtype=float)
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    # dict preserves first encounter order, making ties deterministic.
    groups: dict[tuple[int, int, int], list[int]] = {}
    for row, key in enumerate(map(tuple, keys)):
        groups.setdefault(key, []).append(row)
    selected: list[int] = []
    for rows in groups.values():
        indices = np.asarray(rows, dtype=np.int64)
        centroid = xyz[indices].mean(axis=0)
        distance = np.sum((xyz[indices] - centroid) ** 2, axis=1)
        # argmin is stable and therefore chooses the lowest source/input index on ties.
        selected.append(int(indices[int(np.argmin(distance))]))
    selected_array = np.asarray(selected, dtype=np.int64)
    return points[selected_array], source_indices[selected_array]


def prepare_candidates(
    points: np.ndarray,
    projection: ProjectionInputs,
    envelope: CandidateEnvelope | None = None,
    *,
    profile: str | RenderProfile = "medium",
    voxel_size_metres: float | None = None,
    dragging: bool = False,
) -> PreparedPointBuffer:
    """Filter by conservative angular FOV then deterministically voxelize.

    The returned points remain source XYZ[I] attributes and are not projected.
    """
    p = _as_xyzi(points)
    envelope = envelope or CandidateEnvelope()
    if isinstance(profile, str):
        try:
            profile = RENDER_PROFILES[profile.lower()]
        except KeyError as exc:
            raise ValueError(f"unknown render profile: {profile}") from exc
    size = float(profile.voxel_size_metres if voxel_size_metres is None else voxel_size_metres)
    if size <= 0 or not np.isfinite(size):
        raise ValueError("voxel size must be finite and positive")
    camera_points = _rectified_camera_points(p, projection)
    mask = _angular_mask(camera_points, projection, envelope.effective_margin(dragging))
    # Attributes must remain upload-safe as well; non-finite XYZ[I] rows are
    # never retained even when the non-finite value is reflectance.
    mask &= np.isfinite(p).all(axis=1)
    candidate = p[mask]
    source_indices = np.flatnonzero(mask).astype(np.int64)
    retained, retained_indices = _voxel_downsample(candidate, source_indices, size)
    metadata = {
        "camera_id": projection.camera_id,
        "margin_degrees": envelope.effective_margin(dragging),
        "envelope_revision": envelope.revision,
        "intrinsics_revision": projection.intrinsics_revision,
        "calibration_revision": projection.calibration_revision,
        "voxel_size_metres": size,
    }
    return PreparedPointBuffer(
        retained, retained_indices, int(len(candidate)), int(len(retained)),
        RenderProfile(profile.name, size), envelope, projection.camera_id, metadata,
    )
