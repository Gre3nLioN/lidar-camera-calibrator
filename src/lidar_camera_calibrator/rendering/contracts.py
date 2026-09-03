"""Qt-free renderer contracts shared by scene-graph and preparation paths.

Importing this module never imports PySide6 or any OpenGL implementation. Keep
these value objects independent of the selected GPU backend so renderer-free
startup and software image diagnostics remain safe.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np


class OverlayRenderError(RuntimeError):
    """Typed, user-displayable rendering/backend failure."""


class ProjectionStatus(str, Enum):
    IDLE = "idle"
    PREPARING = "preparing"
    READY = "ready"
    STALE = "stale"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True)
class OverlaySettings:
    coloring: str = "depth"
    point_size_px: float = 2.0
    opacity: float = 1.0
    depth_min_metres: float = 0.0
    depth_max_metres: float = 100.0
    only_points_in_image: bool = True
    render_density: str = "medium"

    def __post_init__(self) -> None:
        if self.coloring not in ("depth", "intensity"):
            raise ValueError("coloring must be depth or intensity")
        if self.point_size_px <= 0 or not np.isfinite(self.point_size_px):
            raise ValueError("point_size_px must be finite and positive")
        if not 0 <= self.opacity <= 1 or not np.isfinite(self.opacity):
            raise ValueError("opacity must be in [0, 1]")
        if self.depth_min_metres < 0 or self.depth_max_metres <= self.depth_min_metres:
            raise ValueError("invalid depth range")
        if self.render_density.lower() not in ("light", "medium", "full"):
            raise ValueError("render_density must be light, medium, or full")


@dataclass(frozen=True)
class ProjectionUniforms:
    """Uniform-only update payload for GPU point projection."""

    transform: np.ndarray
    rectification: np.ndarray
    projection_matrix: np.ndarray
    image_size: tuple[int, int]
    intrinsics: np.ndarray | None = None
    rectified_translation: np.ndarray | None = None
    settings: OverlaySettings = OverlaySettings()
    viewport_rect: tuple[float, float, float, float] | None = None
    framebuffer_size: tuple[int, int] | None = None

    def __post_init__(self) -> None:
        matrices = {}
        for name, shape in (("transform", (4, 4)), ("rectification", (3, 3)), ("projection_matrix", (3, 4))):
            value = np.asarray(getattr(self, name), dtype=np.float32)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f"{name} must be finite with shape {shape}")
            matrices[name] = value
        if len(self.image_size) != 2 or any(int(x) <= 0 for x in self.image_size):
            raise ValueError("image_size must be positive")
        k = np.asarray(matrices["projection_matrix"][:, :3] if self.intrinsics is None else self.intrinsics, dtype=np.float32)
        if k.shape != (3, 3) or not np.isfinite(k).all():
            raise ValueError("intrinsics must be finite with shape (3, 3)")
        p = matrices["projection_matrix"]
        t = np.linalg.solve(p[:, :3].astype(float), p[:, 3].astype(float)) if self.rectified_translation is None else np.asarray(self.rectified_translation, dtype=np.float32)
        if t.shape != (3,) or not np.isfinite(t).all():
            raise ValueError("rectified_translation must be finite with shape (3,)")
        matrices["projection_matrix"] = np.column_stack((k, k @ t))
        viewport = (0.0, 0.0, float(self.image_size[0]), float(self.image_size[1])) if self.viewport_rect is None else tuple(map(float, self.viewport_rect))
        framebuffer = self.image_size if self.framebuffer_size is None else tuple(map(int, self.framebuffer_size))
        if len(viewport) != 4 or viewport[2] <= 0 or viewport[3] <= 0 or len(framebuffer) != 2 or min(framebuffer) <= 0:
            raise ValueError("invalid viewport/framebuffer dimensions")
        object.__setattr__(self, "viewport_rect", viewport)
        object.__setattr__(self, "framebuffer_size", framebuffer)
        for name, value in (*matrices.items(), ("intrinsics", k), ("rectified_translation", t)):
            value = np.array(value, copy=True)
            value.setflags(write=False)
            object.__setattr__(self, name, value)
