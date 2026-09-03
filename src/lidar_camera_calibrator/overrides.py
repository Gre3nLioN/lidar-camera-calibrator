"""Portable JSON save file for the final LiDAR-to-camera transform."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np


_MATRIX_KEY = "T_camera_00_from_velodyne"


class CalibrationOverrideError(ValueError):
    """A saved calibration override is malformed or incompatible."""


@dataclass(frozen=True)
class CalibrationOverride:
    """The final 4×4 transform that replaces KITTI's Velodyne-to-camera edge."""

    matrix: np.ndarray

    def __post_init__(self) -> None:
        matrix = np.array(self.matrix, dtype=float, copy=True)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("matrix must be a finite 4x4 array")
        matrix.setflags(write=False)
        object.__setattr__(self, "matrix", matrix)

    def to_dict(self) -> dict[str, list[list[float]]]:
        return {_MATRIX_KEY: self.matrix.tolist()}

    def to_json(self, *, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    def save(self, path: str | Path, *, indent: int = 2) -> None:
        Path(path).write_text(self.to_json(indent=indent) + "\n", encoding="utf-8")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CalibrationOverride":
        if set(data) != {_MATRIX_KEY}:
            raise ValueError(f"save file must contain only {_MATRIX_KEY}")
        return cls(np.asarray(data[_MATRIX_KEY], dtype=float))

    @classmethod
    def load(cls, path: str | Path) -> "CalibrationOverride":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


@dataclass(frozen=True)
class CameraEdgeOverride:
    target: Mapping[str, str]
    source: Mapping[str, str]
    matrix: np.ndarray

    def __post_init__(self) -> None:
        matrix = np.array(self.matrix, dtype=float, copy=True)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("camera edge matrix must be a finite 4x4 array")
        matrix.setflags(write=False)
        object.__setattr__(self, "matrix", matrix)
        object.__setattr__(self, "target", MappingProxyType(dict(self.target)))
        object.__setattr__(self, "source", MappingProxyType(dict(self.source)))

    def to_dict(self) -> dict[str, object]:
        return {
            "target": dict(self.target),
            "source": dict(self.source),
            "matrix": self.matrix.tolist(),
        }


@dataclass(frozen=True)
class SceneCalibrationOverride:
    camera_edges: Mapping[str, CameraEdgeOverride]
    intrinsics: Mapping[str, np.ndarray]

    def __post_init__(self) -> None:
        edges = MappingProxyType(dict(self.camera_edges))
        intrinsics = {}
        for camera_name, matrix in self.intrinsics.items():
            value = np.array(matrix, dtype=float, copy=True)
            if value.shape != (3, 3) or not np.isfinite(value).all():
                raise ValueError("camera intrinsics must be finite 3x3 arrays")
            value.setflags(write=False)
            intrinsics[camera_name] = value
        object.__setattr__(self, "camera_edges", edges)
        object.__setattr__(self, "intrinsics", MappingProxyType(intrinsics))

    def to_dict(self) -> dict[str, object]:
        return {
            "profile_name": "lidar-camera-scene",
            "profile_version": 1,
            "camera_edges": {
                name: edge.to_dict() for name, edge in self.camera_edges.items()
            },
            "intrinsics": {
                name: matrix.tolist() for name, matrix in self.intrinsics.items()
            },
        }

    def to_json(self, *, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    def save(self, path: str | Path, *, indent: int = 2) -> None:
        Path(path).write_text(self.to_json(indent=indent) + "\n", encoding="utf-8")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SceneCalibrationOverride":
        expected = {"profile_name", "profile_version", "camera_edges", "intrinsics"}
        if not isinstance(data, Mapping) or set(data) != expected:
            raise CalibrationOverrideError(
                f"scene override must contain exactly {sorted(expected)!r}"
            )
        if data["profile_name"] != "lidar-camera-scene" or data["profile_version"] != 1:
            raise CalibrationOverrideError(
                "scene override must have profile identity lidar-camera-scene/1"
            )
        raw_edges, raw_intrinsics = data["camera_edges"], data["intrinsics"]
        if not isinstance(raw_edges, Mapping) or not isinstance(raw_intrinsics, Mapping):
            raise CalibrationOverrideError("camera_edges and intrinsics must be objects")
        edges = {}
        for camera_name, raw_edge in raw_edges.items():
            if not isinstance(camera_name, str) or not isinstance(raw_edge, Mapping):
                raise CalibrationOverrideError("camera edge names must map to objects")
            if set(raw_edge) != {"target", "source", "matrix"}:
                raise CalibrationOverrideError(
                    f"camera_edges[{camera_name!r}] must contain target, source, and matrix"
                )
            try:
                edge = CameraEdgeOverride(
                    raw_edge["target"], raw_edge["source"], raw_edge["matrix"]
                )
            except (TypeError, ValueError) as exc:
                raise CalibrationOverrideError(
                    f"camera_edges[{camera_name!r}] is invalid: {exc}"
                ) from exc
            camera_ref = {"role": "camera", "camera_name": camera_name}
            if camera_ref not in (dict(edge.target), dict(edge.source)):
                raise CalibrationOverrideError(
                    f"camera_edges[{camera_name!r}] is not adjacent to its named camera"
                )
            edges[camera_name] = edge
        try:
            return cls(edges, raw_intrinsics)
        except (TypeError, ValueError) as exc:
            raise CalibrationOverrideError(f"scene override intrinsics are invalid: {exc}") from exc

    @classmethod
    def load(cls, path: str | Path) -> "SceneCalibrationOverride":
        source = Path(path)
        try:
            data = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise CalibrationOverrideError(
                f"could not read scene override {source}: {exc}"
            ) from exc
        return cls.from_dict(data)
