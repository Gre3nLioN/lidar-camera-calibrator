"""Stable package-level configuration, launcher, and result contracts."""
from __future__ import annotations

from collections import OrderedDict
from concurrent.futures import CancelledError, Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
import sys
from threading import RLock
from types import MappingProxyType
from typing import Literal, Mapping

import numpy as np

from .app import WorkspaceController
from .overrides import SceneCalibrationOverride
from .profile import FrameRef, SceneMcapReader


@dataclass(frozen=True)
class CalibrationConfig:
    frame_limit: int | None = None
    preload_count: int = 2
    renderer_mode: Literal["cpu", "production", "disabled"] = "cpu"
    playback_speed: float = 4.0
    window_title: str | None = None
    initial_override: str | Path | None = None

    def __post_init__(self) -> None:
        if self.frame_limit is not None and (
            isinstance(self.frame_limit, bool) or int(self.frame_limit) != self.frame_limit or self.frame_limit <= 0
        ):
            raise ValueError("frame_limit must be None or a positive integer")
        if isinstance(self.preload_count, bool) or int(self.preload_count) != self.preload_count or self.preload_count < 0:
            raise ValueError("preload_count must be a non-negative integer")
        if self.renderer_mode not in {"cpu", "production", "disabled"}:
            raise ValueError("renderer_mode must be cpu, production, or disabled")
        if self.frame_limit is not None:
            object.__setattr__(self, "frame_limit", int(self.frame_limit))
        object.__setattr__(self, "preload_count", int(self.preload_count))
        if isinstance(self.playback_speed, bool):
            raise ValueError("playback_speed must be finite and positive")
        speed = float(self.playback_speed)
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError("playback_speed must be finite and positive")
        object.__setattr__(self, "playback_speed", speed)
        if self.window_title is not None and not str(self.window_title).strip():
            raise ValueError("window_title must be None or non-empty")
        if self.initial_override is not None:
            if not str(self.initial_override).strip():
                raise ValueError("initial_override must be None or a non-empty path")
            object.__setattr__(self, "initial_override", Path(self.initial_override).expanduser())


@dataclass(frozen=True)
class CameraEdgeCalibration:
    camera_name: str
    target: FrameRef
    source: FrameRef
    original_matrix: np.ndarray
    matrix: np.ndarray

    def __post_init__(self) -> None:
        if not self.camera_name or not isinstance(self.target, FrameRef) or not isinstance(self.source, FrameRef):
            raise ValueError("camera edge requires a camera name and FrameRef target/source")
        if FrameRef.camera(self.camera_name) not in {self.target, self.source}:
            raise ValueError("camera edge must be adjacent to its named camera frame")
        for name in ("original_matrix", "matrix"):
            value = np.array(getattr(self, name), dtype=float, copy=True)
            if value.shape != (4, 4) or not np.isfinite(value).all():
                raise ValueError(f"{name} must be a finite 4x4 matrix")
            value.setflags(write=False)
            object.__setattr__(self, name, value)

    @property
    def changed(self) -> bool:
        return not np.array_equal(self.original_matrix, self.matrix)


@dataclass(frozen=True)
class CalibrationResult:
    camera_edges: Mapping[str, CameraEdgeCalibration]
    intrinsics: Mapping[str, np.ndarray]

    def __post_init__(self) -> None:
        edge_values = dict(self.camera_edges)
        if any(name != edge.camera_name for name, edge in edge_values.items()):
            raise ValueError("camera edge mapping keys must match edge camera names")
        if set(self.intrinsics) != set(edge_values):
            raise ValueError("intrinsics and camera edge mappings must contain the same cameras")
        edges = MappingProxyType(edge_values)
        intrinsics = {}
        for camera_name, matrix in self.intrinsics.items():
            value = np.array(matrix, dtype=float, copy=True)
            if value.shape != (3, 3) or not np.isfinite(value).all():
                raise ValueError(f"intrinsics[{camera_name!r}] must be a finite 3x3 matrix")
            value.setflags(write=False)
            intrinsics[camera_name] = value
        object.__setattr__(self, "camera_edges", edges)
        object.__setattr__(self, "intrinsics", MappingProxyType(intrinsics))

    @property
    def matrices(self) -> Mapping[str, np.ndarray]:
        return MappingProxyType({name: edge.matrix for name, edge in self.camera_edges.items()})

    @property
    def changed_cameras(self) -> tuple[str, ...]:
        return tuple(name for name, edge in self.camera_edges.items() if edge.changed)


class _ReaderView:
    """Count-limited reader with staged prefetch and a 40-frame rolling cache."""

    def __init__(self, reader: SceneMcapReader, frame_limit: int | None, preload_count: int) -> None:
        self.source = reader
        self.calibration = reader.calibration
        self._count = len(reader) if frame_limit is None else min(len(reader), int(frame_limit))
        self._initial_target = min(10, self._count)
        self._capacity = min(40, self._count)
        self._lock = RLock()
        self._cache: OrderedDict[int, object] = OrderedDict()
        self._loaded_indices: set[int] = set()
        self._futures: dict[int, Future] = {}
        self._errors: dict[int, Exception] = {}
        self._expanded_initial_window = False
        self._executor: ThreadPoolExecutor | None = None
        for index in range(min(int(preload_count), self._initial_target)):
            self._cache[index] = reader.frame(index)
            self._loaded_indices.add(index)
        # Keep Windows on the established synchronous Qt/NumPy safety path.
        if sys.platform != "win32" and len(self._cache) < self._count:
            self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="scene-frame-cache")
            if len(self._loaded_indices) >= self._initial_target:
                self._expanded_initial_window = True
                self._schedule_window(0)
            else:
                self._schedule_window(0, self._initial_target)

    def __len__(self) -> int:
        return self._count

    @property
    def buffer_target(self) -> int:
        return self._capacity

    @property
    def buffered_count(self) -> int:
        if self._executor is None:
            # Windows deliberately decodes synchronously; every in-range frame is
            # immediately available even though it is not retained in the LRU.
            return self.buffer_target
        with self._lock:
            return len(self._loaded_indices)

    def was_loaded(self, index: int) -> bool:
        if self._executor is None:
            return True
        with self._lock:
            return int(index) in self._loaded_indices

    def is_ready(self, index: int) -> bool:
        index = int(index)
        if self._executor is None:
            return True
        self._schedule_window(index)
        with self._lock:
            future = self._futures.get(index)
            return index in self._cache or index in self._errors or (future is not None and future.done())

    def frame(self, index: int):
        index = int(index)
        if index < 0 or index >= self._count:
            raise IndexError(index)
        with self._lock:
            cached = self._cache.get(index)
            error = self._errors.get(index)
        if cached is not None:
            self._schedule_window(index)
            return cached
        if error is not None:
            raise RuntimeError(f"failed to decode scene frame {index}") from error

        self._schedule_window(index)
        with self._lock:
            future = self._futures.get(index)
        frame = future.result() if future is not None else self.source.frame(index)
        self._store(index, frame)
        self._schedule_window(index)
        return frame

    def _schedule_window(self, anchor: int, window_size: int | None = None) -> None:
        executor = self._executor
        if executor is None:
            return
        size = self._capacity if window_size is None else min(int(window_size), self._capacity)
        desired = set(range(anchor, min(self._count, anchor + size)))
        with self._lock:
            for stale_index, future in tuple(self._futures.items()):
                if stale_index not in desired and future.cancel():
                    self._futures.pop(stale_index, None)
            for index in sorted(desired):
                if index in self._cache or index in self._futures or index in self._errors:
                    continue
                future = executor.submit(self.source.frame, index)
                self._futures[index] = future
                future.add_done_callback(
                    lambda completed, frame_index=index: self._publish(frame_index, completed)
                )

    def _publish(self, index: int, future: Future) -> None:
        try:
            frame = future.result()
        except CancelledError:
            with self._lock:
                self._futures.pop(index, None)
            return
        except Exception as exc:
            with self._lock:
                self._futures.pop(index, None)
                self._errors[index] = exc
            return
        with self._lock:
            self._futures.pop(index, None)
        self._store(index, frame)
        expand = False
        with self._lock:
            if not self._expanded_initial_window and all(
                frame_index in self._loaded_indices for frame_index in range(self._initial_target)
            ):
                self._expanded_initial_window = True
                expand = True
        if expand:
            self._schedule_window(0)

    def _store(self, index: int, frame: object) -> None:
        with self._lock:
            self._cache[index] = frame
            self._loaded_indices.add(index)
            while len(self._cache) > self._capacity:
                self._cache.popitem(last=False)

    def shutdown(self) -> None:
        executor = self._executor
        self._executor = None
        if executor is not None:
            executor.shutdown(wait=True, cancel_futures=True)
        with self._lock:
            self._cache.clear()
            self._loaded_indices.clear()
            self._futures.clear()
            self._errors.clear()


def launch_calibrator(
    scene_mcap: str | Path,
    config: CalibrationConfig | None = None,
) -> CalibrationResult:
    """Open a blocking calibration window and return final camera-edge overrides."""
    config = CalibrationConfig() if config is None else config
    if not isinstance(config, CalibrationConfig):
        raise TypeError("config must be CalibrationConfig or None")
    reader = SceneMcapReader(scene_mcap)
    adapter = _ReaderView(reader, config.frame_limit, config.preload_count)
    if not len(adapter):  # Canonical profiles already reject this, retained at API boundary.
        raise ValueError("scene profile contains no frames")
    primary_camera = reader.camera_names[0]
    controller = WorkspaceController(adapter, validation_camera=primary_camera)
    if config.initial_override is not None:
        controller.apply_scene_override(SceneCalibrationOverride.load(config.initial_override))
    controller.dispatch("set_playback_rate", rate=config.playback_speed)

    from .ui.main import run_controller
    try:
        run_controller(
            controller,
            renderer_mode=config.renderer_mode,
            window_title=config.window_title,
        )
        return _calibration_result(reader, controller)
    finally:
        adapter.shutdown()


def _calibration_result(
    reader: SceneMcapReader, controller: WorkspaceController
) -> CalibrationResult:
    edges = {}
    for camera_name in reader.camera_names:
        camera_delta = controller.calibration_delta_for_camera(camera_name)
        _, edge = reader.calibration_edge(camera_name)
        matrix = edge.matrix
        camera = FrameRef.camera(camera_name)
        if edge.target == camera:
            matrix = camera_delta @ edge.matrix
        elif edge.source == camera:
            matrix = edge.matrix @ np.linalg.inv(camera_delta)
        edges[camera_name] = CameraEdgeCalibration(
            camera_name, edge.target, edge.source, edge.matrix, matrix
        )
    intrinsics = {
        camera_name: controller.working.intrinsics(camera_name)
        for camera_name in reader.camera_names
    }
    return CalibrationResult(edges, intrinsics)
