"""Qt-free runtime controller for the KITTI calibration workspace."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from ..models import FrameBundle, LoadedCalibration, WorkingCalibration
from ..overrides import (
    CalibrationOverride, CalibrationOverrideError, CameraEdgeOverride, SceneCalibrationOverride,
)
from .commands import CalibrationCommand
from .state import (
    ExportSnapshot, OverlaySettings, RendererReadyInput, TimelineSnapshot,
    WorkingCalibrationSnapshot, WorkspaceSnapshot,
)


_DENSITIES = {"light": 0.20, "medium": 0.08, "full": 0.02}
_TRANSLATION_AXES = {"x": 0, "y": 1, "z": 2}
_ROTATION_AXES = {"roll": 0, "pitch": 1, "yaw": 2}
_INTRINSIC_NAMES = {"fx", "fy", "cx", "cy"}


@dataclass(frozen=True)
class _CalibrationHistoryState:
    translations: dict[str, np.ndarray]
    rotations: dict[str, np.ndarray]
    intrinsics: dict[str, np.ndarray]


class WorkspaceController:
    """Owns one synchronized timeline and one sequence-static working calibration.

    The controller deliberately does not project or filter points.  It passes raw
    Velodyne XYZ[I] and canonical matrices to the renderer boundary.
    """

    def __init__(self, adapter: Any, validation_camera: str = "image_02") -> None:
        self.adapter = adapter
        self.loaded: LoadedCalibration = adapter.calibration
        self.validation_camera = validation_camera
        if validation_camera not in self.loaded.cameras:
            raise ValueError(f"unknown validation camera: {validation_camera}")
        self.selected_camera = validation_camera
        self.working = WorkingCalibration(self.loaded)
        self._camera_translation_offsets = {
            camera_id: np.zeros(3, dtype=float) for camera_id in self.loaded.cameras
        }
        self._camera_rotation_offsets = {
            camera_id: np.zeros(3, dtype=float) for camera_id in self.loaded.cameras
        }
        self._activate_camera_offsets(validation_camera)
        self.intrinsics_enabled = False
        self.overlay = OverlaySettings()
        self.is_playing = False
        self.playback_rate = 1.0
        self._revision = 0
        self._saved_signature = self._calibration_signature()
        self._undo_stack: list[_CalibrationHistoryState] = []
        self._redo_stack: list[_CalibrationHistoryState] = []
        self._pending_history_state: _CalibrationHistoryState | None = None
        self._export = ExportSnapshot()
        self._requested_index = 0
        self._frame: FrameBundle | None = None
        self._displayed: FrameBundle | None = None
        self._projection_status = "idle"
        self._projection_message: str | None = None
        if len(adapter):
            self._load_frame(0)

    @property
    def frame_count(self) -> int:
        return len(self.adapter)

    def snapshot(self) -> WorkspaceSnapshot:
        timestamp = self._frame.timestamp if self._frame is not None else 0.0
        timeline = TimelineSnapshot(self._requested_index, self.frame_count, timestamp, self.is_playing, self.playback_rate)
        working = WorkingCalibrationSnapshot(
            self.working.translation_offset_metres,
            self.working.rotation_offset_degrees,
            {self.selected_camera: self.working.intrinsics(self.selected_camera)},
            self.intrinsics_enabled,
        )
        renderer_input = self._renderer_input()
        return WorkspaceSnapshot(
            self.loaded.dataset,
            self.loaded.sequence,
            self.selected_camera,
            timeline,
            self._frame,
            self._displayed,
            self.loaded,
            working,
            self.dirty,
            self._projection_status,
            self._projection_message,
            renderer_input,
            self.overlay,
            bool(self._undo_stack),
            bool(self._redo_stack),
            self._export,
        )

    def _calibration_signature(self) -> tuple[Any, ...]:
        values: list[Any] = []
        for camera_id in sorted(self.loaded.cameras):
            values.extend((
                camera_id,
                tuple(self._camera_translation_offsets[camera_id]),
                tuple(self._camera_rotation_offsets[camera_id]),
            ))
        for camera_id in sorted(self.working.intrinsic_overrides):
            values.extend((camera_id, tuple(self.working.intrinsic_overrides[camera_id].reshape(-1))))
        return tuple(values)

    @property
    def dirty(self) -> bool:
        return self._calibration_signature() != self._saved_signature

    def _capture_history_state(self) -> _CalibrationHistoryState:
        return _CalibrationHistoryState(
            {camera_id: np.array(value, copy=True) for camera_id, value in self._camera_translation_offsets.items()},
            {camera_id: np.array(value, copy=True) for camera_id, value in self._camera_rotation_offsets.items()},
            {camera_id: np.array(matrix, copy=True) for camera_id, matrix in self.working.intrinsic_overrides.items()},
        )

    @staticmethod
    def _history_states_equal(left: _CalibrationHistoryState, right: _CalibrationHistoryState) -> bool:
        return (
            left.translations.keys() == right.translations.keys()
            and all(np.array_equal(left.translations[key], right.translations[key]) for key in left.translations)
            and left.rotations.keys() == right.rotations.keys()
            and all(np.array_equal(left.rotations[key], right.rotations[key]) for key in left.rotations)
            and left.intrinsics.keys() == right.intrinsics.keys()
            and all(np.array_equal(left.intrinsics[key], right.intrinsics[key]) for key in left.intrinsics)
        )

    def _restore_history_state(self, state: _CalibrationHistoryState) -> None:
        self._camera_translation_offsets = {
            camera_id: np.array(value, copy=True) for camera_id, value in state.translations.items()
        }
        self._camera_rotation_offsets = {
            camera_id: np.array(value, copy=True) for camera_id, value in state.rotations.items()
        }
        self._activate_camera_offsets(self.selected_camera)
        self.working.intrinsic_overrides = {
            camera_id: np.array(matrix, copy=True) for camera_id, matrix in state.intrinsics.items()
        }

    def _finish_numeric_change(self, before: _CalibrationHistoryState) -> None:
        if self._pending_history_state is None and not self._history_states_equal(before, self._capture_history_state()):
            self._undo_stack.append(before)
            self._redo_stack.clear()
        self._changed()

    def dispatch(self, command: CalibrationCommand | str | Mapping[str, Any], **payload: Any) -> WorkspaceSnapshot:
        if isinstance(command, str):
            name, args = command, payload
        elif isinstance(command, Mapping):
            # Compatibility with thin Qt bridges that serialize positional args.
            name = str(command.get("name", ""))
            positional = tuple(command.get("args", ()))
            names = {
                "seek": ("frame_index",), "select_camera": ("camera_id",),
                "adjust_translation": ("axis", "value"), "adjust_rotation": ("axis", "value"),
                "reset_parameter": ("name",), "set_intrinsic": ("name", "value"), "set_intrinsics_enabled": ("enabled",),
                "set_overlay_setting": ("name", "value"), "confirm_export": ("path",),
            }.get(name, ())
            args = dict(zip(names, positional))
            args.update(payload)
        else:
            name, args = command.name, dict(command.payload)
            args.update(payload)
        handler = getattr(self, f"_cmd_{name}", None)
        if handler is None:
            raise ValueError(f"unknown workspace command: {name}")
        handler(**args)
        return self.snapshot()

    def play(self) -> WorkspaceSnapshot:
        return self.dispatch("play")

    def pause(self) -> WorkspaceSnapshot:
        return self.dispatch("pause")

    def seek(self, frame_index: int) -> WorkspaceSnapshot:
        return self.dispatch("seek", frame_index=frame_index)

    def select_camera(self, camera_id: str) -> WorkspaceSnapshot:
        return self.dispatch("select_camera", camera_id=camera_id)

    def adjust_translation(self, axis: str, value: float) -> WorkspaceSnapshot:
        return self.dispatch("adjust_translation", axis=axis, value=value)

    def adjust_rotation(self, axis: str, value: float) -> WorkspaceSnapshot:
        return self.dispatch("adjust_rotation", axis=axis, value=value)

    def set_intrinsic(self, name: str, value: float) -> WorkspaceSnapshot:
        return self.dispatch("set_intrinsic", name=name, value=value)

    def _cmd_play(self) -> None:
        self.is_playing = True

    def _cmd_pause(self) -> None:
        self.is_playing = False

    def _cmd_set_playback_rate(self, rate: float) -> None:
        rate = float(rate)
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError("playback rate must be finite and positive")
        self.playback_rate = rate

    def _cmd_seek(self, frame_index: int) -> None:
        self._load_frame(int(frame_index))

    def _cmd_select_camera(self, camera_id: str) -> None:
        if camera_id not in self.loaded.cameras:
            raise ValueError(f"unknown camera: {camera_id}")
        self.selected_camera = camera_id
        self._activate_camera_offsets(camera_id)
        self._update_projection_state()

    def _cmd_begin_calibration_change(self) -> None:
        if self._pending_history_state is None:
            self._pending_history_state = self._capture_history_state()

    def _cmd_end_calibration_change(self) -> None:
        before = self._pending_history_state
        self._pending_history_state = None
        if before is not None and not self._history_states_equal(before, self._capture_history_state()):
            self._undo_stack.append(before)
            self._redo_stack.clear()

    def _cmd_undo(self) -> None:
        if not self._undo_stack:
            return
        self._redo_stack.append(self._capture_history_state())
        self._restore_history_state(self._undo_stack.pop())
        self._changed()

    def _cmd_redo(self) -> None:
        if not self._redo_stack:
            return
        self._undo_stack.append(self._capture_history_state())
        self._restore_history_state(self._redo_stack.pop())
        self._changed()

    def _cmd_adjust_translation(self, axis: str, value: float) -> None:
        key = str(axis).lower()
        if key not in _TRANSLATION_AXES:
            raise ValueError("translation axis must be x, y, or z")
        value = float(value)
        if not np.isfinite(value):
            raise ValueError("translation must be finite")
        before = self._pending_history_state or self._capture_history_state()
        self.working.translation_offset_metres[_TRANSLATION_AXES[key]] = value
        self._finish_numeric_change(before)

    def _cmd_adjust_rotation(self, axis: str, value: float) -> None:
        key = str(axis).lower()
        if key not in _ROTATION_AXES:
            raise ValueError("rotation axis must be roll, pitch, or yaw")
        value = float(value)
        if not np.isfinite(value):
            raise ValueError("rotation must be finite")
        before = self._pending_history_state or self._capture_history_state()
        self.working.rotation_offset_degrees[_ROTATION_AXES[key]] = value
        self._finish_numeric_change(before)

    def _cmd_set_intrinsics_enabled(self, enabled: bool) -> None:
        self.intrinsics_enabled = bool(enabled)

    def _cmd_set_intrinsic(self, name: str, value: float) -> None:
        if not self.intrinsics_enabled:
            raise ValueError("intrinsic editing is disabled")
        name = str(name).lower()
        if name not in _INTRINSIC_NAMES:
            raise ValueError(f"unknown intrinsic: {name}")
        value = float(value)
        if not np.isfinite(value) or (name in {"fx", "fy"} and value <= 0):
            raise ValueError("intrinsic value must be finite and focal lengths positive")
        before = self._pending_history_state or self._capture_history_state()
        k = np.array(self.working.intrinsics(self.selected_camera), copy=True)
        if name == "fx":
            k[0, 0] = value
        elif name == "fy":
            k[1, 1] = value
        elif name == "cx":
            k[0, 2] = value
        else:
            k[1, 2] = value
        self.working.intrinsic_overrides[self.selected_camera] = k
        self._finish_numeric_change(before)

    def _cmd_reset_parameter(self, name: str) -> None:
        name = str(name).lower()
        aliases = {
            "translation_x": "x", "translation_y": "y", "translation_z": "z",
            "rotation_roll": "roll", "rotation_pitch": "pitch", "rotation_yaw": "yaw",
        }
        name = aliases.get(name, name)
        before = self._capture_history_state()
        if name in _TRANSLATION_AXES:
            self.working.translation_offset_metres[_TRANSLATION_AXES[name]] = 0.0
        elif name in _ROTATION_AXES:
            self.working.rotation_offset_degrees[_ROTATION_AXES[name]] = 0.0
        elif name in _INTRINSIC_NAMES:
            k = np.array(self.working.intrinsics(self.selected_camera), copy=True)
            loaded = self.loaded.cameras[self.selected_camera].intrinsics
            idx = {"fx": (0, 0), "fy": (1, 1), "cx": (0, 2), "cy": (1, 2)}[name]
            k[idx] = loaded[idx]
            if np.array_equal(k, loaded):
                self.working.intrinsic_overrides.pop(self.selected_camera, None)
            else:
                self.working.intrinsic_overrides[self.selected_camera] = k
        else:
            raise ValueError(f"unknown calibration parameter: {name}")
        self._finish_numeric_change(before)

    def _cmd_reset_extrinsics(self) -> None:
        before = self._capture_history_state()
        self.working.reset_extrinsics()
        self._finish_numeric_change(before)

    def _cmd_reset_intrinsics(self) -> None:
        before = self._capture_history_state()
        self.working.intrinsic_overrides.pop(self.selected_camera, None)
        self._finish_numeric_change(before)

    def _cmd_reset_all(self) -> None:
        before = self._capture_history_state()
        self.working.reset_extrinsics()
        self.working.intrinsic_overrides.pop(self.selected_camera, None)
        self._finish_numeric_change(before)

    def _cmd_set_overlay_setting(self, name: str, value: Any) -> None:
        key = str(name)
        values = self.overlay.__dict__.copy()
        if key == "render_density":
            value = str(value).lower()
            if value not in _DENSITIES:
                raise ValueError("render_density must be light, medium, or full")
            values["voxel_size_metres"] = _DENSITIES[value]
        elif key not in values:
            raise ValueError(f"unknown overlay setting: {key}")
        values[key] = value
        if key in {"point_size_px", "opacity", "depth_min_metres", "depth_max_metres", "voxel_size_metres"}:
            values[key] = float(value)
        if not 0 <= float(values["opacity"]) <= 1:
            raise ValueError("opacity must be between 0 and 1")
        self.overlay = OverlaySettings(**values)

    def _cmd_request_export(self, suggested_path: str | Path | None = None) -> None:
        target = str(suggested_path) if suggested_path is not None else self._export.suggested_path
        self._export = ExportSnapshot("confirming", target, None)

    def _cmd_cancel_export(self) -> None:
        self._export = ExportSnapshot("idle", self._export.suggested_path, None)

    def _cmd_confirm_export(self, path: str | Path | None = None) -> None:
        if self._export.status != "confirming":
            raise ValueError("export confirmation is required before writing an override")
        target = Path(path or self._export.suggested_path)
        try:
            self._export_override().save(target)
        except (OSError, ValueError, TypeError) as exc:
            self._export = ExportSnapshot("failed", str(target), str(exc))
            return
        self._saved_signature = self._calibration_signature()
        self._export = ExportSnapshot("succeeded", str(target), None)

    def _cmd_copy_calibration(self) -> str:
        # dispatch() returns a snapshot; this convenience method returns JSON below.
        return self._export_override().to_json()

    def copy_calibration(self) -> str:
        return self._export_override().to_json()

    def _export_override(self) -> CalibrationOverride | SceneCalibrationOverride:
        reader = getattr(self.adapter, "source", self.adapter)
        if not hasattr(reader, "calibration_edge") or not hasattr(reader, "transform_between"):
            return CalibrationOverride(self.transform_for_camera(self.selected_camera))

        from ..profile import FrameRef
        edges = {}
        intrinsics = {}
        for camera_name in self.loaded.cameras:
            _, edge = reader.calibration_edge(camera_name)
            delta = self.calibration_delta_for_camera(camera_name)
            matrix = edge.matrix
            camera_frame = FrameRef.camera(camera_name)
            if edge.target == camera_frame:
                matrix = delta @ edge.matrix
            elif edge.source == camera_frame:
                matrix = edge.matrix @ np.linalg.inv(delta)
            if not np.array_equal(matrix, edge.matrix):
                edges[camera_name] = CameraEdgeOverride(
                    self._frame_ref_dict(edge.target),
                    self._frame_ref_dict(edge.source),
                    matrix,
                )
            loaded_intrinsics = self.loaded.cameras[camera_name].intrinsics
            working_intrinsics = self.working.intrinsics(camera_name)
            if not np.array_equal(working_intrinsics, loaded_intrinsics):
                intrinsics[camera_name] = working_intrinsics
        return SceneCalibrationOverride(edges, intrinsics)

    @staticmethod
    def _frame_ref_dict(frame) -> dict[str, str]:
        result = {"role": frame.role.value}
        if frame.camera_name is not None:
            result["camera_name"] = frame.camera_name
        return result

    def apply_scene_override(self, override: SceneCalibrationOverride) -> None:
        """Apply one canonical save file as initial state without creating history."""
        if not isinstance(override, SceneCalibrationOverride):
            raise TypeError("override must be SceneCalibrationOverride")
        reader = getattr(self.adapter, "source", self.adapter)
        if not hasattr(reader, "calibration_edge"):
            raise CalibrationOverrideError("scene overrides require a canonical scene reader")
        unknown = (set(override.camera_edges) | set(override.intrinsics)) - set(self.loaded.cameras)
        if unknown:
            raise CalibrationOverrideError(
                f"override references cameras absent from this scene: {sorted(unknown)!r}"
            )

        from ..profile import FrameRef, TransformSpec
        from ..transforms import rotation_matrix_to_euler_xyz_degrees
        for camera_name, saved in override.camera_edges.items():
            _, original = reader.calibration_edge(camera_name)
            try:
                saved_target = FrameRef.from_mapping(saved.target)
                saved_source = FrameRef.from_mapping(saved.source)
                TransformSpec(saved_target, saved_source, saved.matrix)
            except (TypeError, ValueError) as exc:
                raise CalibrationOverrideError(
                    f"camera_edges[{camera_name!r}] is not a valid rigid edge: {exc}"
                ) from exc
            if (saved_target, saved_source) != (original.target, original.source):
                raise CalibrationOverrideError(
                    f"camera_edges[{camera_name!r}] direction does not match the scene; "
                    f"expected {self._frame_ref_dict(original.target)} from "
                    f"{self._frame_ref_dict(original.source)}"
                )
            camera = FrameRef.camera(camera_name)
            if original.target == camera:
                delta = saved.matrix @ np.linalg.inv(original.matrix)
            elif original.source == camera:
                delta = np.linalg.inv(saved.matrix) @ original.matrix
            else:  # protected by calibration_edge(), retained as a defensive contract check
                raise CalibrationOverrideError(
                    f"camera_edges[{camera_name!r}] is not camera-adjacent in the scene"
                )
            try:
                TransformSpec(camera, FrameRef.lidar(), delta)
            except ValueError as exc:
                raise CalibrationOverrideError(
                    f"camera_edges[{camera_name!r}] produces an invalid rigid delta: {exc}"
                ) from exc
            self._camera_translation_offsets[camera_name] = np.array(delta[:3, 3], copy=True)
            self._camera_rotation_offsets[camera_name] = rotation_matrix_to_euler_xyz_degrees(delta[:3, :3])

        for camera_name, matrix in override.intrinsics.items():
            value = np.asarray(matrix, dtype=float)
            if (
                value.shape != (3, 3) or not np.isfinite(value).all()
                or value[0, 0] <= 0 or value[1, 1] <= 0
                or not np.allclose(value[2], [0, 0, 1], atol=1e-9)
            ):
                raise CalibrationOverrideError(
                    f"intrinsics[{camera_name!r}] must be finite with positive focal lengths "
                    "and row [0, 0, 1]"
                )
            self.working.intrinsic_overrides[camera_name] = np.array(value, copy=True)
        if override.intrinsics:
            self.intrinsics_enabled = True

        self._undo_stack.clear()
        self._redo_stack.clear()
        self._pending_history_state = None
        self._activate_camera_offsets(self.selected_camera)
        self._changed()

    def _load_frame(self, index: int) -> None:
        if index < 0 or index >= self.frame_count:
            raise IndexError(index)
        self._requested_index = index
        try:
            frame = self.adapter.frame(index)
        except Exception as exc:
            self._projection_status, self._projection_message = "unavailable", str(exc)
            return
        self._frame = frame
        self._displayed = frame
        self._update_projection_state()

    def _update_projection_state(self) -> None:
        if self._frame is None:
            self._projection_status, self._projection_message = "unavailable", "No frame loaded"
        elif self._frame.cameras.get(self.selected_camera) is None:
            self._projection_status, self._projection_message = "unavailable", "Synchronized camera image unavailable"
        elif self._frame.lidar_points.size == 0:
            self._projection_status, self._projection_message = "unavailable", "LiDAR frame is empty"
        else:
            self._projection_status, self._projection_message = "ready", None

    def _changed(self) -> None:
        self._revision += 1
        self._update_projection_state()

    def _activate_camera_offsets(self, camera_id: str) -> None:
        self.working.translation_offset_metres = self._camera_translation_offsets[camera_id]
        self.working.rotation_offset_degrees = self._camera_rotation_offsets[camera_id]

    def base_transform_for_camera(self, camera_id: str) -> np.ndarray:
        return self.loaded.lidar_to_cameras.get(
            camera_id, self.loaded.t_camera_00_from_velodyne
        )

    def calibration_delta_for_camera(self, camera_id: str) -> np.ndarray:
        from ..transforms import compose_working_transform
        return compose_working_transform(
            np.eye(4),
            self._camera_translation_offsets[camera_id],
            self._camera_rotation_offsets[camera_id],
        )

    def transform_for_camera(self, camera_id: str) -> np.ndarray:
        return self.calibration_delta_for_camera(camera_id) @ self.base_transform_for_camera(camera_id)

    def _projection_model(self, camera_id: str) -> dict[str, Any]:
        camera = self.loaded.cameras[camera_id]
        rectification_reference = self.loaded.cameras.get("image_00", camera).rectification
        return {
            "transform": self.transform_for_camera(camera_id),
            "projection_matrix": camera.projection_matrix,
            "rectification_00": rectification_reference,
            "intrinsics": self.working.intrinsics(camera_id),
            "rectified_translation": np.linalg.solve(camera.projection_matrix[:, :3], camera.projection_matrix[:, 3]),
            "original_transform": self.base_transform_for_camera(camera_id),
            "original_intrinsics": camera.intrinsics,
            "baseline_rule": "P_rect_working=[K_working | K_working @ inv(K_loaded) @ P_rect_loaded[:,3]]; R_rect_00 exactly once",
            "image_size": camera.image_size,
        }

    def _renderer_input(self) -> RendererReadyInput | None:
        # Never hand an unavailable/empty frame to the renderer. The UI retains
        # the last displayed image and uses the typed status/message instead.
        if self._projection_status != "ready" or self._frame is None or self.selected_camera not in self.loaded.cameras:
            return None
        model = self._projection_model(self.selected_camera)
        model["camera_models"] = {
            camera_id: self._projection_model(camera_id)
            for camera_id in self.loaded.cameras
        }
        return RendererReadyInput(self._frame.frame_index, self.selected_camera, self._frame.lidar_points, model, self.overlay, self._revision)
