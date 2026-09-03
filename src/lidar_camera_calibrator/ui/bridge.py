from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import numpy as np
from PySide6.QtCore import QObject, Property, QRunnable, QThreadPool, QTimer, QUrl, Signal, Slot


class _PreparationSignals(QObject):
    finished = Signal(object, object)


class _PreparationTask(QRunnable):
    """Worker-side candidate preparation; never touches Qt Quick resources."""
    def __init__(self, token, renderer_input, cache):
        super().__init__()
        self.token = token
        self.renderer_input = renderer_input
        self.cache = cache
        self.signals = _PreparationSignals()

    def run(self):
        try:
            from ..rendering import CandidateEnvelope, prepare_renderer_input
            result = prepare_renderer_input(self.renderer_input, CandidateEnvelope(), cache=self.cache)
            self.signals.finished.emit(self.token, result)
        except Exception as exc:  # delivered to UI thread as an unavailable state
            self.signals.finished.emit(self.token, exc)



class WorkspaceBridge(QObject):
    """Qt-facing semantic boundary.

    A controller can be injected later; until then this deterministic mock keeps
    every QML state and transition usable without geometry/rendering code.
    """
    snapshotChanged = Signal()
    exportRequested = Signal()

    def __init__(self, controller=None, parent=None, startup_error=None, renderer_disabled=False, cpu_overlay=False):
        super().__init__(parent)
        self.controller = controller
        self._startup_error = str(startup_error or "")
        self._renderer_disabled = bool(renderer_disabled)
        self._cpu_overlay = bool(cpu_overlay)
        self._camera = ""
        self._camera_names = []
        self._camera_details = {}
        self._available_cameras = set()
        self._all_synchronized = False
        self._dataset_label = "KITTI / 0005"
        self._profile_label = "KITTI"
        self._sequence_label = "0005"
        self._frame = 184
        self._frame_count = 454
        self._timestamp = 6 * 60 + 42.183
        self._playing = False
        try:
            self._playback_speed = float(os.environ.get("LIDAR_CALIBRATOR_PLAYBACK_SPEED", "4.0"))
        except ValueError:
            self._playback_speed = 4.0
        self._playback_speed = float(np.clip(self._playback_speed, 0.25, 8.0))
        self._playback_waiting = False
        self._buffered_count = 0
        self._buffer_target = 0
        self._pending_seek = None
        self._dirty = False
        self._projection_status = "ready"
        self._projection_message = ""
        self._renderer_ready = False
        self._image_available = False
        self._export_status = "idle"
        self._export_path = ""
        self._export_success_id = 0
        self._can_undo = False
        self._can_redo = False
        self._renderer_item = None
        self._renderer_points_token = None
        self._renderer_pending_token = None
        self._renderer_uniforms = None
        self._prep_cache = None
        self._cpu_prepared_token = None
        self._cpu_prepared_points = None
        self._cpu_prepared_transform = None
        self._cpu_prepared_intrinsics = None
        self._cpu_prepared_translation = None
        self._cpu_envelope_revision = 0
        self._cpu_overlay_image = None
        self._cpu_overlay_revision = 0
        self._cpu_original_overlay_image = None
        self._cpu_original_overlay_revision = 0
        self._cpu_original_prepared_token = None
        self._cpu_original_prepared_points = None
        self._cpu_original_render_token = None
        self._preview_projection_enabled = False
        self._calibration_active = True
        self._preview_overlay_images = {camera_id: None for camera_id in self._camera_names}
        self._preview_overlay_revisions = {camera_id: 0 for camera_id in self._camera_names}
        self._preview_overlay_tokens = {camera_id: None for camera_id in self._camera_names}
        try:
            self._preview_max_points = int(os.environ.get("LIDAR_CALIBRATOR_PREVIEW_MAX_POINTS", "25000"))
        except ValueError:
            self._preview_max_points = 25_000
        self._preview_max_points = max(2_000, min(self._preview_max_points, 100_000))
        self._lidar_scene_image = None
        self._lidar_scene_revision = 0
        self._lidar_scene_frame_token = None
        self._lidar_points = None
        self._lidar_point_count = 0
        self._lidar_rendered_count = 0
        self._lidar_pick_indices = None
        self._lidar_pivot = np.zeros(3, dtype=np.float32)
        self._lidar_base_scale_ratio = None
        self._lidar_refine_timer = QTimer(self)
        self._lidar_refine_timer.setSingleShot(True)
        self._lidar_refine_timer.setInterval(70)
        self._lidar_refine_timer.timeout.connect(self._refine_lidar_scene)
        self._lidar_view = {"yaw": -18.0, "pitch": 58.0, "zoom": 1.0, "pan_x": 0.0, "pan_y": 0.0,
                            "width": 1280, "height": 720, "interactive": False,
                            "anchor_x": 0.5, "anchor_y": 0.5}
        try:
            self._cpu_max_points = int(os.environ.get("LIDAR_CALIBRATOR_CPU_MAX_POINTS", "150000"))
        except ValueError:
            self._cpu_max_points = 150_000
        self._cpu_max_points = max(1_000, min(self._cpu_max_points, 1_000_000))
        preparation_mode = os.environ.get(
            "LIDAR_CALIBRATOR_PREP_MODE",
            os.environ.get("LIDAR_CALIBRATOR_PREPARATION_MODE", "auto"),
        ).lower()
        if preparation_mode not in {"auto", "sync", "async"}:
            preparation_mode = "auto"
        # NumPy preparation in a Qt pooled thread reproducibly corrupts the
        # Windows heap (0xc0000374). Prefer the stable UI-thread path there;
        # diagnostic users can explicitly choose async or sync on any platform.
        self._prepare_synchronously = (
            preparation_mode == "sync"
            or (preparation_mode == "auto" and sys.platform == "win32")
        )
        if preparation_mode == "async":
            self._prepare_synchronously = False
        self._safe_preparation_mode = self._prepare_synchronously
        self._prep_pool = None
        if not self._prepare_synchronously:
            self._prep_pool = QThreadPool(self)
            self._prep_pool.setMaxThreadCount(1)
        # QRunnable instances must outlive their worker execution. Without this
        # registry Qt may auto-delete a task while Python/NumPy still uses it.
        self._prep_tasks = {}
        self._overlay_open = False
        self._comparison_open = False
        self._export_open = False
        self._processing = False
        self._overlay_settings = {"coloring": "depth", "pointSizePx": 3.0, "opacity": 0.72, "depthMinMetres": 2.0, "depthMaxMetres": 60.0, "renderDensity": "medium", "onlyPointsInImage": True}
        self._intrinsics_enabled = False
        self._playback_timer = QTimer(self)
        self._playback_timer.setSingleShot(True)
        self._playback_timer.timeout.connect(self._advance_playback)
        self._buffer_timer = QTimer(self)
        self._buffer_timer.setInterval(100)
        self._buffer_timer.timeout.connect(self._refresh_buffer_status)
        self._values = {"x": 0.012, "y": -0.004, "z": 0.0, "roll": 0.08, "pitch": -0.12, "yaw": 0.03,
                        "fx": 721.58, "fy": 721.54, "cx": 609.56, "cy": 172.85}
        if self.controller is not None:
            self._refresh_buffer_status(emit=False)
            self._sync_controller(self.controller.snapshot())
            if self._buffered_count < self._buffer_target:
                self._buffer_timer.start()

    def _emit(self):
        self.snapshotChanged.emit()

    def _dispatch(self, name, *args):
        if self.controller is None or not hasattr(self.controller, "dispatch"):
            return
        result = self.controller.dispatch({"name": name, "args": args})
        if result is not None:
            self._sync_controller(result)

    def _sync_controller(self, snapshot):
        """Project the controller's immutable semantic snapshot into QML properties."""
        self._camera = snapshot.selected_camera
        self._camera_names = list(snapshot.loaded_calibration.cameras)
        self._profile_label = snapshot.dataset_label
        self._sequence_label = snapshot.sequence_id or "Untitled scene"
        self._dataset_label = self._profile_label
        if snapshot.sequence_id:
            self._dataset_label += f" / {snapshot.sequence_id}"
        for camera_id in self._camera_names:
            self._preview_overlay_images.setdefault(camera_id, None)
            self._preview_overlay_revisions.setdefault(camera_id, 0)
            self._preview_overlay_tokens.setdefault(camera_id, None)
        self._preview_overlay_images = {
            camera_id: self._preview_overlay_images[camera_id] for camera_id in self._camera_names
        }
        self._preview_overlay_revisions = {
            camera_id: self._preview_overlay_revisions[camera_id] for camera_id in self._camera_names
        }
        self._preview_overlay_tokens = {
            camera_id: self._preview_overlay_tokens[camera_id] for camera_id in self._camera_names
        }
        self._frame = snapshot.timeline.frame_index
        self._frame_count = snapshot.timeline.frame_count
        self._timestamp = snapshot.timeline.timestamp
        self._playing = snapshot.timeline.is_playing
        self._dirty = snapshot.dirty
        self._projection_status = snapshot.projection_status
        self._projection_message = snapshot.projection_message or ""
        self._renderer_ready = snapshot.renderer_input is not None
        frame = snapshot.frame_bundle or snapshot.displayed_bundle
        self._available_cameras = {
            camera_id for camera_id in self._camera_names
            if frame is not None and frame.cameras.get(camera_id) is not None
        }
        self._all_synchronized = bool(frame is not None) and len(self._available_cameras) == len(self._camera_names)
        self._camera_details = {}
        for camera_id in self._camera_names:
            calibration = snapshot.loaded_calibration.cameras[camera_id]
            current_camera = frame.cameras.get(camera_id) if frame is not None else None
            sync_delta_ms = (
                (current_camera.timestamp - frame.timestamp) * 1000.0
                if current_camera is not None else None
            )
            extrinsics_modified = False
            intrinsics_modified = not np.array_equal(
                self.controller.working.intrinsics(camera_id), calibration.intrinsics
            )
            if hasattr(self.controller, "calibration_delta_for_camera"):
                extrinsics_modified = not np.allclose(
                    self.controller.calibration_delta_for_camera(camera_id), np.eye(4),
                    atol=1e-12, rtol=0,
                )
            self._camera_details[camera_id] = {
                "resolution": f"{calibration.image_size[0]} × {calibration.image_size[1]}",
                "syncDeltaMs": sync_delta_ms,
                "syncLabel": "unavailable" if sync_delta_ms is None else f"{sync_delta_ms:+.1f} ms",
                "ready": current_camera is not None,
                "extrinsicsModified": extrinsics_modified,
                "intrinsicsModified": intrinsics_modified,
                "modified": extrinsics_modified or intrinsics_modified,
            }
        camera_frame = frame.cameras.get(self._camera) if frame is not None else None
        self._image_available = camera_frame is not None
        lidar_token = None if frame is None else frame.frame_index
        if lidar_token != self._lidar_scene_frame_token:
            first_lidar_frame = self._lidar_scene_frame_token is None
            self._lidar_scene_frame_token = lidar_token
            self._lidar_points = None if frame is None else frame.lidar_points
            # Picking is frame-specific. Camera orientation, zoom, pan, pivot,
            # and fitted scale remain stable while playing or scrubbing.
            self._lidar_pick_indices = None
            if first_lidar_frame:
                self._lidar_base_scale_ratio = None
                if self._lidar_points is not None and len(self._lidar_points):
                    xyz = np.asarray(self._lidar_points[:, :3], dtype=np.float32)
                    xyz = xyz[np.isfinite(xyz).all(axis=1)]
                    if len(xyz):
                        low, high = np.percentile(xyz, (1.0, 99.0), axis=0)
                        self._lidar_pivot = ((low + high) * 0.5).astype(np.float32)
            self._lidar_refine_timer.stop()
            self._push_lidar_scene(quick=not first_lidar_frame)
            if not first_lidar_frame:
                self._lidar_refine_timer.start()
        self._processing = snapshot.projection_status == "preparing"
        previous_export_status = self._export_status
        self._export_status = snapshot.export.status
        self._export_path = snapshot.export.suggested_path
        if self._export_status == "succeeded" and previous_export_status != "succeeded":
            self._export_success_id += 1
        self._can_undo = snapshot.can_undo
        self._can_redo = snapshot.can_redo
        working = snapshot.working_calibration
        for key, value in zip(("x", "y", "z"), working.translation_offset_metres):
            self._values[key] = float(value)
        for key, value in zip(("roll", "pitch", "yaw"), working.rotation_offset_degrees):
            self._values[key] = float(value)
        self._intrinsics_enabled = working.intrinsics_enabled
        matrix = working.intrinsics.get(self._camera)
        if matrix is not None:
            self._values.update({"fx": float(matrix[0, 0]), "fy": float(matrix[1, 1]),
                                 "cx": float(matrix[0, 2]), "cy": float(matrix[1, 2])})
        overlay = snapshot.overlay
        self._overlay_settings.update({"coloring": overlay.coloring, "pointSizePx": overlay.point_size_px,
            "opacity": overlay.opacity, "depthMinMetres": overlay.depth_min_metres,
            "depthMaxMetres": overlay.depth_max_metres, "renderDensity": overlay.render_density,
            "onlyPointsInImage": overlay.only_points_in_image})
        if self._cpu_overlay:
            if self._calibration_active:
                self._push_cpu_overlay(snapshot)
            self._push_preview_overlays(snapshot)
        else:
            self._push_renderer(snapshot)
        self._emit()

    @Property(str, notify=snapshotChanged)
    def startupError(self): return self._startup_error
    @Property(bool, notify=snapshotChanged)
    def rendererDisabled(self): return self._renderer_disabled
    @Property(str, notify=snapshotChanged)
    def diagnosticMessage(self):
        if self._cpu_overlay:
            return f"LiDAR overlay · max {self._cpu_max_points:,} prepared points"
        return "Renderer disabled (software diagnostic; no OpenGL/FBO overlay)" if self._renderer_disabled else ""
    @Property(bool, notify=snapshotChanged)
    def cpuOverlayEnabled(self): return self._cpu_overlay
    @Property(str, notify=snapshotChanged)
    def datasetLabel(self): return self._dataset_label
    @Property(str, notify=snapshotChanged)
    def profileLabel(self): return self._profile_label
    @Property(str, notify=snapshotChanged)
    def sequenceLabel(self): return self._sequence_label
    @Property(str, notify=snapshotChanged)
    def selectedCamera(self): return self._camera
    @Property("QStringList", notify=snapshotChanged)
    def cameraNames(self): return self._camera_names
    @Property("QVariantList", notify=snapshotChanged)
    def cameraModels(self):
        return [
            {
                "cameraId": camera_id,
                "imageUrl": self._camera_image_url(camera_id),
                "overlayUrl": self._preview_overlay_url(camera_id),
                "selected": camera_id == self._camera,
                **self._camera_details.get(camera_id, {}),
            }
            for camera_id in self._camera_names
        ]
    @Property(int, notify=snapshotChanged)
    def frameIndex(self): return self._frame
    @Property(int, notify=snapshotChanged)
    def frameCount(self): return self._frame_count
    @Property(str, notify=snapshotChanged)
    def timestampLabel(self): return f"06:42.{int((self._timestamp % 1) * 1000):03d}"
    @Property(bool, notify=snapshotChanged)
    def playing(self): return self._playing
    @Property(bool, notify=snapshotChanged)
    def playbackWaiting(self): return self._playback_waiting
    @Property(float, notify=snapshotChanged)
    def playbackSpeed(self): return self._playback_speed
    @Property(int, notify=snapshotChanged)
    def bufferedFrameCount(self): return self._buffered_count
    @Property(int, notify=snapshotChanged)
    def bufferTarget(self): return self._buffer_target
    @Property(bool, notify=snapshotChanged)
    def buffering(self): return self._buffered_count < self._buffer_target
    @Property(bool, notify=snapshotChanged)
    def dirty(self): return self._dirty
    @Property(bool, notify=snapshotChanged)
    def allSynchronized(self): return self._all_synchronized
    @Property(str, notify=snapshotChanged)
    def imuStatus(self): return "Pose available" if self._lidar_scene_frame_token is not None else "Unavailable"
    @Property(str, notify=snapshotChanged)
    def projectionStatus(self): return self._projection_status
    @Property(str, notify=snapshotChanged)
    def projectionMessage(self): return self._projection_message
    @Property(str, notify=snapshotChanged)
    def imageUrl(self):
        return f"image://workspace/current?camera={self._camera}&frame={self._frame}" if self._image_available else ""
    def _camera_image_url(self, camera_id):
        return (
            f"image://workspace/current?camera={camera_id}&frame={self._frame}"
            if camera_id in self._available_cameras else ""
        )

    @Slot(str, result=str)
    def cameraImageUrl(self, camera_id):
        return self._camera_image_url(str(camera_id))
    @Property(str, notify=snapshotChanged)
    def lidarViewUrl(self):
        return f"image://workspace-lidar/current?revision={self._lidar_scene_revision}" if self._lidar_scene_image is not None else ""
    @Property(int, notify=snapshotChanged)
    def lidarPointCount(self): return self._lidar_point_count
    @Property(int, notify=snapshotChanged)
    def lidarRenderedCount(self): return self._lidar_rendered_count
    @Property(str, notify=snapshotChanged)
    def overlayUrl(self):
        return f"image://workspace-overlay/current?revision={self._cpu_overlay_revision}" if self._cpu_overlay_image is not None else ""
    @Property(str, notify=snapshotChanged)
    def originalOverlayUrl(self):
        return f"image://workspace-overlay/original?revision={self._cpu_original_overlay_revision}" if self._cpu_original_overlay_image is not None else ""
    @Property(bool, notify=snapshotChanged)
    def previewProjectionEnabled(self): return self._preview_projection_enabled
    def _preview_overlay_url(self, camera_id):
        image = self._preview_overlay_images.get(camera_id)
        return (
            f"image://workspace-overlay/preview-{camera_id}?revision={self._preview_overlay_revisions.get(camera_id, 0)}"
            if self._preview_projection_enabled and image is not None else ""
        )

    @Slot(str, result=str)
    def previewOverlayUrl(self, camera_id):
        return self._preview_overlay_url(str(camera_id))
    @Property(str, notify=snapshotChanged)
    def overlayColoring(self): return str(self._overlay_settings["coloring"])
    @Property(float, notify=snapshotChanged)
    def overlayPointSize(self): return float(self._overlay_settings["pointSizePx"])
    @Property(float, notify=snapshotChanged)
    def overlayOpacity(self): return float(self._overlay_settings["opacity"])
    @Property(float, notify=snapshotChanged)
    def overlayDepthMin(self): return float(self._overlay_settings["depthMinMetres"])
    @Property(float, notify=snapshotChanged)
    def overlayDepthMax(self): return float(self._overlay_settings["depthMaxMetres"])
    @Property(str, notify=snapshotChanged)
    def overlayDensity(self): return str(self._overlay_settings["renderDensity"])
    @Property(bool, notify=snapshotChanged)
    def overlayOnlyPointsInImage(self): return bool(self._overlay_settings["onlyPointsInImage"])
    @Property(bool, notify=snapshotChanged)
    def processing(self): return self._processing
    @Property(bool, notify=snapshotChanged)
    def safePreparationMode(self): return self._safe_preparation_mode
    @Property(bool, notify=snapshotChanged)
    def rendererReady(self): return self._renderer_ready
    @Property(str, notify=snapshotChanged)
    def exportStatus(self): return self._export_status
    @Property(str, notify=snapshotChanged)
    def exportPath(self): return self._export_path
    @Property(int, notify=snapshotChanged)
    def exportSuccessId(self): return self._export_success_id
    @Property(bool, notify=snapshotChanged)
    def canUndo(self): return self._can_undo
    @Property(bool, notify=snapshotChanged)
    def canRedo(self): return self._can_redo
    @Property(bool, notify=snapshotChanged)
    def intrinsicsEnabled(self): return self._intrinsics_enabled
    @Property(bool, notify=snapshotChanged)
    def overlayOpen(self): return self._overlay_open
    @Property(bool, notify=snapshotChanged)
    def comparisonOpen(self): return self._comparison_open
    @Property(bool, notify=snapshotChanged)
    def exportOpen(self): return self._export_open

    def overlay_array(self, original=False):
        return self._cpu_original_overlay_image if original else self._cpu_overlay_image

    def preview_overlay_array(self, camera_id):
        return self._preview_overlay_images.get(camera_id)

    def image_array(self, camera_id=None):
        if self.controller is None:
            return None
        snapshot = self.controller.snapshot()
        frame = snapshot.frame_bundle or snapshot.displayed_bundle
        if frame is None:
            return None
        camera = frame.cameras.get(camera_id or self._camera)
        return None if camera is None else camera.image

    def lidar_scene_array(self):
        return self._lidar_scene_image

    def _push_lidar_scene(self, *, quick=False):
        if self._lidar_points is None:
            self._lidar_scene_image = None
            self._lidar_point_count = 0
            self._lidar_rendered_count = 0
            return
        from ..rendering import render_lidar_scene
        result = render_lidar_scene(self._lidar_points, **{
            "yaw_degrees": self._lidar_view["yaw"],
            "pitch_degrees": self._lidar_view["pitch"],
            "zoom": self._lidar_view["zoom"],
            "pan_x": self._lidar_view["pan_x"],
            "pan_y": self._lidar_view["pan_y"],
            "width": self._lidar_view["width"],
            "height": self._lidar_view["height"],
            "interactive": bool(quick or self._lidar_view["interactive"]),
            "pivot_xyz": self._lidar_pivot,
            "anchor_x": self._lidar_view["anchor_x"],
            "anchor_y": self._lidar_view["anchor_y"],
            "base_scale_ratio": self._lidar_base_scale_ratio,
        })
        self._lidar_scene_image = result.rgb
        self._lidar_point_count = result.point_count
        self._lidar_rendered_count = result.rendered_count
        self._lidar_base_scale_ratio = result.base_scale_ratio
        if result.pick_indices is not None:
            self._lidar_pick_indices = result.pick_indices
        self._lidar_scene_revision += 1

    def _refine_lidar_scene(self):
        if self._lidar_points is None:
            return
        self._push_lidar_scene()
        self._emit()

    def value(self, key): return self._values[key]
    @Slot(str, result=float)
    def parameter(self, key): return float(self._values.get(key, 0.0))

    def _buffer_adapter(self):
        return None if self.controller is None else self.controller.adapter

    def _refresh_buffer_status(self, emit=True):
        adapter = self._buffer_adapter()
        target = 0 if adapter is None else int(getattr(adapter, "buffer_target", len(adapter)))
        count = target if adapter is None else int(getattr(adapter, "buffered_count", target))
        changed = count != self._buffered_count or target != self._buffer_target
        self._buffered_count, self._buffer_target = count, target
        pending = self._pending_seek
        if count >= target and pending is None:
            self._buffer_timer.stop()
        if pending is not None and self._frame_ready(pending):
            self._pending_seek = None
            self.seek(pending)
            return
        if changed and emit:
            self._emit()

    def _frame_ready(self, index):
        adapter = self._buffer_adapter()
        ready = getattr(adapter, "is_ready", None)
        return True if not callable(ready) else bool(ready(index))

    def _schedule_next_frame(self):
        if not self._playing:
            return
        next_index = self._frame + 1
        if next_index >= self._frame_count:
            self._stop_playback()
            return
        if not self._frame_ready(next_index):
            self._playback_waiting = True
            self._playback_timer.start(50)
            self._emit()
            return
        adapter = self._buffer_adapter()
        try:
            current = adapter.frame(self._frame)
            following = adapter.frame(next_index)
        except Exception as exc:
            self._show_frame_error(next_index, exc)
            return
        delta = float(following.timestamp - current.timestamp)
        if not np.isfinite(delta) or delta <= 0:
            delta = 0.1
        self._playback_waiting = False
        self._playback_timer.start(int(np.clip(round(delta * 1000 / self._playback_speed), 20, 1000)))

    @Slot()
    def _advance_playback(self):
        if not self._playing:
            return
        next_index = self._frame + 1
        if next_index >= self._frame_count:
            self._stop_playback()
            return
        if not self._frame_ready(next_index):
            self._playback_waiting = True
            self._playback_timer.start(50)
            self._emit()
            return
        self.seek(next_index)

    def _stop_playback(self):
        self._playback_timer.stop()
        self._playback_waiting = False
        self._playing = False
        self._dispatch("pause")
        self._emit()

    def _show_frame_error(self, index, error):
        if self._playing:
            self._stop_playback()
        self._pending_seek = None
        self._processing = False
        self._projection_status = "error"
        self._projection_message = f"Frame {int(index) + 1} failed to load: {error}"
        self._emit()

    @Slot()
    def togglePlay(self):
        if self._playing:
            self._stop_playback()
            return
        if self._frame >= self._frame_count - 1:
            self.seek(0)
        self._playing = True
        self._dispatch("play")
        self._schedule_next_frame()
        self._emit()

    @Slot(int, result=bool)
    def frameBuffered(self, frame):
        frame = max(0, min(self._frame_count - 1, int(frame)))
        adapter = self._buffer_adapter()
        was_loaded = getattr(adapter, "was_loaded", None)
        return self._frame_ready(frame) if not callable(was_loaded) else bool(was_loaded(frame))

    @Slot(int)
    def selectFrame(self, frame):
        if self._playing:
            self._stop_playback()
        self.seek(frame)

    @Slot(int)
    def seek(self, frame):
        self._playback_timer.stop()
        target = max(0, min(self._frame_count - 1, int(frame)))
        if not self._frame_ready(target):
            self._pending_seek = target
            self._processing = True
            if not self._buffer_timer.isActive():
                self._buffer_timer.start()
            self._emit()
            return
        self._pending_seek = None
        self._frame = target
        self._processing = True
        try:
            self._dispatch("seek", self._frame)
        except Exception as exc:
            self._show_frame_error(target, exc)
            return
        self._processing = False
        self._refresh_buffer_status(emit=False)
        if self._playing:
            self._schedule_next_frame()
        self._emit()
    @Slot(str)
    def selectCamera(self, camera):
        if camera in self._camera_names:
            self._camera = camera; self._dispatch("select_camera", camera); self._emit()
    @Slot(float, float, result=bool)
    def beginLidarNavigation(self, normalized_x, normalized_y):
        """Select the nearest settled-render point as orbit/zoom pivot."""
        if self._lidar_pick_indices is None or self._lidar_points is None:
            return False
        pick = self._lidar_pick_indices
        height, width = pick.shape
        x = int(np.clip(round(float(normalized_x) * (width - 1)), 0, width - 1))
        y = int(np.clip(round(float(normalized_y) * (height - 1)), 0, height - 1))
        radius = max(5, int(round(min(width, height) * 0.025)))
        x0, x1 = max(0, x - radius), min(width, x + radius + 1)
        y0, y1 = max(0, y - radius), min(height, y + radius + 1)
        window = pick[y0:y1, x0:x1]
        rows, columns = np.nonzero(window >= 0)
        if not len(rows):
            return False
        distances = (columns + x0 - x) ** 2 + (rows + y0 - y) ** 2
        nearest = int(np.argmin(distances))
        source_index = int(window[rows[nearest], columns[nearest]])
        point = np.asarray(self._lidar_points[source_index, :3], dtype=np.float32)
        if not np.isfinite(point).all():
            return False
        self._lidar_pivot = np.array(point, copy=True)
        # The pivot remains beneath the cursor; pan is applied separately.
        self._lidar_view["anchor_x"] = float(normalized_x) - self._lidar_view["pan_x"]
        self._lidar_view["anchor_y"] = float(normalized_y) - self._lidar_view["pan_y"]
        return True

    @Slot(float, float, float, float, float)
    @Slot(float, float, float, float, float, float, float, bool)
    def setLidarView(self, yaw, pitch, zoom, pan_x, pan_y, width=1280, height=720, interactive=False):
        self._lidar_view.update({
            "yaw": float(yaw) % 360.0,
            "pitch": float(np.clip(pitch, 10.0, 88.0)),
            "zoom": float(np.clip(zoom, 0.5, 12.0)),
            "pan_x": float(np.clip(pan_x, -2.0, 2.0)),
            "pan_y": float(np.clip(pan_y, -2.0, 2.0)),
            "width": int(np.clip(round(width), 320, 1920)),
            "height": int(np.clip(round(height), 180, 1080)),
            "interactive": bool(interactive),
        })
        self._push_lidar_scene()
        self._emit()
    @Slot()
    def beginCalibrationChange(self): self._dispatch("begin_calibration_change")
    @Slot()
    def endCalibrationChange(self): self._dispatch("end_calibration_change")
    @Slot()
    def undo(self): self._dispatch("undo")
    @Slot()
    def redo(self): self._dispatch("redo")
    @Slot(str, float)
    def adjustTranslation(self, axis, value): self._set_value(axis, value); self._dispatch("adjust_translation", axis, value)
    @Slot(str, float)
    def adjustRotation(self, axis, value): self._set_value(axis, value); self._dispatch("adjust_rotation", axis, value)
    @Slot(str, float)
    def setIntrinsic(self, name, value):
        self._set_value(name, float(value)); self._dispatch("set_intrinsic", name, value); self._emit()
    def _set_value(self, key, value): self._values[key] = float(value); self._dirty = True; self._emit()
    @Slot(bool)
    def setIntrinsicsEnabled(self, value): self._intrinsics_enabled = value; self._dispatch("set_intrinsics_enabled", value); self._emit()
    @Slot(str)
    def resetParameter(self, key):
        defaults = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0, "fx": 721.58, "fy": 721.54, "cx": 609.56, "cy": 172.85}
        if key in defaults: self._values[key] = defaults[key]; self._dirty = True; self._dispatch("reset_parameter", key); self._emit()
    @Slot()
    def resetExtrinsics(self): self._dispatch("reset_extrinsics")
    @Slot()
    def resetIntrinsics(self): self._dispatch("reset_intrinsics")
    @Slot()
    def resetAll(self):
        for key in ("x", "y", "z", "roll", "pitch", "yaw"): self._values[key] = 0
        self._dirty = True; self._dispatch("reset_all"); self._emit()
    def _cpu_out_of_envelope(self, data):
        """Conservatively detect when the prepared 5° candidate envelope is stale."""
        if self._cpu_prepared_transform is None:
            return True
        model = data.projection_model
        current_transform = np.asarray(model["transform"], dtype=float)
        current_k = np.asarray(model.get("intrinsics"), dtype=float)
        current_t = np.asarray(model.get("rectified_translation"), dtype=float)
        # Rotation/FOV changes over the documented 5° margin require refresh.
        relative = current_transform[:3, :3] @ self._cpu_prepared_transform[:3, :3].T
        angle = np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
        focal_change = np.max(np.abs(current_k - self._cpu_prepared_intrinsics) / np.maximum(np.abs(self._cpu_prepared_intrinsics), 1.0))
        # Translation has no single depth-independent angular bound; use a
        # conservative 0.25 m guard rather than silently cropping candidates.
        translation_change = np.linalg.norm(current_transform[:3, 3] - self._cpu_prepared_transform[:3, 3])
        baseline_change = np.linalg.norm(current_t - self._cpu_prepared_translation)
        return bool(angle > np.deg2rad(5.0) or focal_change > 0.10 or translation_change > 0.25 or baseline_change > 0.25)

    def _push_preview_overlays(self, snapshot):
        """Render heavily bounded working projections for main-view cameras."""
        if not self._preview_projection_enabled or snapshot.renderer_input is None:
            return
        from ..rendering import OverlaySettings as RenderOverlaySettings, render_cpu_overlay

        data = snapshot.renderer_input
        settings = snapshot.overlay
        camera_models = data.projection_model.get("camera_models", {})
        render_settings = RenderOverlaySettings(
            coloring=settings.coloring,
            point_size_px=max(1.0, min(float(settings.point_size_px), 3.0)),
            opacity=settings.opacity,
            depth_min_metres=settings.depth_min_metres,
            depth_max_metres=settings.depth_max_metres,
            render_density=settings.render_density,
            only_points_in_image=True,
        )
        for camera_id in self._camera_names:
            model = camera_models.get(camera_id)
            if model is None:
                continue
            token = (
                data.frame_index, data.calibration_revision, camera_id,
                settings.coloring, render_settings.point_size_px, settings.opacity,
                settings.depth_min_metres, settings.depth_max_metres,
                self._preview_max_points,
            )
            if token == self._preview_overlay_tokens[camera_id]:
                continue
            projection = SimpleNamespace(camera_id=camera_id, projection_model=model)
            result = render_cpu_overlay(
                data.points_xyzi, projection, render_settings,
                max_points=self._preview_max_points,
            )
            self._preview_overlay_images[camera_id] = result.rgba
            self._preview_overlay_revisions[camera_id] += 1
            self._preview_overlay_tokens[camera_id] = token

    def _push_cpu_original(self, data, settings, render_settings):
        """Render immutable dataset calibration only while comparison is visible."""
        if not self._comparison_open:
            return
        from ..rendering import CandidateEnvelope, prepare_renderer_input, render_cpu_overlay

        model = dict(data.projection_model)
        model["transform"] = model.get("original_transform", model["transform"])
        model["intrinsics"] = model.get("original_intrinsics", model["intrinsics"])
        original_data = SimpleNamespace(
            frame_index=data.frame_index,
            camera_id=data.camera_id,
            points_xyzi=data.points_xyzi,
            projection_model=model,
            overlay=settings,
            calibration_revision=-1,
        )
        prep_token = (
            data.frame_index, data.camera_id,
            settings.render_density, settings.voxel_size_metres,
        )
        if prep_token != self._cpu_original_prepared_token:
            prepared = prepare_renderer_input(
                original_data, CandidateEnvelope(), cache=self._prep_cache
            )
            self._cpu_original_prepared_points = prepared.points
            self._cpu_original_prepared_token = prep_token
            self._cpu_original_render_token = None
        render_token = prep_token + (
            settings.coloring, settings.point_size_px, settings.opacity,
            settings.depth_min_metres, settings.depth_max_metres,
        )
        if render_token == self._cpu_original_render_token:
            return
        result = render_cpu_overlay(
            self._cpu_original_prepared_points,
            original_data,
            render_settings,
            max_points=self._cpu_max_points,
        )
        self._cpu_original_overlay_image = result.rgba
        self._cpu_original_overlay_revision += 1
        self._cpu_original_render_token = render_token

    def _push_cpu_playback_overlay(self, data, settings):
        """Bounded direct projection for smooth, frame-aligned CPU playback."""
        from ..rendering import OverlaySettings as RenderOverlaySettings, render_cpu_overlay

        render_settings = RenderOverlaySettings(
            coloring=settings.coloring,
            point_size_px=max(1.0, min(float(settings.point_size_px), 3.0)),
            opacity=settings.opacity,
            depth_min_metres=settings.depth_min_metres,
            depth_max_metres=settings.depth_max_metres,
            render_density="light",
            only_points_in_image=settings.only_points_in_image,
        )
        result = render_cpu_overlay(
            data.points_xyzi, data, render_settings, max_points=self._preview_max_points
        )
        self._cpu_overlay_image = result.rgba
        self._cpu_overlay_revision += 1
        if self._comparison_open:
            model = dict(data.projection_model)
            model["transform"] = model.get("original_transform", model["transform"])
            model["intrinsics"] = model.get("original_intrinsics", model["intrinsics"])
            original_data = SimpleNamespace(camera_id=data.camera_id, projection_model=model)
            original = render_cpu_overlay(
                data.points_xyzi, original_data, render_settings, max_points=self._preview_max_points
            )
            self._cpu_original_overlay_image = original.rgba
            self._cpu_original_overlay_revision += 1
        self._projection_status = "ready"
        self._projection_message = ""

    def _push_cpu_overlay(self, snapshot):
        """Prepare candidates per frame/profile and refresh beyond 5° coverage."""
        data = snapshot.renderer_input
        if data is None:
            self._cpu_overlay_image = None
            self._projection_status = "unavailable"
            self._projection_message = "LiDAR overlay unavailable: no renderer input"
            return
        controller_status = snapshot.projection_status
        if controller_status == "unavailable":
            self._cpu_overlay_image = None
            self._projection_status = controller_status
            self._projection_message = snapshot.projection_message or "Projection unavailable"
            return
        if controller_status == "stale":
            # Keep the last valid layer visible; the controller owns refresh timing.
            self._projection_status = "stale"
            self._projection_message = snapshot.projection_message or "Refreshing CPU overlay coverage"
            return
        try:
            from ..rendering import CandidateEnvelope, OverlaySettings as RenderOverlaySettings
            from ..rendering import PointPreparationCache, prepare_renderer_input, render_cpu_overlay
            settings = snapshot.overlay
            if self._playing:
                self._push_cpu_playback_overlay(data, settings)
                return
            # The conservative 5° envelope is revisioned by calibration input;
            # rebuild candidates whenever geometry changes so a large perturbation
            # cannot reuse an out-of-envelope CPU buffer.
            token = (data.frame_index, data.camera_id, settings.render_density, settings.voxel_size_metres)
            out_of_envelope = self._cpu_out_of_envelope(data)
            if token != self._cpu_prepared_token or out_of_envelope:
                if self._prep_cache is None:
                    self._prep_cache = PointPreparationCache(max_entries=4)
                if token == self._cpu_prepared_token and out_of_envelope:
                    self._cpu_envelope_revision += 1
                    self._projection_status = "stale"
                    self._projection_message = "Candidate envelope exceeded; refreshing overlay"
                envelope = CandidateEnvelope(revision=self._cpu_envelope_revision)
                prepared = prepare_renderer_input(data, envelope, cache=self._prep_cache)
                self._cpu_prepared_points = prepared.points
                self._cpu_prepared_token = token
                model = data.projection_model
                self._cpu_prepared_transform = np.array(model["transform"], dtype=float, copy=True)
                self._cpu_prepared_intrinsics = np.array(model["intrinsics"], dtype=float, copy=True)
                self._cpu_prepared_translation = np.array(model["rectified_translation"], dtype=float, copy=True)
            render_settings = RenderOverlaySettings(
                coloring=settings.coloring, point_size_px=settings.point_size_px,
                opacity=settings.opacity, depth_min_metres=settings.depth_min_metres,
                depth_max_metres=settings.depth_max_metres,
                render_density=settings.render_density,
                only_points_in_image=settings.only_points_in_image,
            )
            result = render_cpu_overlay(
                self._cpu_prepared_points, data, render_settings, max_points=self._cpu_max_points
            )
            self._cpu_overlay_image = result.rgba
            self._cpu_overlay_revision += 1
            self._push_cpu_original(data, settings, render_settings)
            # Never hide controller-level unavailable/stale status behind a
            # successful CPU rasterization.
            if controller_status in {"unavailable", "stale"}:
                self._projection_status = controller_status
                self._projection_message = snapshot.projection_message or "Projection unavailable"
            else:
                self._projection_status = "ready"
                self._projection_message = ""
        except Exception as exc:
            self._cpu_overlay_image = None
            self._projection_status = "unavailable"
            self._projection_message = f"LiDAR overlay unavailable: {exc}"

    @Slot(QObject)
    def attachRenderer(self, item):
        """Connect the registered Qt scene-graph item to opaque renderer input."""
        self._renderer_item = item
        print(f"LiDAR renderer attachment: rendererReady={self._renderer_ready} renderer_input={self._renderer_ready}", flush=True)
        if not getattr(item, "supports_points", True):
            self._projection_status = "unavailable"
            self._projection_message = getattr(item, "backend_message", "GPU overlay unavailable")
            item.mark_unavailable(self._projection_message)
            self._emit()
            return
        if self.controller is not None:
            self._push_renderer(self.controller.snapshot())

    def _push_renderer(self, snapshot):
        if self._renderer_item is None:
            return
        if snapshot.renderer_input is None:
            self._renderer_pending_token = None
            if snapshot.projection_status == "unavailable":
                self._renderer_item.mark_unavailable(snapshot.projection_message or "Projection unavailable")
            return
        try:
            from ..rendering import (CandidateEnvelope, OverlaySettings as RenderOverlaySettings,
                                     PointPreparationCache, PreparationKey, ProjectionUniforms,
                                     prepare_renderer_input)
            data = snapshot.renderer_input
            model = data.projection_model
            settings = snapshot.overlay
            uniforms = ProjectionUniforms(
                model["transform"], model["rectification_00"], model["projection_matrix"],
                tuple(model["image_size"]), intrinsics=model.get("intrinsics"),
                rectified_translation=model.get("rectified_translation"),
                settings=RenderOverlaySettings(
                    coloring=settings.coloring, point_size_px=settings.point_size_px,
                    opacity=settings.opacity, depth_min_metres=settings.depth_min_metres,
                    depth_max_metres=settings.depth_max_metres,
                    render_density=settings.render_density,
                    only_points_in_image=settings.only_points_in_image,
                ),
            )
            points_token = (data.frame_index, data.camera_id, settings.render_density,
                            settings.voxel_size_metres)
            self._renderer_uniforms = uniforms
            # Calibration/overlay edits update uniforms without re-uploading points.
            self._renderer_item.set_projection_uniforms(uniforms)
            if points_token != self._renderer_points_token and points_token != self._renderer_pending_token:
                if self._prep_cache is None:
                    self._prep_cache = PointPreparationCache(max_entries=4)
                cache_key = PreparationKey(
                    int(data.frame_index), str(data.camera_id), 0,
                    float(settings.voxel_size_metres),
                    int(getattr(data, "calibration_revision", 0)),
                )
                cached = self._prep_cache.get(cache_key)
                if cached is not None:
                    # Cache hits publish immediately; misses retain the prior VBO.
                    self._renderer_item.set_points(cached.points)
                    self._renderer_points_token = points_token
                    self._renderer_pending_token = None
                    self._processing = False
                    self._projection_message = ""
                else:
                    self._renderer_pending_token = points_token
                    self._processing = True
                    self._projection_status = "preparing"
                    self._renderer_item.mark_preparing()
                    if self._prepare_synchronously:
                        # Windows safety fallback: keep NumPy out of Qt's pooled
                        # worker thread. This can briefly block the UI on a cache
                        # miss, but avoids the observed native heap corruption.
                        try:
                            result = prepare_renderer_input(
                                data, CandidateEnvelope(), cache=self._prep_cache
                            )
                        except Exception as exc:
                            result = exc
                        self._on_prepared(points_token, result)
                    else:
                        task = _PreparationTask(points_token, data, self._prep_cache)
                        task.setAutoDelete(False)
                        self._prep_tasks[points_token] = task
                        task.signals.finished.connect(self._on_prepared)
                        self._prep_pool.start(task)
        except (ImportError, RuntimeError, TypeError, ValueError, KeyError) as exc:
            self._projection_status = "unavailable"
            self._projection_message = f"Renderer unavailable: {exc}"
            print(f"LiDAR FBO/overlay error: {exc}", flush=True)

    @Slot(object, object)
    def _on_prepared(self, token, result):
        # Release the retained task on both accepted and stale-result paths.
        self._prep_tasks.pop(token, None)
        if token != self._renderer_pending_token or self._renderer_item is None:
            return
        self._renderer_pending_token = None
        if isinstance(result, Exception):
            self._projection_status = "unavailable"
            self._projection_message = f"Candidate preparation failed: {result}"
            self._renderer_item.mark_unavailable(self._projection_message)
            self._emit()
            return
        self._renderer_item.set_points(result.points)
        self._renderer_points_token = token
        self._processing = False
        self._projection_status = "ready"
        self._projection_message = ""
        if self._renderer_uniforms is not None:
            self._renderer_item.set_projection_uniforms(self._renderer_uniforms)
        self._renderer_item.update()
        self._emit()

    @Slot()
    def shutdown(self):
        """Wait for worker completion before Qt tears down the bridge/context."""
        self._playback_timer.stop()
        self._buffer_timer.stop()
        self._lidar_refine_timer.stop()
        if self._prep_pool is not None:
            self._prep_pool.waitForDone()
        self._prep_tasks.clear()
        adapter = self._buffer_adapter()
        shutdown = getattr(adapter, "shutdown", None)
        if callable(shutdown):
            shutdown()

    @Slot(str, float)
    @Slot(str, str)
    @Slot(str, bool)
    def setOverlaySetting(self, name, value):
        name = str(name)
        self._overlay_settings[name] = value
        controller_names = {"pointSizePx": "point_size_px", "depthMinMetres": "depth_min_metres",
                            "depthMaxMetres": "depth_max_metres", "renderDensity": "render_density",
                            "onlyPointsInImage": "only_points_in_image"}
        self._dispatch("set_overlay_setting", controller_names.get(name, name), value)
        self._emit()
    @Slot()
    def toggleOverlay(self): self._overlay_open = not self._overlay_open; self._emit()
    @Slot(bool)
    def setCalibrationActive(self, active):
        self._calibration_active = bool(active)
        if self._calibration_active and self._cpu_overlay and self.controller is not None:
            self._push_cpu_overlay(self.controller.snapshot())
        self._emit()
    @Slot()
    def togglePreviewProjection(self):
        self._preview_projection_enabled = not self._preview_projection_enabled
        if self._preview_projection_enabled and self.controller is not None:
            self._push_preview_overlays(self.controller.snapshot())
        self._emit()
    @Slot()
    def toggleComparison(self):
        self._comparison_open = not self._comparison_open
        if self._comparison_open and self._cpu_overlay and self.controller is not None:
            self._push_cpu_overlay(self.controller.snapshot())
        self._emit()
    @Slot()
    def requestExport(self): self._export_open = True; self._dispatch("request_export"); self._emit()
    @Slot()
    def cancelExport(self): self._export_open = False; self._dispatch("cancel_export"); self._emit()
    @Slot()
    @Slot(QUrl)
    @Slot(str)
    def confirmExport(self, path=None):
        if isinstance(path, QUrl):
            path = path.toLocalFile()
        self._export_open = False
        if path:
            self._dispatch("confirm_export", str(path))
        else:
            self._dispatch("confirm_export")
        self._emit()
    @Slot()
    def reviewCalibration(self): self._projection_status = "ready"; self._projection_message = ""; self._emit()
