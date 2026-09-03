from __future__ import annotations
import os
import sys
from pathlib import Path
from urllib.parse import parse_qs
from PySide6.QtGui import QGuiApplication, QImage
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickImageProvider
from .bridge import WorkspaceBridge


class WorkspaceOverlayImageProvider(QQuickImageProvider):
    """Supplies calibration and camera-preview LiDAR projection layers."""
    def __init__(self, bridge):
        super().__init__(QQuickImageProvider.Image)
        self.bridge = bridge

    def requestImage(self, image_id, size, requested_size):  # noqa: N802 - Qt API
        layer = image_id.split("?", 1)[0]
        if layer.startswith("preview-"):
            image = self.bridge.preview_overlay_array(layer.removeprefix("preview-"))
        else:
            image = self.bridge.overlay_array(original=layer == "original")
        if image is None:
            return QImage()
        height, width = image.shape[:2]
        result = QImage(image.data, width, height, image.strides[0], QImage.Format_RGBA8888).copy()
        if size is not None:
            size.setWidth(width)
            size.setHeight(height)
        return result


class WorkspaceLidarImageProvider(QQuickImageProvider):
    """Supplies the navigable full-frame software LiDAR scene."""
    def __init__(self, bridge):
        super().__init__(QQuickImageProvider.Image)
        self.bridge = bridge

    def requestImage(self, image_id, size, requested_size):  # noqa: N802 - Qt API
        image = self.bridge.lidar_scene_array()
        if image is None:
            return QImage()
        height, width = image.shape[:2]
        result = QImage(image.data, width, height, image.strides[0], QImage.Format_RGB888).copy()
        if size is not None:
            size.setWidth(width)
            size.setHeight(height)
        return result


class WorkspaceImageProvider(QQuickImageProvider):
    """Supplies the currently selected rectified camera image without file I/O."""
    def __init__(self, bridge):
        super().__init__(QQuickImageProvider.Image)
        self.bridge = bridge

    def requestImage(self, image_id, size, requested_size):  # noqa: N802 - Qt API
        """Return one QImage; PySide6 writes image dimensions through ``size``."""
        query = image_id.split("?", 1)[1] if "?" in image_id else ""
        camera_id = parse_qs(query).get("camera", [None])[0]
        image = self.bridge.image_array(camera_id)
        if image is None:
            return QImage()
        height, width = image.shape[:2]
        result = QImage(image.data, width, height, image.strides[0], QImage.Format_RGB888).copy()
        if size is not None:
            size.setWidth(width)
            size.setHeight(height)
        return result


def _controller_from_environment():
    """Build the real Qt-free controller when a KITTI workspace is available."""
    from ..app import BufferedFrameAdapter, WorkspaceController
    from ..kitti import KittiAdapter
    project_root = Path(__file__).resolve().parents[3]
    dataset_root = project_root.parent / "kitti"
    calibration = Path(os.environ.get("LIDAR_CALIBRATION_PATH", dataset_root / "2011_09_26_calib/2011_09_26"))
    sequence = Path(os.environ.get("LIDAR_SEQUENCE_PATH", dataset_root / "2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync"))
    if not calibration.exists():
        raise FileNotFoundError(f"KITTI calibration directory not found: {calibration}")
    if not sequence.exists():
        raise FileNotFoundError(f"KITTI sequence directory not found: {sequence}")
    frame_limit = max(1, int(os.environ.get("LIDAR_CALIBRATOR_BUFFER_FRAMES", "10")))
    preload_count = min(frame_limit, max(1, int(os.environ.get("LIDAR_CALIBRATOR_PRELOAD_FRAMES", "2"))))
    adapter = BufferedFrameAdapter(
        KittiAdapter(calibration, sequence), frame_limit=frame_limit, preload_count=preload_count
    )
    return WorkspaceController(adapter)


def run_controller(
    controller,
    *,
    renderer_mode="cpu",
    window_title=None,
    startup_error=None,
) -> int:
    """Run one controller in an owned application or a blocking nested event loop."""
    os.environ.setdefault("QT_QUICK_CONTROLS_STYLE", "Material")
    os.environ.setdefault("QT_QUICK_CONTROLS_MATERIAL_THEME", "Dark")
    os.environ.setdefault("QT_QUICK_CONTROLS_MATERIAL_ACCENT", "#54b7ff")
    renderer_mode = str(renderer_mode).strip().lower()
    if renderer_mode in {"off", "0"}:
        renderer_mode = "disabled"
    if renderer_mode not in {"production", "disabled", "cpu"}:
        print(f"Unknown renderer mode {renderer_mode!r}; using production", flush=True)
        renderer_mode = "production"
    renderer_disabled = renderer_mode == "disabled"
    cpu_overlay = renderer_mode == "cpu"
    if renderer_disabled or cpu_overlay:
        # Must be forced before QGuiApplication/scenegraph initialization.
        os.environ["QT_QUICK_BACKEND"] = "software"
    print(f"QT_QUICK_BACKEND={os.environ.get('QT_QUICK_BACKEND', 'default')}", flush=True)
    print(f"LiDAR renderer mode: {renderer_mode}", flush=True)
    app = QGuiApplication.instance()
    owns_application = app is None
    if app is None:
        app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()
    bridge = WorkspaceBridge(
        controller, startup_error=startup_error,
        renderer_disabled=renderer_disabled, cpu_overlay=cpu_overlay,
    )
    print(f"LiDAR candidate preparation mode: {'sync (degraded)' if bridge.safePreparationMode else 'async'}", flush=True)
    if renderer_disabled:
        print("LiDAR renderer diagnostic: disabled (software backend; no point overlay)", flush=True)
    elif cpu_overlay:
        print("LiDAR renderer diagnostic: CPU software overlay (bounded prepared points)", flush=True)
    if owns_application:
        app.aboutToQuit.connect(bridge.shutdown)
    engine.addImageProvider("workspace", WorkspaceImageProvider(bridge))
    engine.addImageProvider("workspace-overlay", WorkspaceOverlayImageProvider(bridge))
    engine.addImageProvider("workspace-lidar", WorkspaceLidarImageProvider(bridge))
    if not renderer_disabled and not cpu_overlay:
        try:
            from ..rendering import register_qml_type
            register_qml_type()
            print("LiDAR renderer registration: SceneGraphOverlayItem registered", flush=True)
        except (ImportError, TypeError, RuntimeError) as exc:
            print(f"LiDAR renderer registration failed: {exc}", flush=True)
    engine.rootContext().setContextProperty("workspaceBridge", bridge)
    qml_name = "MainDisabled.qml" if renderer_disabled or cpu_overlay else "Main.qml"
    engine.load(str(Path(__file__).with_name("qml") / qml_name))
    if not engine.rootObjects():
        bridge.shutdown()
        return 1
    root = engine.rootObjects()[0]
    if window_title is not None and hasattr(root, "setTitle"):
        root.setTitle(str(window_title))
    if owns_application:
        result = app.exec()
    else:
        from PySide6.QtCore import QEventLoop
        loop = QEventLoop()
        root.visibleChanged.connect(lambda: loop.quit() if not root.isVisible() else None)
        root.destroyed.connect(loop.quit)
        loop.exec()
        result = 0
    bridge.shutdown()
    return int(result)


def main() -> int:
    if len(sys.argv) > 1 and Path(sys.argv[1]).suffix.lower() == ".mcap":
        from ..api import CalibrationConfig, launch_calibrator
        renderer_mode = os.environ.get("LIDAR_CALIBRATOR_RENDERER", "cpu").strip().lower()
        if renderer_mode in {"off", "0"}:
            renderer_mode = "disabled"
        launch_calibrator(sys.argv[1], CalibrationConfig(renderer_mode=renderer_mode))
        return 0
    controller = None
    startup_error = None
    try:
        controller = _controller_from_environment()
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        startup_error = f"KITTI startup failed: {exc}"
    renderer_mode = os.environ.get("LIDAR_CALIBRATOR_RENDERER", "cpu")
    return run_controller(controller, renderer_mode=renderer_mode, startup_error=startup_error)

if __name__ == "__main__":
    raise SystemExit(main())
