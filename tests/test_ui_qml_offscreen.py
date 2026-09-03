import os
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QUICK_BACKEND", "software")
pytest.importorskip("PySide6.QtGui")
from PySide6.QtCore import QObject, QPointF, Qt
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtTest import QTest

from lidar_camera_calibrator.app import WorkspaceController
from lidar_camera_calibrator.ui.bridge import WorkspaceBridge
from lidar_camera_calibrator.ui.main import WorkspaceImageProvider, WorkspaceLidarImageProvider, WorkspaceOverlayImageProvider
from test_app_controller import FakeAdapter


ROOT = Path(__file__).parents[1]


def test_cpu_mode_qml_loads_offscreen(monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)
    engine = QQmlApplicationEngine()
    engine.addImageProvider("workspace", WorkspaceImageProvider(bridge))
    engine.addImageProvider("workspace-overlay", WorkspaceOverlayImageProvider(bridge))
    engine.addImageProvider("workspace-lidar", WorkspaceLidarImageProvider(bridge))
    engine.rootContext().setContextProperty("workspaceBridge", bridge)
    engine.load(str(ROOT / "src/lidar_camera_calibrator/ui/qml/MainDisabled.qml"))

    roots = engine.rootObjects()
    assert roots
    root = roots[0]
    app.processEvents()
    assert root.property("workspaceMode") == "scene"
    scene_inspector = root.findChild(QObject, "sceneInspector")
    assert scene_inspector is not None
    assert scene_inspector.property("collapsed") is False
    assert scene_inspector.width() >= 210
    sensor_scroll = root.findChild(QObject, "sensorScroll")
    sensor_column = root.findChild(QObject, "sensorColumn")
    assert sensor_scroll is not None and sensor_column is not None
    assert sensor_column.property("width") >= 180
    assert sensor_column.property("width") <= sensor_scroll.property("width")
    scene_inspector.setProperty("collapsed", True)
    app.processEvents()
    assert scene_inspector.width() <= 55
    sidebar_toggle = root.findChild(QObject, "sidebarToggle")
    assert sidebar_toggle is not None
    assert sidebar_toggle.property("width") >= 40
    scene_inspector.setProperty("collapsed", False)
    app.processEvents()
    preview_toggle = root.findChild(QObject, "previewProjectionToggle")
    assert preview_toggle is not None
    toggle_center = preview_toggle.mapToScene(QPointF(preview_toggle.width() * 0.5, preview_toggle.height() * 0.5)).toPoint()
    QTest.mouseClick(root, Qt.LeftButton, Qt.NoModifier, toggle_center)
    app.processEvents()
    assert bridge.previewProjectionEnabled is True
    assert bridge.previewOverlayUrl("image_02")

    repeater = root.findChild(QObject, "dynamicCameraRepeater")
    assert repeater is not None and repeater.property("count") == len(bridge.cameraNames)
    main_scene = root.findChild(QObject, "mainScene")
    assert main_scene is not None
    QTest.qWait(100)
    app.processEvents()
    preview_before_seek = main_scene.property("firstCameraDelegate")
    assert preview_before_seek is not None
    source_before_seek = preview_before_seek.property("imageSource")
    bridge.selectFrame(1)
    app.processEvents()
    # Stable numeric camera-count models preserve retained Image items while their
    # frame-dependent metadata bindings still advance.
    assert main_scene.property("firstCameraDelegate") == preview_before_seek
    assert preview_before_seek.property("imageSource") != source_before_seek
    bridge.selectCamera("image_02")
    root.setProperty("workspaceMode", "calibration")
    app.processEvents()
    assert root.property("workspaceMode") == "calibration"

    bridge.toggleOverlay()
    app.processEvents()
    sliders = root.findChildren(QObject, "overlayPointSize")
    slider = next((candidate for candidate in sliders if candidate.isVisible()), None)
    assert slider is not None
    before_size, before_url = bridge.overlayPointSize, bridge.overlayUrl
    start = slider.mapToScene(QPointF(slider.width() * 0.5, slider.height() * 0.5)).toPoint()
    end = slider.mapToScene(QPointF(slider.width() * 0.9, slider.height() * 0.5)).toPoint()
    QTest.mousePress(root, Qt.LeftButton, Qt.NoModifier, start)
    QTest.qWait(20)
    QTest.mouseMove(root, end, 20)
    QTest.mouseRelease(root, Qt.LeftButton, Qt.NoModifier, end)
    app.processEvents()
    assert bridge.overlayPointSize > before_size
    assert bridge.overlayUrl != before_url

    bridge.toggleOverlay()
    app.processEvents()
    inspector = root.findChild(QObject, "calibrationInspector")
    extrinsics = root.findChild(QObject, "extrinsicsHeader")
    assert inspector is not None and extrinsics is not None
    assert inspector.property("extrinsicsExpanded") is True
    header_center = extrinsics.mapToScene(QPointF(extrinsics.width() * 0.8, extrinsics.height() * 0.5)).toPoint()
    QTest.mouseClick(root, Qt.LeftButton, Qt.NoModifier, header_center)
    app.processEvents()
    assert inspector.property("extrinsicsExpanded") is False

    comparison = root.findChild(QObject, "comparisonButton")
    assert comparison is not None
    assert comparison.width() == pytest.approx(34)
    assert comparison.height() == pytest.approx(34)
    comparison_background = comparison.property("background")
    assert comparison_background.width() == pytest.approx(comparison.width())
    assert comparison_background.height() == pytest.approx(comparison.height())
    comparison_center = comparison.mapToScene(QPointF(comparison.width() * 0.5, comparison.height() * 0.5)).toPoint()
    QTest.mouseClick(root, Qt.LeftButton, Qt.NoModifier, comparison_center)
    app.processEvents()
    assert bridge.comparisonOpen is True
    assert bridge.originalOverlayUrl
    assert root.findChild(QObject, "originalProjectionViewport").property("visible") is True

    timeline_repeater = root.findChild(QObject, "timelineRepeater")
    assert timeline_repeater is not None
    assert timeline_repeater.property("count") == bridge.frameCount
    QTest.keyClick(root, Qt.Key_Space)
    app.processEvents()
    assert bridge.playing is True
    QTest.keyClick(root, Qt.Key_Space)
    app.processEvents()
    assert bridge.playing is False

    root.close()
    engine.rootContext().setContextProperty("workspaceBridge", None)
    engine.clearComponentCache()
    del roots
    del engine
    app.processEvents()
