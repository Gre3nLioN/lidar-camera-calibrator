import os
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_QUICK_BACKEND", "software")
QtGui = pytest.importorskip("PySide6.QtGui")
from PySide6.QtCore import QObject, QPoint, QPointF, QUrl, Qt
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlComponent, QQmlEngine
from PySide6.QtQuick import QQuickItem, QQuickWindow
from PySide6.QtTest import QTest

from lidar_camera_calibrator.app import WorkspaceController
from lidar_camera_calibrator.ui.bridge import WorkspaceBridge
from test_app_controller import FakeAdapter


def _find_visual_item(item: QQuickItem, object_name: str):
    if item.objectName() == object_name:
        return item
    for child in item.childItems():
        found = _find_visual_item(child, object_name)
        if found is not None:
            return found
    return None


def test_playback_stops_and_reports_frame_decode_errors():
    app = QGuiApplication.instance() or QGuiApplication([])

    class FailingAdapter(FakeAdapter):
        def is_ready(self, index): return True
        def frame(self, index):
            if int(index) == 1:
                raise RuntimeError("corrupt camera payload")
            return super().frame(index)

    bridge = WorkspaceBridge(WorkspaceController(FailingAdapter()), cpu_overlay=False)
    try:
        bridge.togglePlay()
        app.processEvents()
        assert bridge.playing is False
        assert bridge.projectionStatus == "error"
        assert "Frame 2 failed to load" in bridge.projectionMessage
        assert "corrupt camera payload" in bridge.projectionMessage
    finally:
        bridge.shutdown()


def test_scene_icon_button_hover_covers_the_full_bordered_button():
    app = QGuiApplication.instance() or QGuiApplication([])
    engine = QQmlEngine()
    qml_path = Path(__file__).parents[1] / "src/lidar_camera_calibrator/ui/qml/SceneIconButton.qml"
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(qml_path)))
    button = component.create()
    assert button is not None, [error.toString() for error in component.errors()]

    window = QQuickWindow()
    window.setWidth(80)
    window.setHeight(80)
    button.setParentItem(window.contentItem())
    button.setProperty("x", 20)
    button.setProperty("y", 20)
    button.setProperty("width", 28)
    button.setProperty("height", 28)
    window.show()
    app.processEvents()

    points = (QPoint(21, 34), QPoint(47, 34))
    for point in points:
        QTest.mouseMove(window, point)
        app.processEvents()
        assert button.property("pointerHovered") is True

    clicks = []
    button.clicked.connect(lambda: clicks.append(True))
    for point in points:
        QTest.mouseClick(window, Qt.LeftButton, Qt.NoModifier, point)
        app.processEvents()
    assert len(clicks) == 2

    window.close()
    button.deleteLater()
    engine.deleteLater()


def test_inspector_z_reset_click_resets_bridge_slider_and_spinbox_inside_scroll_view():
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=False)
    bridge.adjustTranslation("z", 2.517)
    assert bridge.parameter("z") == pytest.approx(2.517)

    qml_dir = Path(__file__).parents[1] / "src/lidar_camera_calibrator/ui/qml"
    qml_url = QUrl.fromLocalFile(str(qml_dir)).toString()
    engine = QQmlEngine()
    engine.rootContext().setContextProperty("workspaceBridge", bridge)
    component = QQmlComponent(engine)
    component.setData(
        f'''import QtQuick
import "{qml_url}" as App
App.Inspector {{ width: 620; height: 620; workspace: workspaceBridge }}'''.encode(),
        QUrl.fromLocalFile(str(qml_dir / "InspectorHarness.qml")),
    )
    root = component.create()
    assert root is not None, [error.toString() for error in component.errors()]

    window = QQuickWindow()
    window.setWidth(620)
    window.setHeight(620)
    root.setParentItem(window.contentItem())
    window.show()
    QTest.qWait(20)
    reset = _find_visual_item(root, "resetExtrinsicParameter_z")
    slider = _find_visual_item(root, "extrinsicSlider_z")
    spinbox = _find_visual_item(root, "extrinsicSpin_z")
    assert reset is not None and slider is not None and spinbox is not None
    center = reset.mapToScene(QPointF(reset.width() / 2, reset.height() / 2))
    QTest.mouseClick(window, Qt.LeftButton, Qt.NoModifier, QPoint(round(center.x()), round(center.y())))
    app.processEvents()

    assert bridge.parameter("z") == 0.0
    assert slider.property("value") == 0.0
    assert spinbox.property("value") == 0
    window.close()
    root.deleteLater()
    engine.deleteLater()


def test_calibration_playback_uses_bounded_direct_overlay_then_restores_full_profile(monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)
    initial_prepared_token = bridge._cpu_prepared_token
    initial_revision = bridge._cpu_overlay_revision

    bridge.togglePlay()
    assert bridge.playing is True
    assert bridge._cpu_prepared_token == initial_prepared_token
    assert bridge._cpu_overlay_revision > initial_revision

    bridge.togglePlay()
    assert bridge.playing is False
    assert bridge._cpu_prepared_token == initial_prepared_token
    assert app is not None


def test_export_success_notification_revision_is_not_retriggered_by_undo(tmp_path, monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)
    bridge.adjustTranslation("x", 0.1)
    bridge.requestExport()
    bridge.confirmExport(str(tmp_path / "calibration.json"))
    assert bridge.exportSuccessId == 1
    bridge.undo()
    assert bridge.exportSuccessId == 1


def test_cpu_bridge_commands_refresh_state_and_overlay(monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)

    initial_url = bridge.overlayUrl
    assert bridge.overlay_array() is not None
    assert bridge.parameter("x") == 0.0

    bridge.adjustTranslation("x", 0.1)
    assert bridge.parameter("x") == pytest.approx(0.1)
    assert bridge.overlayUrl != initial_url

    after_translation = bridge.overlayUrl
    bridge.setOverlaySetting("pointSizePx", 5.0)
    assert bridge.overlayPointSize == pytest.approx(5.0)
    assert bridge.overlayUrl != after_translation
    assert np.count_nonzero(bridge.overlay_array()[..., 3]) > 0

    bridge.setOverlaySetting("opacity", 0.0)
    assert bridge.overlayOpacity == 0.0
    assert not np.any(bridge.overlay_array()[..., 3])

    density_url = bridge.overlayUrl
    bridge.setOverlaySetting("renderDensity", "light")
    assert bridge.overlayDensity == "light"
    assert bridge.overlayUrl != density_url

    bridge.resetAll()
    assert bridge.parameter("x") == 0.0
    assert app is not None


def test_lidar_scene_uses_full_frame_and_navigation_refreshes_image(monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)

    initial_url = bridge.lidarViewUrl
    assert bridge.lidarPointCount == 1
    assert bridge.lidar_scene_array() is not None
    rows, columns = np.nonzero(bridge._lidar_pick_indices >= 0)
    assert len(rows)
    picked = bridge.beginLidarNavigation(
        float(columns[0]) / bridge._lidar_pick_indices.shape[1],
        float(rows[0]) / bridge._lidar_pick_indices.shape[0],
    )
    assert picked is True
    bridge.setLidarView(30.0, 45.0, 2.0, 0.1, -0.1)
    assert bridge.lidarViewUrl != initial_url
    assert bridge.lidarPointCount == 1
    view_before = dict(bridge._lidar_view)
    pivot_before = np.array(bridge._lidar_pivot, copy=True)
    scale_before = bridge._lidar_base_scale_ratio

    bridge.seek(1)

    assert bridge._lidar_view == view_before
    np.testing.assert_array_equal(bridge._lidar_pivot, pivot_before)
    assert bridge._lidar_base_scale_ratio == scale_before
    assert app is not None


def test_main_scene_skips_hidden_calibration_overlay_work(monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)
    calibration_url = bridge.overlayUrl

    bridge.setCalibrationActive(False)
    bridge.seek(1)

    assert bridge.overlayUrl == calibration_url
    bridge.setCalibrationActive(True)
    assert bridge.overlayUrl != calibration_url
    assert app is not None


def test_main_camera_preview_projection_toggle_renders_both_working_cameras(monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)

    assert bridge.previewProjectionEnabled is False
    assert bridge.previewOverlayUrl("image_02") == ""
    bridge.togglePreviewProjection()
    assert bridge.previewProjectionEnabled is True
    assert bridge.previewOverlayUrl("image_02")
    assert bridge.preview_overlay_array("image_02") is not None
    image02_url = bridge.previewOverlayUrl("image_02")

    bridge.adjustTranslation("x", 0.1)

    assert bridge.previewOverlayUrl("image_02") != image02_url
    bridge.togglePreviewProjection()
    assert bridge.previewOverlayUrl("image_02") == ""
    assert app is not None


def test_comparison_keeps_original_immutable_while_working_projection_updates(monkeypatch):
    monkeypatch.setenv("LIDAR_CALIBRATOR_PREP_MODE", "sync")
    app = QGuiApplication.instance() or QGuiApplication([])
    bridge = WorkspaceBridge(WorkspaceController(FakeAdapter()), cpu_overlay=True)

    assert bridge.comparisonOpen is False
    bridge.toggleComparison()
    assert bridge.comparisonOpen is True
    assert bridge.originalOverlayUrl
    original_url = bridge.originalOverlayUrl
    original_pixels = np.array(bridge.overlay_array(original=True), copy=True)
    working_url = bridge.overlayUrl

    bridge.adjustTranslation("x", 0.1)

    assert bridge.overlayUrl != working_url
    assert bridge.originalOverlayUrl == original_url
    assert np.array_equal(bridge.overlay_array(original=True), original_pixels)
    assert app is not None
