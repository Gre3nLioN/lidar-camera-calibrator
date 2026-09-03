"""Focused source regressions for the QML startup shell.

These checks intentionally avoid importing Qt so CI can run without a desktop
scene-graph or Windows graphics stack.
"""
from pathlib import Path


ROOT = Path(__file__).parents[1]
QML = ROOT / "src/lidar_camera_calibrator/ui/qml"


def test_main_uses_unambiguous_bridge_bindings():
    source = (QML / "Main.qml").read_text(encoding="utf-8")
    assert "property var bridge: workspaceBridge" in source
    assert "workspace: workspace" not in source
    assert source.count("workspace: window.bridge") == 7


def test_main_selects_dark_material_controls():
    source = (QML / "Main.qml").read_text(encoding="utf-8")
    startup = (ROOT / "src/lidar_camera_calibrator/ui/main.py").read_text(encoding="utf-8")
    assert "QT_QUICK_CONTROLS_STYLE\", \"Material\"" in startup
    assert "QT_QUICK_CONTROLS_MATERIAL_THEME\", \"Dark\"" in startup
    assert "Basic" not in startup
    assert "import QtQuick.Controls.Material" in source
    assert "Material.theme: Material.Dark" in source
    assert "Material.accent" in source
    assert "sync prep · may pause" in source
    assert "candidate preparation mode" in startup


def test_renderer_disabled_uses_renderer_free_entrypoint():
    main = (ROOT / "src/lidar_camera_calibrator/ui/main.py").read_text(encoding="utf-8")
    disabled = (QML / "MainDisabled.qml").read_text(encoding="utf-8")
    camera = (QML / "CameraPanelNoRenderer.qml").read_text(encoding="utf-8")
    assert "LIDAR_CALIBRATOR_RENDERER" in main
    assert "QT_QUICK_BACKEND" in main
    assert "QT_QUICK_BACKEND=" in main
    assert "renderer registration" in main
    assert "MainDisabled.qml" in main
    assert "LidarCalibrator.Rendering" not in disabled
    assert "OpenGLOverlayItem" not in disabled
    assert "OpenGLOverlayItem" not in camera
    assert "Repeater" not in (QML / "CameraPanel.qml").read_text(encoding="utf-8")
    assert "rendererDisabled" in (ROOT / "src/lidar_camera_calibrator/ui/bridge.py").read_text(encoding="utf-8")


def test_startup_failure_is_visible_not_silent():
    main = (ROOT / "src/lidar_camera_calibrator/ui/main.py").read_text(encoding="utf-8")
    bridge = (ROOT / "src/lidar_camera_calibrator/ui/bridge.py").read_text(encoding="utf-8")
    assert "startup_error =" in main
    assert "WorkspaceBridge(" in main
    assert "renderer_disabled=renderer_disabled, cpu_overlay=cpu_overlay" in main
    assert "def startupError" in bridge
    assert "startupError" in (QML / "Main.qml").read_text(encoding="utf-8")
    assert "renderer attachment" in bridge


def test_inspector_is_scrollable_collapsible_and_synchronizes_controls():
    source = (QML / "Inspector.qml").read_text(encoding="utf-8")
    assert "ScrollView" in source
    assert "extrinsicsExpanded" in source
    assert "intrinsicsExpanded" in source
    assert "Lock fx/fy" not in source
    assert "function onSnapshotChanged()" in source
    assert "extrinsicRow.key" in source
    assert "intrinsicRow.key" in source
    assert "id: extrinsicsHeader" in source
    assert "id: intrinsicsHeader" in source
    assert "MouseArea" not in source
    assert "anchors.rightMargin: 12" in source
    assert "implicitWidth: 360" in source
    assert "id: resetAllButton" in source
    assert "id: exportButton" in source
    assert source.count("Layout.fillWidth: true") >= 2
    assert "Reset all" in source


def test_cpu_mode_wires_overlay_drawer_and_maximize_panel():
    main = (QML / "MainDisabled.qml").read_text(encoding="utf-8")
    camera = (QML / "CameraPanelNoRenderer.qml").read_text(encoding="utf-8")
    assert "OverlayDrawer" in camera
    assert "cameraOverlayDrawer" in camera
    assert "SvgIconButton" in camera and 'iconSource: "icons/compare-split.svg"' in camera
    assert (QML / "SvgIconButton.qml").exists()
    assert (QML / "icons/compare-split.svg").exists()
    assert "onOpenOverlay: window.bridge.toggleOverlay()" in main
    assert "cameraMaximized" in main
    assert "signal toggleMaximize" in camera
    assert "workspace.toggleOverlay()" in camera
    assert 'contentItem: Text { text: "⚙︎"' in camera
    assert 'contentItem: Text { text: "⛶"' in camera
    assert "height: 34" in camera
    assert "anchors.verticalCenter: parent.verticalCenter" in camera
    viewport = (QML / "ProjectionViewport.qml").read_text(encoding="utf-8")
    assert "Image.PreserveAspectFit" in viewport
    assert "workspace.toggleComparison()" in camera
    assert 'title: "Original"' in camera
    assert 'title: workspace.comparisonOpen ? "Working"' in camera
    assert "onObservedFrameChanged: resetView()" in camera
    assert "onObservedCameraChanged: resetView()" in camera
    assert "onWheel: function(wheel)" in viewport
    assert "Math.min(8" in viewport
    assert "Qt.ControlModifier" in viewport
    assert "viewState.offsetX" in viewport


def test_main_workspace_prioritizes_navigable_lidar_scene_and_camera_routes():
    main = (QML / "MainDisabled.qml").read_text(encoding="utf-8")
    scene = (QML / "MainScene.qml").read_text(encoding="utf-8")
    lidar = (QML / "LidarScenePanel.qml").read_text(encoding="utf-8")
    camera = (QML / "CameraPreview.qml").read_text(encoding="utf-8")
    assert 'property string workspaceMode: "scene"' in main
    assert "MainScene" in main and "onCalibrateRequested" in main
    assert "LidarScenePanel" in scene
    assert scene.count("CameraPreview") == 1
    assert "dynamicCameraRepeater" in scene and "workspace.cameraNames" in scene
    assert 'cameraId: "image_02"' not in scene and 'cameraId: "image_03"' not in scene
    assert "workspace.lidarViewUrl" in lidar
    assert "Qt.ControlModifier" in lidar
    assert "onWheel: function(wheel)" in lidar
    assert "scene.yaw += dx" in lidar
    assert "scene.pitch + dy" in lidar
    assert "id: viewportResolutionUpdate" in lidar
    assert "onWidthChanged: viewportResolutionUpdate.restart()" in lidar
    assert "if (!navigationUpdate.running) navigationUpdate.start()" in lidar
    assert "scene.publishView(true)" in lidar
    assert "scene.publishView(false)" in lidar
    assert "workspace.beginLidarNavigation" in lidar
    assert "onReleased: { navigationUpdate.stop(); settleUpdate.stop()" in lidar
    assert 'color: "#32ff78"' in lidar
    assert 'glyph: "⛶"' in camera
    assert 'glyph: "⚙︎"' in camera
    assert "preview.overlaySource" in camera
    assert "readonly property var cameraModel: mainScene.workspace.cameraModels[index]" in scene
    assert "overlaySource: cameraModel.overlayUrl" in scene
    assert (QML / "CameraPreview.qml").read_text(encoding="utf-8").count("retainWhileLoading: true") == 2
    projection_viewport = (QML / "ProjectionViewport.qml").read_text(encoding="utf-8")
    assert projection_viewport.count("retainWhileLoading: true") == 2
    assert "visible: source !== \"\"" in projection_viewport
    assert "imageSource: cameraModel.imageUrl" in scene
    main_disabled = (QML / "MainDisabled.qml").read_text(encoding="utf-8")
    assert 'objectName: "previewProjectionToggle"' in main_disabled
    assert main_disabled.index("id: backToScene") > main_disabled.index("Item { Layout.fillWidth: true }")


def test_cpu_overlay_mode_uses_a_separate_transparent_image_provider():
    main = (ROOT / "src/lidar_camera_calibrator/ui/main.py").read_text(encoding="utf-8")
    bridge = (ROOT / "src/lidar_camera_calibrator/ui/bridge.py").read_text(encoding="utf-8")
    camera = (QML / "CameraPanelNoRenderer.qml").read_text(encoding="utf-8")
    assert '"cpu"' in main
    assert 'LIDAR_CALIBRATOR_BUFFER_FRAMES", "10"' in main
    assert 'LIDAR_CALIBRATOR_PLAYBACK_SPEED", "4.0"' in bridge
    assert "WorkspaceOverlayImageProvider" in main
    assert '"workspace-overlay"' in main
    assert "LIDAR_CALIBRATOR_CPU_MAX_POINTS" in bridge
    assert "def overlayUrl" in bridge
    assert "render_cpu_overlay" in bridge
    assert "workspace.overlayUrl" in camera
    drawer = (QML / "OverlayDrawer.qml").read_text(encoding="utf-8")
    assert "LIDAR OVERLAY" in (QML / "MainDisabled.qml").read_text(encoding="utf-8")
    assert "Only points in image" not in drawer
    assert "always clips to the image" in drawer
    assert "onMoved: workspace.setOverlaySetting(\"pointSizePx\"" in drawer
    assert "objectName: \"overlayPointSize\"" in drawer
    assert "id: coloringSelector" in drawer
    assert "id: densitySelector" in drawer
    assert drawer.count('text: "▼"') == 2
    assert drawer.count('color: "#202833"') == 4
    assert drawer.count("RowLayout") == 2
    assert drawer.count("MenuItem") == 5


def test_scenegraph_migration_does_not_register_the_legacy_fbo_item():
    package = (ROOT / "src/lidar_camera_calibrator/rendering/__init__.py").read_text(encoding="utf-8")
    item = (ROOT / "src/lidar_camera_calibrator/rendering/scenegraph_overlay.py").read_text(encoding="utf-8")
    camera = (QML / "CameraPanel.qml").read_text(encoding="utf-8")
    assert "SceneGraphOverlayItem" in package
    assert "OpenGLOverlayItem" not in package
    assert "import QQuickFramebufferObject" not in item
    assert "glDrawArrays" not in item
    assert "SceneGraphOverlayItem" in camera
    assert "Repeater" not in camera
    bridge = (ROOT / "src/lidar_camera_calibrator/ui/bridge.py").read_text(encoding="utf-8")
    assert "supports_points" in bridge
    assert "mark_unavailable" in bridge


def test_windows_preparation_uses_explicit_safe_mode_with_override():
    bridge = (ROOT / "src/lidar_camera_calibrator/ui/bridge.py").read_text(encoding="utf-8")
    timeline = (QML / "Timeline.qml").read_text(encoding="utf-8")
    assert 'LIDAR_CALIBRATOR_PREP_MODE' in bridge
    assert 'sys.platform == "win32"' in bridge
    assert "if not self._prepare_synchronously:" in bridge
    assert "prepare_renderer_input(" in bridge
    assert "self._prep_pool.start(task)" in bridge
    assert "safePreparationMode" in bridge
    assert "Safe preparation (may pause)" in timeline
    assert 'objectName: "timelineFrameMarker"' in timeline
    assert "workspace.selectFrame(marker.frameNumber)" in timeline
    assert "workspace.bufferedFrameCount" in timeline
    assert "workspace.bufferedFrameCount >= 0 && workspace.frameBuffered(frameNumber)" in timeline
    assert 'id: transportStrip' in timeline
    assert 'id: playButton' in timeline
    assert 'id: track' in timeline
    assert 'workspace.playbackSpeed.toFixed(0) + "×"' in timeline
    assert 'Shortcut { sequence: "Space"' in (QML / "MainDisabled.qml").read_text(encoding="utf-8")


def test_calibration_controls_support_reset_history_and_spinbox_inputs():
    inspector = (QML / "Inspector.qml").read_text(encoding="utf-8")
    main = (QML / "MainDisabled.qml").read_text(encoding="utf-8")
    bridge = (ROOT / "src/lidar_camera_calibrator/ui/bridge.py").read_text(encoding="utf-8")
    assert inspector.count("SpinBox") == 2
    assert "TapHandler" not in inspector
    assert inspector.count("SceneIconButton") >= 6
    assert 'objectName: "resetExtrinsicParameter_"' in inspector
    assert 'objectName: "extrinsicSlider_"' in inspector
    assert 'objectName: "extrinsicSpin_"' in inspector
    assert "workspace.resetExtrinsics()" in inspector
    assert "workspace.resetIntrinsics()" in inspector
    assert "workspace.undo()" in inspector and "workspace.redo()" in inspector
    assert 'sequence: "Ctrl+Z"' in main and 'sequence: "Ctrl+Shift+Z"' in main
    assert "Calibration JSON saved" in main and "saveToastTimer" in main
    assert "exportSuccessId" in main and "z: 10000" in main
    assert "def beginCalibrationChange" in bridge and "def redo" in bridge


def test_export_uses_native_save_dialog_and_direction_explicit_format():
    dialog = (QML / "ExportDialog.qml").read_text(encoding="utf-8")
    bridge = (ROOT / "src/lidar_camera_calibrator/ui/bridge.py").read_text(encoding="utf-8")
    controller = (ROOT / "src/lidar_camera_calibrator/app/controller.py").read_text(encoding="utf-8")
    assert "FileDialog" in dialog
    assert "FileDialog.SaveFile" in dialog
    assert "workspace.confirmExport(selectedFile)" in dialog
    assert "Direction-explicit camera edge transforms" in dialog
    assert "onRejected: workspace.cancelExport()" in dialog
    assert "@Slot(QUrl)" in bridge
    assert '"confirm_export": ("path",)' in controller
