import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

ApplicationWindow {
    id: window
    visible: true
    width: 1440; height: 900; minimumWidth: 1024; minimumHeight: 680
    title: window.bridge.cpuOverlayEnabled ? "Vector · LiDAR Camera Calibrator" : "Vector · LiDAR Camera Calibrator (renderer disabled)"
    color: "#0d1117"
    Material.theme: Material.Dark
    Material.accent: "#54b7ff"
    property var bridge: workspaceBridge
    property string workspaceMode: "scene"
    property bool cameraMaximized: false
    property bool saveToastVisible: false
    property int shownExportSuccessId: 0
    onWorkspaceModeChanged: window.bridge.setCalibrationActive(workspaceMode === "calibration")
    Component.onCompleted: window.bridge.setCalibrationActive(false)
    Shortcut { sequence: "Space"; context: Qt.ApplicationShortcut; onActivated: window.bridge.togglePlay() }
    Shortcut { sequence: "Ctrl+Z"; context: Qt.ApplicationShortcut; enabled: window.workspaceMode === "calibration"; onActivated: window.bridge.undo() }
    Shortcut { sequence: "Ctrl+Shift+Z"; context: Qt.ApplicationShortcut; enabled: window.workspaceMode === "calibration"; onActivated: window.bridge.redo() }
    Connections { target: window.bridge
        function onExportSuccessIdChanged() {
            if (window.bridge.exportSuccessId > window.shownExportSuccessId) {
                window.shownExportSuccessId = window.bridge.exportSuccessId
                window.saveToastVisible = true
                saveToastTimer.restart()
            }
        }
    }
    Timer { id: saveToastTimer; interval: 5000; repeat: false; onTriggered: window.saveToastVisible = false }
    header: Rectangle { height: window.workspaceMode === "scene" && window.bridge.startupError === "" ? 58 : (window.bridge.startupError === "" ? 88 : 118); color: "#12171e"; border.color: "#29313d"
        RowLayout { anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; height: 58; anchors.leftMargin: 22; anchors.rightMargin: 18; spacing: 18
            Text { text: "VECTOR"; color: "#e7edf5"; font.pixelSize: 15; font.bold: true; font.letterSpacing: 2 }
            Text { text: window.bridge.datasetLabel; color: "#f1f5f9"; font.pixelSize: 13; font.bold: true }
            Text { text: window.workspaceMode === "scene" ? "3D VIEW" : "CALIBRATION"; color: "#8290a3"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.2 }
            Item { Layout.fillWidth: true }
            Button { id: previewProjectionToggle; objectName: "previewProjectionToggle"; visible: window.workspaceMode === "scene"; checkable: true; checked: window.bridge.previewProjectionEnabled; text: "Project points"; hoverEnabled: true; onClicked: window.bridge.togglePreviewProjection()
                contentItem: Text { text: previewProjectionToggle.text; color: previewProjectionToggle.checked ? "#dff8ff" : "#d6e0ec"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                background: Rectangle { radius: height * 0.10; color: previewProjectionToggle.checked ? (previewProjectionToggle.hovered ? "#238cd1" : "#176da5") : (previewProjectionToggle.hovered ? "#26384a" : "#202833"); border.width: 1; border.color: previewProjectionToggle.checked ? "#65c4ff" : (previewProjectionToggle.hovered ? "#4b7599" : "#344253") }
            }
            Button { id: backToScene; visible: window.workspaceMode === "calibration"; text: "Back to LiDAR view"; hoverEnabled: true; onClicked: { window.cameraMaximized = false; window.workspaceMode = "scene" }
                contentItem: Text { text: backToScene.text; color: "#d6e0ec"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                background: Rectangle { radius: height * 0.10; color: backToScene.hovered ? "#26384a" : "#202833"; border.width: 1; border.color: backToScene.hovered ? "#4b7599" : "#344253" }
            }
            Text { text: window.workspaceMode === "scene" ? "●  " + window.bridge.lidarPointCount.toLocaleString() + " POINTS" : (window.bridge.cpuOverlayEnabled ? "●  LIDAR OVERLAY" : "⚠  RENDERER DISABLED"); color: window.workspaceMode === "scene" ? "#55d99a" : "#f4b85c"; font.pixelSize: 11; font.bold: true }
        }
        Rectangle { visible: window.workspaceMode === "calibration" || window.bridge.startupError !== ""; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.leftMargin: 22; anchors.rightMargin: 22; height: 24; color: "#352d20"; radius: 4
            Text { anchors.fill: parent; anchors.leftMargin: 10; verticalAlignment: Text.AlignVCenter; text: window.bridge.diagnosticMessage; color: "#ffd18a"; font.pixelSize: 11 }
        }
    }
    RowLayout { anchors.fill: parent; anchors.margins: 1; spacing: 1
        SourceTree {
            id: sceneInspector
            Layout.preferredWidth: collapsed ? 54 : Math.max(210, Math.min(250, window.width * 0.17))
            Layout.fillHeight: true
            workspace: window.bridge
            visible: !window.cameraMaximized && !(window.workspaceMode === "scene" && mainScene.panelMaximized)
            onSceneRequested: { window.cameraMaximized = false; window.workspaceMode = "scene" }
            onCalibrationRequested: function(cameraId) { window.bridge.selectCamera(cameraId); window.cameraMaximized = false; window.workspaceMode = "calibration" }
        }
        Item { Layout.fillWidth: true; Layout.fillHeight: true
            MainScene { id: mainScene; anchors.fill: parent; workspace: window.bridge; visible: window.workspaceMode === "scene"
                onCalibrateRequested: function(cameraId) { window.bridge.selectCamera(cameraId); window.cameraMaximized = false; window.workspaceMode = "calibration" }
            }
            CameraPanelNoRenderer { anchors.fill: parent; workspace: window.bridge; visible: window.workspaceMode === "calibration" && !window.cameraMaximized; onToggleMaximize: window.cameraMaximized = true }
            CameraPanelNoRenderer { anchors.fill: parent; workspace: window.bridge; visible: window.workspaceMode === "calibration" && window.cameraMaximized; onToggleMaximize: window.cameraMaximized = false }
        }
        Inspector { Layout.preferredWidth: Math.max(320, window.width * 0.24); Layout.fillHeight:true; workspace: window.bridge; visible: window.workspaceMode === "calibration" && !window.cameraMaximized; onOpenOverlay: window.bridge.toggleOverlay() }
    }
    footer: Item {
        implicitHeight: Math.max(108, window.height * 0.13)
        Timeline {
            anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
            height: parent.height * 0.88
            workspace: window.bridge
        }
    }
    Rectangle { id: saveToast; visible: window.saveToastVisible; z: 10000
        anchors.top: parent.top; anchors.topMargin: parent.height * 0.015; anchors.right: parent.right; anchors.rightMargin: parent.width * 0.018
        width: parent.width * 0.28; height: parent.height * 0.06; radius: height * 0.13; color: "#17352d"; border.color: "#3c9e7a"
        RowLayout { anchors.fill: parent; anchors.margins: 12; spacing: 10
            Text { text: "✓"; color: "#62d6a7"; font.pixelSize: 18; font.bold: true }
            ColumnLayout { Layout.fillWidth: true; spacing: 1
                Text { text: "Calibration JSON saved"; color: "#e5fff5"; font.pixelSize: 12; font.bold: true }
                Text { text: window.bridge.exportPath; color: "#a7d9c7"; font.pixelSize: 10; elide: Text.ElideMiddle; Layout.fillWidth: true }
            }
            SceneIconButton { Layout.preferredWidth: 24; Layout.preferredHeight: 24; glyph: "×"; tooltipText: "Close"; onClicked: { saveToastTimer.stop(); window.saveToastVisible = false } }
        }
    }
    ExportDialog { workspace: window.bridge; visible: window.bridge.exportOpen; anchors.centerIn: parent; z: 20 }
}
