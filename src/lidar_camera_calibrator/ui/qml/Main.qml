import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

ApplicationWindow {
    id: window
    visible: true
    width: 1440; height: 900; minimumWidth: 1024; minimumHeight: 680
    title: "Vector · LiDAR Camera Calibrator"
    color: "#0d1117"
    Material.theme: Material.Dark
    Material.accent: "#54b7ff"
    Material.primary: Material.BlueGrey
    property bool cameraMaximized: false
    // Explicitly capture the context bridge; child `workspace` properties must
    // never bind to the same-named context identifier (Windows QML self-shadowing).
    property var bridge: workspaceBridge
    header: Rectangle { height: window.bridge.startupError === "" ? 58 : 88; color: "#12171e"; border.color: "#29313d"
        RowLayout { anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; height: 58; anchors.leftMargin: 22; anchors.rightMargin: 18; spacing: 18
            Text { text: "VECTOR"; color: "#e7edf5"; font.pixelSize: 15; font.bold: true; font.letterSpacing: 2 }
            Rectangle { width:1; height:24; color:"#344253" }
            Text { text: window.bridge.datasetLabel; color: "#f1f5f9"; font.pixelSize: 13; font.bold: true }
            Text { text: window.bridge.selectedCamera; color: "#8290a3"; font.pixelSize: 12 }
            Item { Layout.fillWidth: true }
            Text { visible: window.bridge.safePreparationMode; text: "⚠  sync prep · may pause"; color: "#f4b85c"; font.pixelSize: 11 }
            Text { text: "●  synced"; color: "#62d6a7"; font.pixelSize: 12 }
            Button { text: "Share"; onClicked: window.bridge.requestExport() }
        }
        Rectangle { visible: window.bridge.startupError !== ""; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.leftMargin: 22; anchors.rightMargin: 22; height: 24; color: "#3b2529"; radius: 4
            Text { anchors.fill: parent; anchors.leftMargin: 10; verticalAlignment: Text.AlignVCenter; text: window.bridge.startupError; color: "#ffb4ab"; font.pixelSize: 11; elide: Text.ElideRight }
        }
    }
    RowLayout { anchors.fill: parent; anchors.margins: 1; spacing: 1
        SourceTree {
            id: sceneInspector
            Layout.preferredWidth: collapsed ? 54 : 232
            Layout.fillHeight: true
            workspace: window.bridge
            visible: !window.cameraMaximized
            onCalibrationRequested: function(cameraId) { window.bridge.selectCamera(cameraId) }
        }
        Item { Layout.fillWidth:true; Layout.fillHeight:true
            CameraPanel { anchors.fill: parent; workspace: window.bridge; visible: !window.cameraMaximized; onToggleMaximize: window.cameraMaximized = !window.cameraMaximized }
            CameraPanel { anchors.fill: parent; workspace: window.bridge; visible: window.cameraMaximized; onToggleMaximize: window.cameraMaximized = false }
        }
        Inspector { Layout.preferredWidth: 360; Layout.fillHeight:true; workspace: window.bridge; visible: !window.cameraMaximized; onOpenOverlay: window.bridge.toggleOverlay() }
    }
    footer: Timeline { workspace: window.bridge }
    OverlayDrawer { anchors.top: parent.top; anchors.bottom: parent.bottom; anchors.right: parent.right; workspace: window.bridge; visible: window.bridge.overlayOpen; z: 10 }
    ExportDialog { workspace: window.bridge; visible: window.bridge.exportOpen; anchors.centerIn: parent; z: 20 }
}
