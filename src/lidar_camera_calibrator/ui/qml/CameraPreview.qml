import QtQuick
import QtQuick.Layouts

Rectangle {
    id: preview
    objectName: cameraId + "Preview"
    required property string cameraId
    required property url imageSource
    property url overlaySource
    property bool maximized: false
    signal toggleMaximize
    signal calibrate
    color: "#111820"
    border.color: "#293746"
    radius: Math.min(width, height) * 0.012
    clip: true

    ColumnLayout {
        anchors.fill: parent
        spacing: 0
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: Math.max(cameraLabel.implicitHeight * 2.4, preview.height * 0.12)
            color: "#151a21"
            RowLayout { anchors.fill: parent; anchors.leftMargin: parent.width * 0.025; anchors.rightMargin: parent.width * 0.02; spacing: parent.width * 0.012
                Text { text: "CAMERA"; color: "#8290a3"; font.pixelSize: cameraLabel.font.pixelSize * 0.76; font.bold: true; font.letterSpacing: 1.1; Layout.alignment: Qt.AlignVCenter }
                Text { id: cameraLabel; text: preview.cameraId; color: "#f1f5f9"; font.bold: true; Layout.alignment: Qt.AlignVCenter }
                Item { Layout.fillWidth: true }
                SceneIconButton { objectName: preview.cameraId + "FullscreenButton"; Layout.preferredHeight: parent.height * 0.82; Layout.preferredWidth: height; glyph: "⛶"; tooltipText: preview.maximized ? "Restore camera panel" : "Maximize camera panel"; onClicked: preview.toggleMaximize() }
                SceneIconButton { objectName: preview.cameraId + "CalibrateButton"; Layout.preferredHeight: parent.height * 0.82; Layout.preferredWidth: height; glyph: "⚙︎"; tooltipText: "Calibrate " + preview.cameraId; onClicked: preview.calibrate() }
            }
        }
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Image { anchors.fill: parent; anchors.margins: Math.max(2, Math.min(parent.width, parent.height) * 0.015); source: preview.imageSource; fillMode: Image.PreserveAspectFit; asynchronous: true; retainWhileLoading: true; smooth: true }
            Image { anchors.fill: parent; anchors.margins: Math.max(2, Math.min(parent.width, parent.height) * 0.015); source: preview.overlaySource; fillMode: Image.PreserveAspectFit; asynchronous: false; retainWhileLoading: true; smooth: false; visible: source !== "" }
        }
    }
}
