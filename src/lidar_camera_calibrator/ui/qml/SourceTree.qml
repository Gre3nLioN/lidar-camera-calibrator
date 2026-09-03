import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: panel
    objectName: "sceneInspector"
    required property var workspace
    property bool collapsed: false
    readonly property int expandedWidth: 232
    signal sceneRequested()
    signal calibrationRequested(string cameraId)
    color: "#11161d"
    border.color: "#29313d"
    clip: true

    component SensorRow: Rectangle {
        id: sensorRow
        required property string glyph
        required property string title
        property string subtitle: ""
        property string statusText: ""
        property color statusColor: "#62d6a7"
        property bool selected: false
        property bool modified: false
        property bool actionable: true
        property int trailingSpace: 0
        signal activated()
        Layout.fillWidth: true
        Layout.preferredHeight: panel.collapsed ? 46 : 58
        radius: 6
        color: selected ? "#1c3448" : (rowMouse.containsMouse && actionable ? "#19222d" : "transparent")
        border.width: selected ? 1 : 0
        border.color: "#4189bb"

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: panel.collapsed ? 0 : 9
            anchors.rightMargin: panel.collapsed ? 0 : 7 + sensorRow.trailingSpace
            spacing: 8
            Item {
                Layout.preferredWidth: panel.collapsed ? sensorRow.width : 26
                Layout.fillHeight: true
                Text {
                    anchors.centerIn: parent
                    text: sensorRow.glyph
                    color: sensorRow.selected ? "#72c8ff" : "#9cabbc"
                    font.pixelSize: 17
                    font.bold: true
                }
                Rectangle {
                    visible: sensorRow.modified && panel.collapsed
                    width: 7; height: 7; radius: 4
                    color: "#f4b85c"
                    anchors.right: parent.right
                    anchors.rightMargin: panel.collapsed ? 10 : 0
                    anchors.top: parent.top
                    anchors.topMargin: 8
                }
            }
            ColumnLayout {
                visible: !panel.collapsed
                Layout.fillWidth: true
                spacing: 2
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 5
                    Text { text: sensorRow.title; color: "#e3eaf2"; font.pixelSize: 12; font.bold: sensorRow.selected; elide: Text.ElideRight; Layout.fillWidth: true }
                    Text { visible: sensorRow.modified; text: "MODIFIED"; color: "#f4b85c"; font.pixelSize: 8; font.bold: true; font.letterSpacing: .6 }
                }
                Text { text: sensorRow.subtitle; color: "#78889a"; font.pixelSize: 10; elide: Text.ElideRight; Layout.fillWidth: true }
                RowLayout {
                    visible: sensorRow.statusText !== ""
                    spacing: 5
                    Rectangle { width: 6; height: 6; radius: 3; color: sensorRow.statusColor }
                    Text { text: sensorRow.statusText; color: "#91a0b1"; font.pixelSize: 9; elide: Text.ElideRight; Layout.fillWidth: true }
                }
            }
        }
        MouseArea {
            id: rowMouse
            anchors.fill: parent
            enabled: sensorRow.actionable
            hoverEnabled: true
            cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
            onClicked: sensorRow.activated()
            ToolTip.visible: panel.collapsed && containsMouse
            ToolTip.text: sensorRow.title + (sensorRow.statusText ? " · " + sensorRow.statusText : "")
            ToolTip.delay: 450
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: panel.collapsed ? 5 : 10
        spacing: 7

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 34
            Text {
                visible: !panel.collapsed
                text: "SCENE INSPECTOR"
                color: "#dce6f1"
                font.pixelSize: 11
                font.bold: true
                font.letterSpacing: 1.1
                Layout.fillWidth: true
            }
            SceneIconButton {
                objectName: "sidebarToggle"
                Layout.preferredWidth: 32; Layout.preferredHeight: 32
                Layout.fillWidth: panel.collapsed
                Layout.alignment: panel.collapsed ? Qt.AlignHCenter : Qt.AlignRight
                glyph: panel.collapsed ? "›" : "‹"
                tooltipText: panel.collapsed ? "Expand scene inspector" : "Collapse scene inspector"
                onClicked: panel.collapsed = !panel.collapsed
            }
        }

        Rectangle {
            visible: !panel.collapsed
            Layout.fillWidth: true
            Layout.preferredHeight: 91
            radius: 7
            color: "#151c25"
            border.color: "#293746"
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 9; spacing: 2
                Text { text: panel.workspace.profileLabel; color: "#728399"; font.pixelSize: 9; font.bold: true; font.letterSpacing: .8; elide: Text.ElideRight; Layout.fillWidth: true }
                Text { text: panel.workspace.sequenceLabel; color: "#edf3f8"; font.pixelSize: 11; font.bold: true; elide: Text.ElideMiddle; Layout.fillWidth: true }
                Text { text: "Frame " + (panel.workspace.frameIndex + 1) + " of " + panel.workspace.frameCount; color: "#8e9eaf"; font.pixelSize: 10 }
                RowLayout { spacing: 5
                    Rectangle { width: 7; height: 7; radius: 4; color: panel.workspace.allSynchronized ? "#62d6a7" : "#f4b85c" }
                    Text { text: panel.workspace.allSynchronized ? "All sensors synchronized" : "Waiting for synchronized data"; color: panel.workspace.allSynchronized ? "#91cdb5" : "#d9ad6b"; font.pixelSize: 9 }
                }
            }
        }

        ScrollView {
            id: sensorScroll
            objectName: "sensorScroll"
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            contentWidth: availableWidth
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
            ColumnLayout {
                id: sensorColumn
                objectName: "sensorColumn"
                width: sensorScroll.availableWidth
                spacing: 4

                Text { visible: !panel.collapsed; text: "SENSORS"; color: "#66778a"; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1.1; Layout.topMargin: 2; Layout.leftMargin: 3 }
                SensorRow {
                    glyph: "⌁"
                    title: "LiDAR"
                    subtitle: panel.workspace.lidarPointCount.toLocaleString() + " points in current frame"
                    statusText: panel.workspace.buffering ? panel.workspace.bufferedFrameCount + " / " + panel.workspace.bufferTarget + " buffered" : "Frame ready"
                    statusColor: panel.workspace.buffering ? "#f4b85c" : "#62d6a7"
                    onActivated: panel.sceneRequested()
                }
                SensorRow {
                    glyph: "◇"
                    title: "IMU localization"
                    subtitle: "world-from-IMU pose"
                    statusText: panel.workspace.imuStatus
                    statusColor: panel.workspace.imuStatus === "Pose available" ? "#62d6a7" : "#e57373"
                    actionable: false
                }

                Text { visible: !panel.collapsed; text: "CAMERAS  ·  " + panel.workspace.cameraModels.length; color: "#66778a"; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1.1; Layout.topMargin: 9; Layout.leftMargin: 3 }
                Repeater {
                    model: panel.workspace.cameraModels
                    delegate: Item {
                        required property var modelData
                        Layout.fillWidth: true
                        Layout.preferredHeight: panel.collapsed ? 46 : 66
                        SensorRow {
                            anchors.fill: parent
                            glyph: "▣"
                            title: modelData.cameraId
                            subtitle: modelData.resolution
                            statusText: modelData.ready ? "Sync " + modelData.syncLabel : "Image unavailable"
                            statusColor: modelData.ready ? "#62d6a7" : "#e57373"
                            selected: modelData.selected
                            modified: modelData.modified
                            trailingSpace: modelData.selected ? 28 : 0
                            onActivated: panel.workspace.selectCamera(modelData.cameraId)
                        }
                        SceneIconButton {
                            visible: !panel.collapsed && modelData.selected
                            anchors.right: parent.right
                            anchors.rightMargin: 6
                            anchors.bottom: parent.bottom
                            anchors.bottomMargin: 5
                            width: 24; height: 24
                            glyph: "⚙︎"
                            tooltipText: "Calibrate " + modelData.cameraId
                            onClicked: panel.calibrationRequested(modelData.cameraId)
                        }
                    }
                }
            }
        }

        Text {
            visible: !panel.collapsed
            Layout.fillWidth: true
            text: panel.workspace.cameraModels.filter(function(camera) { return camera.modified }).length + " modified camera" + (panel.workspace.cameraModels.filter(function(camera) { return camera.modified }).length === 1 ? "" : "s")
            color: panel.workspace.dirty ? "#f4b85c" : "#66778a"
            font.pixelSize: 9
            horizontalAlignment: Text.AlignHCenter
        }
    }
}
