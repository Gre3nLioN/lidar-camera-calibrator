import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: timeline
    property var workspace
    color: "#151a21"
    implicitHeight: 96

    ColumnLayout {
        anchors.fill: parent
        anchors.leftMargin: parent.width * 0.014
        anchors.rightMargin: parent.width * 0.014
        anchors.topMargin: parent.height * 0.12
        anchors.bottomMargin: parent.height * 0.14
        spacing: parent.height * 0.09

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: timeline.height * 0.25
            spacing: timeline.width * 0.012

            Text {
                text: "FRAME " + (workspace.frameIndex + 1) + " / " + workspace.frameCount
                color: "#e7edf5"
                font.pixelSize: Math.max(10, parent.height * 0.48)
                font.bold: true
                font.letterSpacing: 0.7
                Layout.alignment: Qt.AlignVCenter
            }
            Rectangle { Layout.preferredWidth: 1; Layout.preferredHeight: parent.height * 0.55; color: "#344253" }
            Text {
                text: workspace.timestampLabel
                color: "#8290a3"
                font.pixelSize: Math.max(10, parent.height * 0.46)
                Layout.alignment: Qt.AlignVCenter
            }
            Item { Layout.fillWidth: true }
            Text {
                text: workspace.playbackSpeed.toFixed(0) + "×"
                color: "#aab8c8"
                font.pixelSize: Math.max(10, parent.height * 0.46)
                Layout.alignment: Qt.AlignVCenter
            }
            Text {
                text: workspace.playbackWaiting ? "Waiting for frame…" : (workspace.buffering ? "Buffered " + workspace.bufferedFrameCount + "/" + workspace.bufferTarget : (workspace.playing ? "Playing" : (workspace.safePreparationMode ? "Safe preparation (may pause)" : "Ready")))
                color: workspace.playbackWaiting || workspace.buffering ? "#f4b85c" : "#62d6a7"
                font.pixelSize: Math.max(10, parent.height * 0.46)
                Layout.alignment: Qt.AlignVCenter
            }
        }

        RowLayout {
            id: transportStrip
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: timeline.width * 0.012

            Button {
                id: playButton
                objectName: "timelinePlayButton"
                Layout.preferredWidth: Math.max(34, transportStrip.height * 0.92)
                Layout.fillHeight: true
                hoverEnabled: true
                text: workspace.playing ? "Ⅱ" : "▶"
                onClicked: workspace.togglePlay()
                contentItem: Text {
                    text: playButton.text
                    color: playButton.hovered ? "#dff4ff" : "#e7edf5"
                    font.pixelSize: Math.max(13, timeline.height * 0.17)
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle {
                    radius: height * 0.16
                    color: playButton.hovered ? "#26384a" : "#202833"
                    border.width: 1
                    border.color: playButton.hovered ? "#4b7599" : "#344253"
                }
            }

            Item {
                id: track
                Layout.fillWidth: true
                Layout.fillHeight: true
                Rectangle {
                    anchors.fill: parent
                    radius: height * 0.12
                    color: "#111820"
                    border.width: 1
                    border.color: "#29313d"
                }
                Rectangle {
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.leftMargin: parent.width * 0.02
                    anchors.rightMargin: parent.width * 0.02
                    height: Math.max(2, parent.height * 0.10)
                    radius: height / 2
                    color: "#344253"
                }
                Rectangle {
                    anchors.left: parent.left
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.leftMargin: parent.width * 0.02
                    width: workspace.frameCount <= 1 ? 0 : (parent.width * 0.96) * Math.max(0, workspace.bufferedFrameCount - 1) / (workspace.frameCount - 1)
                    height: Math.max(2, parent.height * 0.10)
                    radius: height / 2
                    color: "#315f55"
                }
                Repeater {
                    objectName: "timelineRepeater"
                    model: workspace.frameCount
                    delegate: Item {
                        id: marker
                        objectName: "timelineFrameMarker"
                        property int frameNumber: modelData
                        property bool buffered: workspace.bufferedFrameCount >= 0 && workspace.frameBuffered(frameNumber)
                        width: Math.max(20, track.height * 0.58)
                        height: track.height
                        x: workspace.frameCount <= 1 ? (track.width - width) / 2 : (track.width * 0.02) + frameNumber * ((track.width * 0.96) - width) / (workspace.frameCount - 1)
                        Rectangle {
                            anchors.centerIn: parent
                            width: marker.frameNumber === workspace.frameIndex ? parent.height * 0.48 : parent.height * 0.22
                            height: marker.frameNumber === workspace.frameIndex ? parent.height * 0.72 : parent.height * 0.22
                            radius: width / 2
                            color: marker.frameNumber === workspace.frameIndex ? "#50b8ff" : (marker.buffered ? "#55d99a" : "#596575")
                            border.width: marker.frameNumber === workspace.frameIndex ? Math.max(1, width * 0.10) : 0
                            border.color: "#d9f2ff"
                            Behavior on color { ColorAnimation { duration: 100 } }
                        }
                        MouseArea { id: markerMouse; anchors.fill: parent; hoverEnabled: true; cursorShape: Qt.PointingHandCursor; onClicked: workspace.selectFrame(marker.frameNumber) }
                        ToolTip { visible: markerMouse.containsMouse; text: marker.buffered ? "Frame " + (marker.frameNumber + 1) : "Frame " + (marker.frameNumber + 1) + " · buffering" }
                    }
                }
            }
        }
    }
}
