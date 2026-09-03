import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import LidarCalibrator.Rendering 1.0

Rectangle {
    id: panel
    property var workspace
    property bool maximized: false
    color: "#0d1117"
    signal toggleMaximize
    Column { anchors.fill: parent; spacing: 0
        Rectangle { height: 48; width: parent.width; color: "#151a21"
            RowLayout { id: toolbar; anchors.left: parent.left; anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; anchors.leftMargin: 16; anchors.rightMargin: 12; height: 34; spacing: 12
                Text { text: "CAMERA"; color: "#8290a3"; font.pixelSize: 10; font.bold: true; font.letterSpacing: 1.3; Layout.alignment: Qt.AlignVCenter }
                Text { text: workspace.selectedCamera; color: "#f1f5f9"; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignVCenter }
                Text { text: "· Synchronized · " + workspace.timestampLabel; color: "#8290a3"; font.pixelSize: 11; Layout.alignment: Qt.AlignVCenter }
                Item { Layout.fillWidth: true }
                Button { id: maximizeButton; Layout.preferredWidth: toolbar.height; Layout.preferredHeight: toolbar.height; text: ""; hoverEnabled: true
                    contentItem: Text { text: "⛶"; color: maximizeButton.hovered ? "#50b8ff" : "#c7d2e0"; opacity: maximizeButton.hovered ? 1.0 : 0.82; font.pixelSize: toolbar.height * 0.56; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; Behavior on color { ColorAnimation { duration: 120 } } }
                    background: Item {}
                    ToolTip { visible: maximizeButton.hovered; delay: 500; text: "Maximize camera panel"; x: maximizeButton.width - implicitWidth; y: maximizeButton.height * 1.15 }
                    onClicked: panel.toggleMaximize() }
                Button { id: settingsButton; Layout.preferredWidth: toolbar.height; Layout.preferredHeight: toolbar.height; text: ""; hoverEnabled: true
                    contentItem: Text { text: "⚙︎"; color: settingsButton.hovered ? "#50b8ff" : "#c7d2e0"; opacity: settingsButton.hovered ? 1.0 : 0.82; font.pixelSize: toolbar.height * 0.56; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; Behavior on color { ColorAnimation { duration: 120 } } }
                    background: Item {}
                    ToolTip { visible: settingsButton.hovered; delay: 500; text: "Overlay settings"; x: settingsButton.width - implicitWidth; y: settingsButton.height * 1.15 }
                    onClicked: workspace.toggleOverlay() }
            }
        }
        Item { width: parent.width; height: parent.height - 48
            Rectangle { anchors.fill: parent; anchors.margins: 26; color: "#202a34"; radius: 7; border.color: "#344253"
                // Placeholder image surface until the renderer supplies a texture.
                Rectangle { anchors.fill: parent; anchors.margins: 1; color: "#26333d"; radius: 6
                    gradient: Gradient {
                        GradientStop { position: 0; color: "#344754" }
                        GradientStop { position: 1; color: "#172129" }
                    }
                    Image { id: cameraImage; anchors.fill: parent; source: workspace.imageUrl; fillMode: Image.PreserveAspectFit; asynchronous: true; smooth: true }
                    // Keep the Qt scene-graph item in the painted image rectangle
                    // so projection pixels are not shifted by PreserveAspectFit letterboxing.
                    SceneGraphOverlayItem { id: overlay
                        x: (parent.width - cameraImage.paintedWidth) / 2
                        y: (parent.height - cameraImage.paintedHeight) / 2
                        width: cameraImage.paintedWidth; height: cameraImage.paintedHeight
                        visible: workspace.rendererReady && width > 0 && height > 0; opacity: .95
                        Component.onCompleted: workspace.attachRenderer(overlay)
                    }
                    Text { visible: workspace.imageUrl === ""; anchors.centerIn: parent; text: workspace.selectedCamera + "  ·  frame " + workspace.frameIndex; color: "#aab8c8"; font.pixelSize: 16 }
                    Rectangle { visible: workspace.startupError !== ""; anchors.centerIn: parent; width: Math.min(parent.width - 48, 620); height: errorText.implicitHeight + 38; radius: 8; color: "#3b2424"; border.color: "#b76b62"
                        Text { id: errorText; anchors.centerIn: parent; width: parent.width - 32; wrapMode: Text.Wrap; horizontalAlignment: Text.AlignHCenter; text: workspace.startupError; color: "#ffd0ca"; font.pixelSize: 13 }
                    }
                    Text { visible: !workspace.rendererReady && workspace.imageUrl !== ""; anchors.centerIn: parent; text: "Preparing projected overlay…"; color: "#aab8c8"; font.pixelSize: 13 }
                    Rectangle { visible: workspace.projectionStatus === "unavailable"; anchors.centerIn: parent; width: 300; height: 116; radius: 8; color: "#18212b"; border.color: "#865c55"
                        Column { anchors.centerIn: parent; spacing: 8; Text { text: "Projection unavailable"; color: "#ffb4a7"; font.bold: true; font.pixelSize: 15 }
                            Text { text: workspace.projectionMessage || "The image remains available; projected points are hidden."; color: "#b7c2d0"; width: 250; wrapMode: Text.Wrap; font.pixelSize: 11 }
                            Button { text: "Review calibration file"; onClicked: workspace.reviewCalibration() } }
                    }
                }
                Rectangle { visible: workspace.processing; anchors.top: parent.top; anchors.left: parent.left; anchors.margins: 14; width: 185; height: 28; radius: 14; color: "#233346"
                    Text { anchors.centerIn: parent; text: "Processing frame…"; color: "#78c8ff"; font.pixelSize: 11 } }
            }
        }
    }
}
