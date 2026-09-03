import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: panel
    property var workspace
    property bool maximized: false
    property int observedFrame: workspace ? workspace.frameIndex : -1
    property string observedCamera: workspace ? workspace.selectedCamera : ""
    color: "#0d1117"
    signal toggleMaximize
    function resetView() { sharedViewState.scaleValue = 1.0; sharedViewState.offsetX = 0.0; sharedViewState.offsetY = 0.0 }
    onObservedFrameChanged: resetView()
    onObservedCameraChanged: resetView()
    QtObject { id: sharedViewState; objectName: "sharedProjectionViewState"; property real scaleValue: 1.0; property real offsetX: 0.0; property real offsetY: 0.0 }
    Column { anchors.fill: parent; spacing: 0
        Rectangle { width: parent.width; height: 48; color: "#151a21"
            RowLayout { id: cameraHeaderRow; anchors.left: parent.left; anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; anchors.leftMargin: 14; anchors.rightMargin: 14; height: 34; spacing: 12
                Text { text: "CAMERA"; color: "#8290a3"; font.pixelSize: 10; font.bold: true; font.letterSpacing: 1.3; Layout.alignment: Qt.AlignVCenter }
                Text { text: workspace.selectedCamera; color: "#f1f5f9"; font.pixelSize: 14; font.bold: true; Layout.alignment: Qt.AlignVCenter }
                Text { text: workspace.cpuOverlayEnabled ? "· LiDAR overlay" : "· image underlay only"; color: "#f4b85c"; font.pixelSize: 11; Layout.alignment: Qt.AlignVCenter }
                Item { Layout.fillWidth: true }
                Text { id: navigationHint; text: "Ctrl+drag pan · Wheel zoom · Compare original / working"; color: "#718096"; font.pixelSize: 10; visible: panel.width > implicitWidth * 2.7; Layout.alignment: Qt.AlignVCenter }
                RowLayout { id: headerActions; property real iconSlotSize: cameraHeaderRow.height; readonly property real actionRowWidth: iconSlotSize * 3 + spacing * 2; Layout.minimumWidth: actionRowWidth; Layout.preferredWidth: actionRowWidth; Layout.maximumWidth: actionRowWidth; Layout.minimumHeight: iconSlotSize; Layout.preferredHeight: iconSlotSize; Layout.maximumHeight: iconSlotSize; Layout.alignment: Qt.AlignVCenter; spacing: cameraHeaderRow.height * 0.35
                    SvgIconButton { id: comparisonButton; objectName: "comparisonButton"; Layout.minimumWidth: headerActions.iconSlotSize; Layout.preferredWidth: headerActions.iconSlotSize; Layout.maximumWidth: headerActions.iconSlotSize; Layout.minimumHeight: headerActions.iconSlotSize; Layout.preferredHeight: headerActions.iconSlotSize; Layout.maximumHeight: headerActions.iconSlotSize; iconSource: "icons/compare-split.svg"; iconScale: 0.78; tooltipText: workspace.comparisonOpen ? "Close original comparison" : "Compare original and working"; active: workspace.comparisonOpen; onClicked: workspace.toggleComparison() }
                    Button { id: maximizeButton; implicitWidth: 34; implicitHeight: 34; Layout.minimumWidth: headerActions.iconSlotSize; Layout.preferredWidth: headerActions.iconSlotSize; Layout.maximumWidth: headerActions.iconSlotSize; Layout.minimumHeight: headerActions.iconSlotSize; Layout.preferredHeight: headerActions.iconSlotSize; Layout.maximumHeight: headerActions.iconSlotSize; text: ""; hoverEnabled: true
                    contentItem: Text { text: "⛶"; color: maximizeButton.hovered ? "#50b8ff" : "#c7d2e0"; opacity: maximizeButton.hovered ? 1.0 : 0.82; font.pixelSize: maximizeButton.height * 0.56; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; Behavior on color { ColorAnimation { duration: 120 } } }
                    background: Item {}
                    ToolTip { visible: maximizeButton.hovered; delay: 500; text: "Maximize camera panel"; x: maximizeButton.width - implicitWidth; y: maximizeButton.height * 1.15 }
                    onClicked: panel.toggleMaximize() }
                Button { id: settingsButton; implicitWidth: 34; implicitHeight: 34; Layout.minimumWidth: headerActions.iconSlotSize; Layout.preferredWidth: headerActions.iconSlotSize; Layout.maximumWidth: headerActions.iconSlotSize; Layout.minimumHeight: headerActions.iconSlotSize; Layout.preferredHeight: headerActions.iconSlotSize; Layout.maximumHeight: headerActions.iconSlotSize; text: ""; hoverEnabled: true
                    contentItem: Text { text: "⚙︎"; color: settingsButton.hovered ? "#50b8ff" : "#c7d2e0"; opacity: settingsButton.hovered ? 1.0 : 0.82; font.pixelSize: settingsButton.height * 0.56; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; Behavior on color { ColorAnimation { duration: 120 } } }
                    background: Item {}
                    ToolTip { visible: settingsButton.hovered; delay: 500; text: "Overlay settings"; x: settingsButton.width - implicitWidth; y: settingsButton.height * 1.15 }
                    onClicked: workspace.toggleOverlay() }
                }
            }
        }
        Item { width: parent.width; height: parent.height - 48
            Rectangle { id: projectionSurface; anchors.fill: parent; anchors.margins: Math.max(8, Math.min(parent.width, parent.height) * 0.03); color: "#111820"; radius: Math.min(width, height) * 0.008; clip: true
                RowLayout { anchors.fill: parent; spacing: workspace.comparisonOpen ? Math.max(2, width * 0.003) : 0
                    ProjectionViewport { objectName: "originalProjectionViewport"; visible: workspace.comparisonOpen; Layout.fillWidth: true; Layout.fillHeight: true; title: "Original"; viewState: sharedViewState; imageSource: workspace.imageUrl; overlaySource: workspace.originalOverlayUrl }
                    ProjectionViewport { objectName: "workingProjectionViewport"; Layout.fillWidth: true; Layout.fillHeight: true; title: workspace.comparisonOpen ? "Working" : ""; viewState: sharedViewState; imageSource: workspace.imageUrl; overlaySource: workspace.overlayUrl }
                }
                Text { visible: workspace.imageUrl === ""; anchors.centerIn: parent; text: "No camera image · frame " + workspace.frameIndex; color: "#aab8c8"; font.pixelSize: 16 }
                Rectangle { visible: !workspace.cpuOverlayEnabled; anchors.centerIn: parent; width: parent.width * 0.32; height: parent.height * 0.10; radius: height * 0.12; color: "#352d20"; border.color: "#8e6b39"
                    Text { anchors.centerIn: parent; text: workspace.diagnosticMessage; color: "#ffd18a"; font.pixelSize: 13 }
                }
                Rectangle { visible: workspace.cpuOverlayEnabled && (workspace.projectionStatus === "stale" || workspace.projectionStatus === "preparing"); anchors.centerIn: parent; width: parent.width * 0.32; height: parent.height * 0.10; radius: height * 0.12; color: "#352d20"; border.color: "#8e6b39"
                    Text { anchors.centerIn: parent; width: parent.width * 0.92; wrapMode: Text.Wrap; horizontalAlignment: Text.AlignHCenter; text: workspace.projectionMessage || "Refreshing LiDAR overlay coverage…"; color: "#ffd18a"; font.pixelSize: 12 }
                }
                Rectangle { visible: workspace.cpuOverlayEnabled && workspace.projectionStatus === "unavailable"; anchors.centerIn: parent; width: parent.width * 0.32; height: parent.height * 0.10; radius: height * 0.12; color: "#3b2424"; border.color: "#b76b62"
                    Text { anchors.centerIn: parent; width: parent.width * 0.92; wrapMode: Text.Wrap; horizontalAlignment: Text.AlignHCenter; text: workspace.projectionMessage; color: "#ffd0ca"; font.pixelSize: 12 }
                }
            }
            OverlayDrawer {
                id: cameraOverlayDrawer
                anchors.top: projectionSurface.top; anchors.bottom: projectionSurface.bottom; anchors.right: projectionSurface.right
                anchors.margins: Math.max(6, Math.min(projectionSurface.width, projectionSurface.height) * 0.015)
                width: Math.min(implicitWidth, projectionSurface.width * 0.42)
                workspace: panel.workspace
                visible: workspace.overlayOpen
                z: 10
            }
        }
    }
}
