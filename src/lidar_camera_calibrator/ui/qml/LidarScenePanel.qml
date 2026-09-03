import QtQuick
import QtQuick.Layouts

Rectangle {
    id: scene
    required property var workspace
    property bool maximized: false
    property real yaw: -18
    property real pitch: 58
    property real zoom: 1
    property real panX: 0
    property real panY: 0
    property bool wheelNavigating: false
    signal toggleMaximize
    color: "#0e161e"
    border.color: "#293746"
    radius: Math.min(width, height) * 0.008
    clip: true

    function publishView(interactive) { workspace.setLidarView(yaw, pitch, zoom, panX, panY, viewport.width, viewport.height, interactive) }
    function requestInteractiveView(scheduleWheelSettle) {
        if (scheduleWheelSettle) settleUpdate.restart()
        if (!navigationUpdate.running) navigationUpdate.start()
    }
    function resetView() { yaw = -18; pitch = 58; zoom = 1; panX = 0; panY = 0; publishView(false) }
    Timer { id: navigationUpdate; interval: 16; repeat: false; onTriggered: scene.publishView(true) }
    Timer { id: settleUpdate; interval: 160; repeat: false; onTriggered: { scene.wheelNavigating = false; scene.publishView(false) } }
    Timer {
        id: viewportResolutionUpdate
        interval: 100
        repeat: false
        onTriggered: {
            if (viewport.width > 1 && viewport.height > 1)
                scene.publishView(false)
        }
    }
    Component.onCompleted: viewportResolutionUpdate.restart()

    ColumnLayout {
        anchors.fill: parent
        spacing: 0
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: Math.max(sceneTitle.implicitHeight * 2.6, scene.height * 0.065)
            color: "#151a21"
            RowLayout { anchors.fill: parent; anchors.leftMargin: parent.width * 0.018; anchors.rightMargin: parent.width * 0.014; spacing: parent.width * 0.01
                Text { text: "LIDAR"; color: "#8290a3"; font.pixelSize: sceneTitle.font.pixelSize * 0.76; font.bold: true; font.letterSpacing: 1.2; Layout.alignment: Qt.AlignVCenter }
                Text { id: sceneTitle; text: "3D scene"; color: "#f1f5f9"; font.bold: true; Layout.alignment: Qt.AlignVCenter }
                Text { text: workspace.lidarPointCount.toLocaleString() + " points" + (workspace.lidarRenderedCount < workspace.lidarPointCount ? " · preview " + workspace.lidarRenderedCount.toLocaleString() : ""); color: "#8290a3"; font.pixelSize: sceneTitle.font.pixelSize * 0.78; Layout.alignment: Qt.AlignVCenter }
                Item { Layout.fillWidth: true }
                Text { text: "Ctrl+drag orbit · Ctrl+Shift+drag pan · Wheel zoom · Point under cursor is pivot"; color: "#718096"; font.pixelSize: sceneTitle.font.pixelSize * 0.72; visible: scene.width > implicitWidth * 1.6; Layout.alignment: Qt.AlignVCenter }
                SceneIconButton { Layout.preferredHeight: parent.height * 0.82; Layout.preferredWidth: height; glyph: "⛶"; tooltipText: scene.maximized ? "Restore LiDAR panel" : "Maximize LiDAR panel"; onClicked: scene.toggleMaximize() }
            }
        }
        Item {
            id: viewport
            Layout.fillWidth: true
            Layout.fillHeight: true
            onWidthChanged: viewportResolutionUpdate.restart()
            onHeightChanged: viewportResolutionUpdate.restart()
            Image { anchors.fill: parent; source: workspace.lidarViewUrl; fillMode: Image.PreserveAspectFit; asynchronous: false; smooth: false }
            Rectangle { anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: Math.min(parent.width, parent.height) * 0.025; width: legendText.implicitWidth + height * 1.8; height: legendText.implicitHeight * 2.0; radius: height * 0.18; color: "#cc151a21"; border.color: "#344253"
                Rectangle { anchors.left: parent.left; anchors.leftMargin: parent.height * 0.40; anchors.verticalCenter: parent.verticalCenter; width: parent.height * 0.32; height: width; radius: width / 2; color: "#32ff78" }
                Text { id: legendText; anchors.left: parent.left; anchors.leftMargin: parent.height; anchors.verticalCenter: parent.verticalCenter; text: "Ego vehicle"; color: "#dce6f2" }
            }
            MouseArea {
                id: navigation
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton
                hoverEnabled: true
                preventStealing: true
                property bool navigating: false
                property bool panning: false
                property real lastX: 0
                property real lastY: 0
                onPressed: function(mouse) {
                    navigating = Boolean(mouse.modifiers & Qt.ControlModifier)
                    if (!navigating) { mouse.accepted = false; return }
                    navigationUpdate.stop(); settleUpdate.stop(); scene.wheelNavigating = false
                    panning = Boolean(mouse.modifiers & Qt.ShiftModifier)
                    if (!panning) workspace.beginLidarNavigation(mouse.x / Math.max(1, viewport.width), mouse.y / Math.max(1, viewport.height))
                    lastX = mouse.x; lastY = mouse.y; cursorShape = Qt.ClosedHandCursor
                }
                onPositionChanged: function(mouse) {
                    if (!navigating || !(mouse.buttons & Qt.LeftButton)) return
                    const dx = mouse.x - lastX
                    const dy = mouse.y - lastY
                    if (panning) {
                        scene.panX += dx / Math.max(1, viewport.width)
                        scene.panY += dy / Math.max(1, viewport.height)
                    } else {
                        scene.yaw += dx / Math.max(1, viewport.width) * 180
                        scene.pitch = Math.max(10, Math.min(88, scene.pitch + dy / Math.max(1, viewport.height) * 120))
                    }
                    lastX = mouse.x; lastY = mouse.y; scene.requestInteractiveView(false)
                }
                onReleased: { navigationUpdate.stop(); settleUpdate.stop(); navigating = false; cursorShape = Qt.ArrowCursor; scene.publishView(false) }
                onCanceled: { navigationUpdate.stop(); settleUpdate.stop(); navigating = false; cursorShape = Qt.ArrowCursor; scene.publishView(false) }
                onDoubleClicked: scene.resetView()
                onWheel: function(wheel) {
                    if (!scene.wheelNavigating) {
                        workspace.beginLidarNavigation(wheel.x / Math.max(1, viewport.width), wheel.y / Math.max(1, viewport.height))
                        scene.wheelNavigating = true
                    }
                    scene.zoom = Math.max(0.5, Math.min(12, scene.zoom * (wheel.angleDelta.y > 0 ? 1.15 : 1 / 1.15)))
                    scene.requestInteractiveView(true); wheel.accepted = true
                }
            }
        }
    }
}
