import QtQuick

Item {
    id: viewport
    required property var viewState
    property url imageSource
    property url overlaySource
    property string title: ""
    clip: true

    Rectangle { anchors.fill: parent; color: "#202a34" }

    Item {
        id: transformedLayer
        width: viewport.width
        height: viewport.height
        x: viewport.viewState.offsetX * viewport.width
        y: viewport.viewState.offsetY * viewport.height
        scale: viewport.viewState.scaleValue
        transformOrigin: Item.Center

        Image {
            anchors.fill: parent
            source: viewport.imageSource
            fillMode: Image.PreserveAspectFit
            asynchronous: true
            retainWhileLoading: true
            smooth: true
        }
        Image {
            anchors.fill: parent
            source: viewport.overlaySource
            fillMode: Image.PreserveAspectFit
            asynchronous: false
            retainWhileLoading: true
            smooth: false
            visible: source !== ""
        }
    }

    Rectangle {
        visible: viewport.title !== ""
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: Math.max(6, viewport.width * 0.015)
        width: paneLabel.implicitWidth + height
        height: paneLabel.implicitHeight * 1.9
        radius: height * 0.18
        color: "#cc151a21"
        border.color: "#465569"
        Text { id: paneLabel; anchors.centerIn: parent; text: viewport.title; color: "#e6edf5"; font.bold: true }
    }

    MouseArea {
        id: navigation
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton
        hoverEnabled: true
        preventStealing: true
        property bool panning: false
        property real lastX: 0
        property real lastY: 0

        onPressed: function(mouse) {
            panning = Boolean(mouse.modifiers & Qt.ControlModifier)
            if (!panning) {
                mouse.accepted = false
                return
            }
            lastX = mouse.x
            lastY = mouse.y
            cursorShape = Qt.ClosedHandCursor
        }
        onPositionChanged: function(mouse) {
            if (!panning || !(mouse.buttons & Qt.LeftButton)) return
            viewState.offsetX += (mouse.x - lastX) / Math.max(1, viewport.width)
            viewState.offsetY += (mouse.y - lastY) / Math.max(1, viewport.height)
            lastX = mouse.x
            lastY = mouse.y
        }
        onReleased: { panning = false; cursorShape = Qt.ArrowCursor }
        onCanceled: { panning = false; cursorShape = Qt.ArrowCursor }
        onWheel: function(wheel) {
            const oldScale = viewState.scaleValue
            const factor = wheel.angleDelta.y > 0 ? 1.15 : 1 / 1.15
            const newScale = Math.max(1, Math.min(8, oldScale * factor))
            if (newScale === oldScale) return
            const centerX = viewport.width / 2
            const centerY = viewport.height / 2
            const oldOffsetX = viewState.offsetX * viewport.width
            const oldOffsetY = viewState.offsetY * viewport.height
            const imageX = (wheel.x - centerX - oldOffsetX) / oldScale
            const imageY = (wheel.y - centerY - oldOffsetY) / oldScale
            viewState.offsetX = (wheel.x - centerX - imageX * newScale) / Math.max(1, viewport.width)
            viewState.offsetY = (wheel.y - centerY - imageY * newScale) / Math.max(1, viewport.height)
            viewState.scaleValue = newScale
            wheel.accepted = true
        }
    }
}
