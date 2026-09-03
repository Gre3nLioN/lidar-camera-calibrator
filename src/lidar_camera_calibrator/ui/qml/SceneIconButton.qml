import QtQuick
import QtQuick.Controls

Button {
    id: control
    property string glyph: ""
    property string tooltipText: ""
    readonly property bool pointerHovered: hitArea.containsMouse
    text: ""
    hoverEnabled: true
    implicitWidth: 28
    implicitHeight: 28
    leftPadding: width * 0.18; rightPadding: width * 0.18
    topPadding: height * 0.18; bottomPadding: height * 0.18
    contentItem: Text {
        text: control.glyph
        color: !control.enabled ? "#536173" : (control.pointerHovered ? "#50b8ff" : "#c7d2e0")
        opacity: !control.enabled ? 0.65 : 1.0
        font.pixelSize: Math.max(11, Math.min(control.width, control.height) * 0.42)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        Behavior on color { ColorAnimation { duration: 120 } }
    }
    background: Item {}
    ToolTip { visible: control.pointerHovered; delay: 500; text: control.tooltipText; x: control.width - implicitWidth; y: control.height * 1.1 }
    MouseArea {
        id: hitArea
        x: 0; y: 0
        width: control.width; height: control.height
        enabled: control.enabled
        hoverEnabled: true
        preventStealing: true
        cursorShape: Qt.PointingHandCursor
        onClicked: control.click()
    }
}
