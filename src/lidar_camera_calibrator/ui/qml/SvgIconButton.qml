import QtQuick
import QtQuick.Controls

Button {
    id: control
    property url iconSource
    property string tooltipText: ""
    property real iconScale: 0.62
    property bool active: false

    hoverEnabled: true
    implicitWidth: 34
    implicitHeight: 34
    padding: 0
    leftInset: 0; rightInset: 0; topInset: 0; bottomInset: 0
    contentItem: Item {}
    background: Rectangle {
        color: "transparent"
        Image {
            anchors.centerIn: parent
            width: parent.width * control.iconScale
            height: parent.height * control.iconScale
            source: control.iconSource
            fillMode: Image.PreserveAspectFit
            smooth: true
            opacity: control.enabled ? (control.hovered || control.active ? 1.0 : 0.78) : 0.42
            Behavior on opacity { NumberAnimation { duration: 120 } }
        }
    }
    ToolTip {
        visible: control.hovered && control.tooltipText !== ""
        delay: 500
        text: control.tooltipText
        x: control.width - implicitWidth
        y: control.height * 1.15
    }
}
