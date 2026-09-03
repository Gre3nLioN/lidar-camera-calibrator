import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs

Rectangle {
    id: dialog
    property var workspace
    width: parent ? Math.min(440, Math.max(280, parent.width * 0.82)) : 440
    height: Math.min(parent ? parent.height * 0.90 : 600, Math.max(300, content.implicitHeight + 48))
    radius: 10
    color: "#1b222c"; border.color: "#3c4b5e"; border.width: 1
    FileDialog {
        id: saveDialog
        title: "Save calibration override"
        fileMode: FileDialog.SaveFile
        nameFilters: ["JSON files (*.json)"]
        defaultSuffix: "json"
        onAccepted: workspace.confirmExport(selectedFile)
        onRejected: workspace.cancelExport()
    }
    Column { id: content; anchors.fill: parent; anchors.margins: 24; spacing: 14
        Row { width: parent.width
            Text { text: "Export calibration override"; color: "#f1f5f9"; font.pixelSize: 18; font.bold: true }
            Item { width: Math.max(0, parent.width - 280); height:1 }
            Text { text:"×"; color:"#aab8c8"; font.pixelSize:22
                MouseArea { anchors.fill:parent; onClicked:workspace.cancelExport() }
            }
        }
        Text { text: "Applies to every frame in this sequence"; color: "#62d6a7"; font.pixelSize: 12 }
        Rectangle { width: parent.width; height: 1; color: "#303b49" }
        Text { text: "FORMAT"; color:"#8290a3"; font.pixelSize:10; font.bold:true; font.letterSpacing:1.2 }
        Text { text: "Direction-explicit camera edge transforms"; color:"#d5deea"; font.pixelSize:13 }
        Text { text: "OUTPUT"; color:"#8290a3"; font.pixelSize:10; font.bold:true; font.letterSpacing:1.2 }
        Text { text: "Choose a .json save location"; color:"#d5deea"; font.pixelSize:13 }
        Text { text: "Changed camera edges and intrinsics only.\nThe loaded scene profile remains unchanged."; color:"#8290a3"; font.pixelSize:11; lineHeight: 1.15 }
        Item { width: parent.width; height: 40
            Row { anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; spacing: 10
                Button { text:"Cancel"; onClicked:workspace.cancelExport() }
                Button { text:"Choose location…"; highlighted:true; onClicked:saveDialog.open() }
            }
        }
    }
}
