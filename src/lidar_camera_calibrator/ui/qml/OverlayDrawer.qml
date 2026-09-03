import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: drawer
    property var workspace
    color: "#1b222c"
    implicitWidth: 292
    Column { anchors.fill: parent; anchors.margins: 18; spacing: 14
        Row { width: parent.width
            Text { text: "Overlay settings"; color: "#f1f5f9"; font.pixelSize: 17; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
            Item { width: Math.max(0, parent.width - 190); height: 1 }
            Button { text:"×"; onClicked: workspace.toggleOverlay(); ToolTip.visible: hovered; ToolTip.text: "Close overlay settings" }
        }
        Text { text: "LiDAR overlay"; color: "#8290a3"; font.pixelSize: 11 }
        Text { text: "APPEARANCE"; color: "#8290a3"; font.pixelSize:10; font.bold:true; font.letterSpacing:1.2 }
        RowLayout { width: parent.width; spacing: parent.width * 0.03
            Text { id: coloringLabel; text:"Coloring"; color:"#b7c2d0"; Layout.preferredWidth: parent.width * 0.30; Layout.alignment: Qt.AlignVCenter }
            Button { id: coloringSelector; Layout.fillWidth: true; Layout.preferredHeight: coloringLabel.implicitHeight * 2.4; padding: 0; onClicked: coloringMenu.open()
                contentItem: Item {
                    Text { anchors.left: parent.left; anchors.leftMargin: parent.width * 0.08; anchors.right: coloringArrow.left; anchors.rightMargin: parent.width * 0.05; anchors.verticalCenter: parent.verticalCenter; text: workspace.overlayColoring === "intensity" ? "Intensity" : "Depth"; color: "#e6edf5"; elide: Text.ElideRight }
                    Text { id: coloringArrow; anchors.right: parent.right; anchors.rightMargin: parent.width * 0.08; anchors.verticalCenter: parent.verticalCenter; text: "▼"; color: "#c7d2e0"; font.pixelSize: coloringLabel.font.pixelSize * 0.85 }
                }
                background: Rectangle { radius: height * 0.10; color: "#202833"; border.width: 1; border.color: coloringSelector.activeFocus ? "#50b8ff" : "#465569" }
                Menu { id: coloringMenu; y: coloringSelector.height; width: coloringSelector.width; padding: width * 0.03
                    background: Rectangle { color: "#202833"; border.width: 1; border.color: "#465569"; radius: coloringSelector.height * 0.10 }
                    MenuItem { id: depthItem; width: coloringMenu.availableWidth; implicitHeight: coloringLabel.implicitHeight * 2.2
                        contentItem: Text { text: "Depth"; color: "#e6edf5"; verticalAlignment: Text.AlignVCenter; leftPadding: coloringMenu.width * 0.06 }
                        background: Rectangle { color: depthItem.highlighted ? "#294765" : "transparent" }
                        onTriggered: workspace.setOverlaySetting("coloring", "depth") }
                    MenuItem { id: intensityItem; width: coloringMenu.availableWidth; implicitHeight: coloringLabel.implicitHeight * 2.2
                        contentItem: Text { text: "Intensity"; color: "#e6edf5"; verticalAlignment: Text.AlignVCenter; leftPadding: coloringMenu.width * 0.06 }
                        background: Rectangle { color: intensityItem.highlighted ? "#294765" : "transparent" }
                        onTriggered: workspace.setOverlaySetting("coloring", "intensity") }
                }
            }
        }
        Row { spacing: 8
            Text { text:"Point size"; color:"#b7c2d0"; width: 76; anchors.verticalCenter: parent.verticalCenter }
            Slider { id: pointSize; objectName: "overlayPointSize"; width:120; from:1; to:8; value:workspace.overlayPointSize; stepSize:1
                onMoved: workspace.setOverlaySetting("pointSizePx", value) }
            Text { text: Math.round(workspace.overlayPointSize) + " px"; color:"#d5deea"; anchors.verticalCenter: parent.verticalCenter }
        }
        Row { spacing: 8
            Text { text:"Opacity"; color:"#b7c2d0"; width:76; anchors.verticalCenter:parent.verticalCenter }
            Slider { id: opacity; objectName: "overlayOpacity"; width:110; from:0; to:1; value:workspace.overlayOpacity; stepSize:.01
                onMoved: workspace.setOverlaySetting("opacity", value) }
            Text { text: Math.round(workspace.overlayOpacity * 100) + "%"; color:"#d5deea"; anchors.verticalCenter:parent.verticalCenter }
        }
        Text { text: "Depth range"; color: "#b7c2d0"; font.pixelSize: 12 }
        Row { spacing: 10
            TextField { width: 78; text: Number(workspace.overlayDepthMin).toFixed(1); placeholderText:"min m"; onEditingFinished: workspace.setOverlaySetting("depthMinMetres", Number(text)) }
            Text { text:"—"; color:"#8290a3"; anchors.verticalCenter:parent.verticalCenter }
            TextField { width:78; text:Number(workspace.overlayDepthMax).toFixed(1); placeholderText:"max m"; onEditingFinished: workspace.setOverlaySetting("depthMaxMetres", Number(text)) }
        }
        Text { text: "RENDERING"; color: "#8290a3"; font.pixelSize:10; font.bold:true; font.letterSpacing:1.2 }
        RowLayout { width: parent.width; spacing: parent.width * 0.03
            Text { id: densityLabel; text:"Density"; color:"#b7c2d0"; Layout.preferredWidth: parent.width * 0.30; Layout.alignment: Qt.AlignVCenter }
            Button { id: densitySelector; Layout.fillWidth: true; Layout.preferredHeight: densityLabel.implicitHeight * 2.4; padding: 0; onClicked: densityMenu.open()
                contentItem: Item {
                    Text { anchors.left: parent.left; anchors.leftMargin: parent.width * 0.08; anchors.right: densityArrow.left; anchors.rightMargin: parent.width * 0.05; anchors.verticalCenter: parent.verticalCenter; text: workspace.overlayDensity === "light" ? "Light · 0.20 m" : workspace.overlayDensity === "full" ? "Full · 0.02 m" : "Medium · 0.08 m"; color: "#e6edf5"; elide: Text.ElideRight }
                    Text { id: densityArrow; anchors.right: parent.right; anchors.rightMargin: parent.width * 0.08; anchors.verticalCenter: parent.verticalCenter; text: "▼"; color: "#c7d2e0"; font.pixelSize: densityLabel.font.pixelSize * 0.85 }
                }
                background: Rectangle { radius: height * 0.10; color: "#202833"; border.width: 1; border.color: densitySelector.activeFocus ? "#50b8ff" : "#465569" }
                Menu { id: densityMenu; y: densitySelector.height; width: densitySelector.width; padding: width * 0.03
                    background: Rectangle { color: "#202833"; border.width: 1; border.color: "#465569"; radius: densitySelector.height * 0.10 }
                    MenuItem { id: lightItem; width: densityMenu.availableWidth; implicitHeight: densityLabel.implicitHeight * 2.2
                        contentItem: Text { text: "Light · 0.20 m"; color: "#e6edf5"; verticalAlignment: Text.AlignVCenter; leftPadding: densityMenu.width * 0.06 }
                        background: Rectangle { color: lightItem.highlighted ? "#294765" : "transparent" }
                        onTriggered: workspace.setOverlaySetting("renderDensity", "light") }
                    MenuItem { id: mediumItem; width: densityMenu.availableWidth; implicitHeight: densityLabel.implicitHeight * 2.2
                        contentItem: Text { text: "Medium · 0.08 m"; color: "#e6edf5"; verticalAlignment: Text.AlignVCenter; leftPadding: densityMenu.width * 0.06 }
                        background: Rectangle { color: mediumItem.highlighted ? "#294765" : "transparent" }
                        onTriggered: workspace.setOverlaySetting("renderDensity", "medium") }
                    MenuItem { id: fullItem; width: densityMenu.availableWidth; implicitHeight: densityLabel.implicitHeight * 2.2
                        contentItem: Text { text: "Full · 0.02 m"; color: "#e6edf5"; verticalAlignment: Text.AlignVCenter; leftPadding: densityMenu.width * 0.06 }
                        background: Rectangle { color: fullItem.highlighted ? "#294765" : "transparent" }
                        onTriggered: workspace.setOverlaySetting("renderDensity", "full") }
                }
            }
        }
        Text { text: "CPU mode always clips to the image · voxel downsampling only · candidate envelope ±5°"; color:"#718096"; font.pixelSize:10; wrapMode:Text.Wrap; width:parent.width }
    }
}
