import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: inspector
    objectName: "calibrationInspector"
    color: "#151a21"
    property var workspace
    property bool extrinsicsExpanded: true
    property bool intrinsicsExpanded: true
    signal openOverlay
    implicitWidth: 360

    ScrollView {
        id: inspectorScroll
        anchors.fill: parent
        anchors.margins: 18
        rightPadding: width * 0.06
        clip: true
        contentWidth: availableWidth
        Column {
            width: inspectorScroll.availableWidth
            spacing: 12

            RowLayout {
                width: parent.width
                spacing: 10
                Text { text: "CALIBRATION"; color: "#8290a3"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.4 }
                Rectangle { visible: workspace.dirty; width: 74; height: 22; radius: 11; color: "#45351c"
                    Text { anchors.centerIn: parent; text: "● UNSAVED"; color: "#f4b85c"; font.pixelSize: 10; font.bold: true } }
                Item { Layout.fillWidth: true }
            }
            RowLayout {
                width: parent.width
                Text { text: "LiDAR → " + workspace.selectedCamera; color: "#f0f4f8"; font.pixelSize: 18; font.bold: true }
                Item { Layout.fillWidth: true }
                SceneIconButton { id: undoButton; objectName: "undoButton"; Layout.preferredWidth: 30; Layout.preferredHeight: 30; glyph: "↶"; tooltipText: "Undo (Ctrl+Z)"; enabled: workspace.canUndo; onClicked: workspace.undo() }
                SceneIconButton { id: redoButton; objectName: "redoButton"; Layout.preferredWidth: 30; Layout.preferredHeight: 30; glyph: "↷"; tooltipText: "Redo (Ctrl+Shift+Z)"; enabled: workspace.canRedo; onClicked: workspace.redo() }
            }
            Text { text: "Working override · applies to every frame"; color: "#8290a3"; font.pixelSize: 12 }
            Rectangle { width: parent.width; height: 1; color: "#29313d" }

            Row {
                width: parent.width; height: 28
                Button { id: extrinsicsHeader; objectName: "extrinsicsHeader"; width: parent.width - resetExtrinsicsButton.width - 4; height: parent.height; padding: 0; hoverEnabled: true
                    background: Rectangle { color: extrinsicsHeader.hovered ? "#1c2632" : "transparent"; radius: 3 }
                    contentItem: Item {
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "EXTRINSICS"; color: "#8290a3"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.2 }
                        Text { anchors.right: parent.right; anchors.rightMargin: 12; anchors.verticalCenter: parent.verticalCenter; text: "▶"; rotation: inspector.extrinsicsExpanded ? 90 : 0; color: "#b7c2d0"; font.pixelSize: 10 }
                    }
                    onClicked: inspector.extrinsicsExpanded = !inspector.extrinsicsExpanded }
                SceneIconButton { id: resetExtrinsicsButton; width: 28; height: parent.height; glyph: "↺"; tooltipText: "Reset extrinsics"; onClicked: workspace.resetExtrinsics() }
            }
            Column {
                width: parent.width; spacing: 8; visible: inspector.extrinsicsExpanded
                Repeater { model: [{k:"x", label:"X", unit:"m", min:-5, max:5, scale:1000, precision:3}, {k:"y", label:"Y", unit:"m", min:-5, max:5, scale:1000, precision:3}, {k:"z", label:"Z", unit:"m", min:-5, max:5, scale:1000, precision:3}, {k:"roll", label:"Roll", unit:"°", min:-45, max:45, scale:100, precision:2}, {k:"pitch", label:"Pitch", unit:"°", min:-45, max:45, scale:100, precision:2}, {k:"yaw", label:"Yaw", unit:"°", min:-45, max:45, scale:100, precision:2}]
                    delegate: Row { id: extrinsicRow; width: parent.width; spacing: 8; property string key: modelData.k
                        function publish(value) { modelData.unit === "m" ? workspace.adjustTranslation(key, value / modelData.scale) : workspace.adjustRotation(key, value / modelData.scale) }
                        Text { id: extrinsicLabel; width: Math.max(36, parent.width * 0.12); text: modelData.label; color: "#b7c2d0"; anchors.verticalCenter: parent.verticalCenter }
                        Slider { id: slider; objectName: "extrinsicSlider_" + extrinsicRow.key; width: Math.max(48, parent.width - extrinsicLabel.width - valueSpin.width - extrinsicUnit.width - resetValueButton.width - parent.spacing * 4); from: modelData.min; to: modelData.max; value: workspace.parameter(extrinsicRow.key); stepSize: modelData.unit === "m" ? .001 : .01
                            onPressedChanged: pressed ? workspace.beginCalibrationChange() : workspace.endCalibrationChange()
                            onMoved: extrinsicRow.publish(Math.round(value * modelData.scale))
 }
                        SpinBox { id: valueSpin; objectName: "extrinsicSpin_" + extrinsicRow.key; width: Math.max(112, parent.width * 0.20); height: Math.max(32, Math.min(40, parent.width * 0.07)); from: Math.round(modelData.min * modelData.scale); to: Math.round(modelData.max * modelData.scale); stepSize: modelData.scale; value: Math.round(workspace.parameter(extrinsicRow.key) * modelData.scale); editable: true; font.pixelSize: 12
                            textFromValue: function(value, locale) { return (value / modelData.scale).toFixed(modelData.precision) }
                            valueFromText: function(text, locale) { return Math.round(Number(text) * modelData.scale) }
                            onValueModified: extrinsicRow.publish(value)
                            contentItem: TextInput {
                                anchors.left: parent.left; anchors.leftMargin: parent.width * 0.28
                                anchors.right: parent.right; anchors.rightMargin: parent.width * 0.28
                                anchors.verticalCenter: parent.verticalCenter
                                text: valueSpin.textFromValue(valueSpin.value, valueSpin.locale)
                                color: "#e7edf5"; selectByMouse: true; clip: true
                                horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter
                                font: valueSpin.font
                                validator: DoubleValidator { bottom: modelData.min; top: modelData.max; decimals: modelData.precision }
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: { valueSpin.value = valueSpin.valueFromText(text, valueSpin.locale); extrinsicRow.publish(valueSpin.value) }
                            } }
                        Text { id: extrinsicUnit; width: Math.max(18, parent.width * 0.05); text:modelData.unit; color:"#718096"; anchors.verticalCenter: parent.verticalCenter }
                        SceneIconButton { id: resetValueButton; objectName: "resetExtrinsicParameter_" + extrinsicRow.key; width: Math.max(26, parent.width * 0.06); height: width; glyph: "↺"; tooltipText: "Reset " + modelData.label; onClicked: workspace.resetParameter(extrinsicRow.key) }
                        Connections { target: workspace; function onSnapshotChanged() { slider.value = workspace.parameter(extrinsicRow.key); valueSpin.value = Math.round(workspace.parameter(extrinsicRow.key) * modelData.scale) } }
                    }
                }
            }

            Row {
                width: parent.width; height: 28
                Button { id: intrinsicsHeader; objectName: "intrinsicsHeader"; width: parent.width - resetIntrinsicsButton.width - 4; height: parent.height; padding: 0; hoverEnabled: true
                    background: Rectangle { color: intrinsicsHeader.hovered ? "#1c2632" : "transparent"; radius: 3 }
                    contentItem: Item {
                        Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; text: "INTRINSICS"; color: "#8290a3"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.2 }
                        Text { anchors.right: parent.right; anchors.rightMargin: 12; anchors.verticalCenter: parent.verticalCenter; text: "▶"; rotation: inspector.intrinsicsExpanded ? 90 : 0; color: "#b7c2d0"; font.pixelSize: 10 }
                    }
                    onClicked: inspector.intrinsicsExpanded = !inspector.intrinsicsExpanded }
                SceneIconButton { id: resetIntrinsicsButton; width: 28; height: parent.height; glyph: "↺"; tooltipText: "Reset intrinsics"; onClicked: workspace.resetIntrinsics() }
            }
            Column {
                width: parent.width; spacing: 8; visible: inspector.intrinsicsExpanded
                CheckBox { text: "Enable intrinsics editing"; checked: workspace.intrinsicsEnabled; onToggled: workspace.setIntrinsicsEnabled(checked); font.pixelSize:11 }
                Repeater { model: [{k:"fx", label:"fx", min:1, max:5000}, {k:"fy", label:"fy", min:1, max:5000}, {k:"cx", label:"cx", min:0, max:5000}, {k:"cy", label:"cy", min:0, max:5000}]
                    delegate: Row { id: intrinsicRow; width: parent.width; spacing: 8; property string key: modelData.k
                        function publish(value) { workspace.setIntrinsic(key, value / 100) }
                        Text { id: intrinsicLabel; width: Math.max(36, parent.width * 0.12); text: modelData.label; color: workspace.intrinsicsEnabled ? "#b7c2d0" : "#536173"; anchors.verticalCenter: parent.verticalCenter }
                        Slider { id: intrinsicSlider; width: Math.max(48, parent.width - intrinsicLabel.width - intrinsicSpin.width - intrinsicUnit.width - resetIntrinsicValue.width - parent.spacing * 4); from:modelData.min; to:modelData.max; value: workspace.parameter(intrinsicRow.key); stepSize: .01; enabled: workspace.intrinsicsEnabled
                            onPressedChanged: pressed ? workspace.beginCalibrationChange() : workspace.endCalibrationChange()
                            onMoved: intrinsicRow.publish(Math.round(value * 100))
 }
                        SpinBox { id: intrinsicSpin; width: Math.max(112, parent.width * 0.20); height: Math.max(32, Math.min(40, parent.width * 0.07)); from: modelData.min * 100; to: modelData.max * 100; stepSize: 100; value: Math.round(workspace.parameter(intrinsicRow.key) * 100); editable: true; enabled: workspace.intrinsicsEnabled; font.pixelSize: 12
                            textFromValue: function(value, locale) { return (value / 100).toFixed(2) }
                            valueFromText: function(text, locale) { return Math.round(Number(text) * 100) }
                            onValueModified: intrinsicRow.publish(value)
                            contentItem: TextInput {
                                anchors.left: parent.left; anchors.leftMargin: parent.width * 0.28
                                anchors.right: parent.right; anchors.rightMargin: parent.width * 0.28
                                anchors.verticalCenter: parent.verticalCenter
                                text: intrinsicSpin.textFromValue(intrinsicSpin.value, intrinsicSpin.locale)
                                color: intrinsicSpin.enabled ? "#e7edf5" : "#536173"; selectByMouse: true; clip: true
                                horizontalAlignment: TextInput.AlignHCenter; verticalAlignment: TextInput.AlignVCenter
                                font: intrinsicSpin.font
                                validator: DoubleValidator { bottom: modelData.min; top: modelData.max; decimals: 2 }
                                inputMethodHints: Qt.ImhFormattedNumbersOnly
                                onEditingFinished: { intrinsicSpin.value = intrinsicSpin.valueFromText(text, intrinsicSpin.locale); intrinsicRow.publish(intrinsicSpin.value) }
                            } }
                        Text { id: intrinsicUnit; width: Math.max(18, parent.width * 0.05); text:"px"; color:"#718096"; anchors.verticalCenter: parent.verticalCenter }
                        SceneIconButton { id: resetIntrinsicValue; objectName: "resetIntrinsicParameter_" + intrinsicRow.key; width: Math.max(26, parent.width * 0.06); height: width; glyph: "↺"; tooltipText: "Reset " + modelData.label; enabled: workspace.intrinsicsEnabled; onClicked: workspace.resetParameter(intrinsicRow.key) }
                        Connections { target: workspace; function onSnapshotChanged() { intrinsicSlider.value = workspace.parameter(intrinsicRow.key); intrinsicSpin.value = Math.round(workspace.parameter(intrinsicRow.key) * 100) } }
                    }
                }
            }

            Rectangle { width: parent.width; height: 1; color: "#29313d" }
            RowLayout { width: parent.width; spacing: width * 0.03
                Button { id: resetAllButton; Layout.fillWidth: true; hoverEnabled: true; text: "Reset all"; onClicked: workspace.resetAll()
                    contentItem: Text { text: resetAllButton.text; color: "#d6e0ec"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    background: Rectangle { radius: height * 0.10; color: resetAllButton.hovered ? "#26384a" : "#202833"; border.width: 1; border.color: resetAllButton.hovered ? "#4b7599" : "#344253" } }
                Button { id: exportButton; Layout.fillWidth: true; hoverEnabled: true; text: "Export JSON"; onClicked: workspace.requestExport()
                    contentItem: Text { text: exportButton.text; color: "#f4f8fc"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    background: Rectangle { radius: height * 0.10; color: exportButton.hovered ? "#238cd1" : "#176da5"; border.width: 1; border.color: exportButton.hovered ? "#65c4ff" : "#2f8ec5" } }
            }
        }
    }
}
