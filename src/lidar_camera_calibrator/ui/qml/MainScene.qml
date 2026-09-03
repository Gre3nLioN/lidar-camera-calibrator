import QtQuick
import QtQuick.Layouts

Item {
    id: mainScene
    objectName: "mainScene"
    required property var workspace
    property string maximizedPanel: ""
    readonly property bool panelMaximized: maximizedPanel !== ""
    readonly property int cameraCount: workspace.cameraModels.length
    property var firstCameraDelegate: null
    readonly property int cameraColumns: cameraCount <= 1 ? 1 : (cameraCount <= 4 ? 2 : Math.ceil(Math.sqrt(cameraCount)))
    signal calibrateRequested(string cameraId)

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: Math.max(5, Math.min(parent.width, parent.height) * 0.009)
        spacing: Math.max(3, height * 0.008)

        LidarScenePanel {
            id: lidarPanel
            workspace: mainScene.workspace
            maximized: mainScene.maximizedPanel === "lidar"
            visible: mainScene.maximizedPanel === "" || maximized
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: mainScene.maximizedPanel === "" ? mainScene.height * 0.68 : mainScene.height
            onToggleMaximize: mainScene.maximizedPanel = maximized ? "" : "lidar"
        }

        GridLayout {
            id: cameraGrid
            objectName: "dynamicCameraGrid"
            visible: mainScene.maximizedPanel !== "lidar"
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: mainScene.maximizedPanel === "" ? mainScene.height * 0.30 : mainScene.height
            columns: mainScene.maximizedPanel === "" ? mainScene.cameraColumns : 1
            rowSpacing: Math.max(3, height * 0.012)
            columnSpacing: Math.max(3, width * 0.006)

            Repeater {
                id: cameraRepeater
                objectName: "dynamicCameraRepeater"
                // Camera identities are stable across frames. Using the changing
                // metadata list as the model recreated delegates and defeated
                // Image.retainWhileLoading, producing a black frame flash.
                model: mainScene.workspace.cameraNames.length
                onItemAdded: function(index, item) {
                    if (index === 0)
                        mainScene.firstCameraDelegate = item
                }
                onItemRemoved: function(index, item) {
                    if (item === mainScene.firstCameraDelegate)
                        mainScene.firstCameraDelegate = null
                }
                delegate: CameraPreview {
                    required property int index
                    readonly property var cameraModel: mainScene.workspace.cameraModels[index]
                    cameraId: cameraModel.cameraId
                    imageSource: cameraModel.imageUrl
                    overlaySource: cameraModel.overlayUrl
                    maximized: mainScene.maximizedPanel === cameraId
                    visible: mainScene.maximizedPanel === "" || maximized
                    Layout.row: maximized ? 0 : Math.floor(index / mainScene.cameraColumns)
                    Layout.column: maximized ? 0 : index % mainScene.cameraColumns
                    Layout.columnSpan: maximized ? 1 : 1
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    onToggleMaximize: mainScene.maximizedPanel = maximized ? "" : cameraId
                    onCalibrate: mainScene.calibrateRequested(cameraId)
                }
            }
        }
    }
}
