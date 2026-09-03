from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
QML_PATH = PROJECT_ROOT / "src/lidar_camera_calibrator/ui/qml/MainDisabled.qml"


@pytest.mark.parametrize(
    "camera_names",
    [
        ("front",),
        ("front", "rear"),
        ("front", "rear", "left", "right"),
    ],
)
def test_live_qml_builds_one_preview_and_source_route_per_profile_camera(
    tmp_path, camera_names
):
    script = f"""
import os, sys
from pathlib import Path
sys.path.insert(0, {str(PROJECT_ROOT / 'tests')!r})
from test_profile_writer import _config
from lidar_camera_calibrator.app import WorkspaceController
from lidar_camera_calibrator.profile import SceneMcapReader, write_profile_mcap
from lidar_camera_calibrator.ui.bridge import WorkspaceBridge
from lidar_camera_calibrator.ui.main import WorkspaceImageProvider, WorkspaceLidarImageProvider, WorkspaceOverlayImageProvider
from PySide6.QtCore import QObject
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
names = {camera_names!r}
scene = write_profile_mcap(_config(camera_names=names), Path({str(tmp_path)!r}) / 'scene.mcap')
reader = SceneMcapReader(scene)
controller = WorkspaceController(reader, validation_camera=names[0])
bridge = WorkspaceBridge(controller, cpu_overlay=True)
app = QGuiApplication([])
engine = QQmlApplicationEngine()
engine.addImageProvider('workspace', WorkspaceImageProvider(bridge))
engine.addImageProvider('workspace-overlay', WorkspaceOverlayImageProvider(bridge))
engine.addImageProvider('workspace-lidar', WorkspaceLidarImageProvider(bridge))
engine.rootContext().setContextProperty('workspaceBridge', bridge)
engine.load({str(QML_PATH)!r})
app.processEvents()
assert engine.rootObjects()
root = engine.rootObjects()[0]
repeater = root.findChild(QObject, 'dynamicCameraRepeater')
assert repeater is not None and repeater.property('count') == len(names)
assert tuple(bridge.cameraNames) == names
assert [model['cameraId'] for model in bridge.cameraModels] == list(names)
assert all(model['imageUrl'] for model in bridge.cameraModels)
assert all(model['resolution'] == '3 × 2' for model in bridge.cameraModels)
assert all(model['ready'] and model['syncLabel'].endswith(' ms') for model in bridge.cameraModels)
assert bridge.allSynchronized and bridge.imuStatus == 'Pose available'
bridge.adjustTranslation('x', 0.2)
assert bridge.cameraModels[0]['modified'] and bridge.cameraModels[0]['extrinsicsModified']
bridge.selectCamera(names[-1])
assert bridge.selectedCamera == names[-1] and controller.selected_camera == names[-1]
bridge.togglePreviewProjection()
assert all(bridge.preview_overlay_array(name) is not None for name in names)
assert all(bridge.previewOverlayUrl(name) for name in names)
print('DYNAMIC_CAMERA_QML_OK', len(names), flush=True)
os._exit(0)
"""
    environment = dict(os.environ)
    environment.update({
        "QT_QPA_PLATFORM": "offscreen",
        "QT_QUICK_BACKEND": "software",
        "LIDAR_CALIBRATOR_PREP_MODE": "sync",
    })
    completed = subprocess.run(
        [sys.executable, "-c", script], cwd=PROJECT_ROOT,
        env=environment, capture_output=True, text=True, timeout=60,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert f"DYNAMIC_CAMERA_QML_OK {len(camera_names)}" in completed.stdout
    assert not any(
        marker in completed.stderr
        for marker in ("ReferenceError", "TypeError", "Binding loop detected", "Cannot anchor", "Error:")
    ), completed.stderr
