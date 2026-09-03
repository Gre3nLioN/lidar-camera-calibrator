import numpy as np
import pytest

PySide6 = pytest.importorskip("PySide6")
from PySide6.QtCore import QSize
from PySide6.QtGui import QImage

from lidar_camera_calibrator.ui.main import WorkspaceImageProvider, WorkspaceLidarImageProvider, WorkspaceOverlayImageProvider


class _Bridge:
    def image_array(self, camera_id=None):
        return np.zeros((3, 4, 3), dtype=np.uint8)


def test_image_provider_returns_qimage_not_pyqt_tuple():
    size = QSize()
    image = WorkspaceImageProvider(_Bridge()).requestImage("current", size, QSize())
    assert isinstance(image, QImage)
    assert image.size() == QSize(4, 3)
    assert size == QSize(4, 3)


def test_lidar_provider_returns_scene_image():
    class LidarBridge:
        def lidar_scene_array(self): return np.zeros((5, 7, 3), dtype=np.uint8)

    image = WorkspaceLidarImageProvider(LidarBridge()).requestImage("current", QSize(), QSize())
    assert image.size() == QSize(7, 5)


def test_overlay_provider_routes_original_and_working_layers():
    class OverlayBridge:
        def __init__(self): self.requested = []
        def overlay_array(self, original=False):
            self.requested.append(original)
            return np.zeros((2, 3, 4), dtype=np.uint8)
        def preview_overlay_array(self, camera_id):
            self.requested.append(camera_id)
            return np.zeros((2, 3, 4), dtype=np.uint8)

    bridge = OverlayBridge()
    provider = WorkspaceOverlayImageProvider(bridge)
    assert provider.requestImage("current?revision=1", QSize(), QSize()).size() == QSize(3, 2)
    assert provider.requestImage("original?revision=1", QSize(), QSize()).size() == QSize(3, 2)
    assert provider.requestImage("preview-image_03?revision=1", QSize(), QSize()).size() == QSize(3, 2)
    assert bridge.requested == [False, True, "image_03"]
