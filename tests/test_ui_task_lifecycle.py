import pytest

PySide6 = pytest.importorskip("PySide6")

from lidar_camera_calibrator.ui.bridge import WorkspaceBridge, _PreparationTask


def test_preparation_task_disables_qt_auto_delete_until_finished():
    task = _PreparationTask((0, "image_02", "medium", 0.08), object(), object())
    task.setAutoDelete(False)
    assert task.autoDelete() is False


def test_bridge_retains_and_releases_preparation_tasks():
    bridge = WorkspaceBridge()
    token = (0, "image_02", "medium", 0.08)
    task = object()
    bridge._prep_tasks[token] = task
    bridge._renderer_pending_token = token
    bridge._renderer_item = None

    bridge._on_prepared(token, RuntimeError("expected test result"))

    assert token not in bridge._prep_tasks


def test_bridge_shutdown_clears_retained_tasks():
    bridge = WorkspaceBridge()
    bridge._prep_tasks["pending"] = object()
    bridge.shutdown()
    assert bridge._prep_tasks == {}
