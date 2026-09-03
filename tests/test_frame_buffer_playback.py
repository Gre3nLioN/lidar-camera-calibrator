import time
from threading import Event

import numpy as np
import pytest

from lidar_camera_calibrator.app import BufferedFrameAdapter, WorkspaceController
from lidar_camera_calibrator.models import FrameBundle
from test_app_controller import FakeAdapter


class FiveFrameAdapter:
    def __init__(self, gate=None, timestamp_step=0.04):
        base = FakeAdapter()
        self.calibration = base.calibration
        self._source = base._frames[0]
        self.gate = gate
        self.timestamp_step = timestamp_step

    def __len__(self): return 5

    def frame(self, index):
        if index >= 2 and self.gate is not None:
            self.gate.wait(timeout=2)
        source = self._source
        return FrameBundle(index, index * self.timestamp_step, source.lidar_points, source.cameras)


def test_buffer_preloads_two_then_publishes_remaining_frames_in_order():
    gate = Event()
    buffer = BufferedFrameAdapter(FiveFrameAdapter(gate), frame_limit=5, preload_count=2)
    try:
        assert len(buffer) == 5
        assert buffer.buffered_indices == (0, 1)
        assert buffer.is_ready(0) and buffer.is_ready(1)
        assert not buffer.is_ready(2)
        gate.set()
        deadline = time.monotonic() + 2
        while buffer.buffered_count < 5 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert buffer.buffered_indices == (0, 1, 2, 3, 4)
        assert buffer.frame(4).frame_index == 4
    finally:
        gate.set()
        buffer.shutdown()


def test_timestamp_playback_stops_on_fifth_frame_and_restarts(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setenv("QT_QUICK_BACKEND", "software")
    QtGui = pytest.importorskip("PySide6.QtGui")
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtTest import QTest
    from lidar_camera_calibrator.ui.bridge import WorkspaceBridge

    app = QGuiApplication.instance() or QGuiApplication([])
    adapter = BufferedFrameAdapter(FiveFrameAdapter(timestamp_step=0.03), frame_limit=5, preload_count=5)
    bridge = WorkspaceBridge(WorkspaceController(adapter), cpu_overlay=True)
    try:
        assert bridge.playbackSpeed == 4.0
        bridge.togglePlay()
        QTest.qWait(350)
        assert bridge.frameIndex == 4
        assert bridge.playing is False

        bridge.togglePlay()
        assert bridge.frameIndex == 0
        assert bridge.playing is True
        bridge.togglePlay()
        paused = bridge.frameIndex
        QTest.qWait(100)
        assert bridge.frameIndex == paused
        assert bridge.playing is False
        assert app is not None
    finally:
        bridge.shutdown()
