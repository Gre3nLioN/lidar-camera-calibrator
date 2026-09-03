from __future__ import annotations

import sys
from threading import Event
import time

import numpy as np
import pytest

from lidar_camera_calibrator import (
    CalibrationConfig,
    CalibrationResult,
    CameraEdgeCalibration,
    FrameRef,
    SceneMcapReader,
    SourceAdapterConfig,
    CalibrationOverrideError, SceneCalibrationOverride, TransformSpec,
    launch_calibrator,
    write_profile_mcap,
)
from test_profile_writer import _config


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"frame_limit": 0}, "frame_limit"),
        ({"preload_count": -1}, "preload_count"),
        ({"renderer_mode": "gpu"}, "renderer_mode"),
        ({"playback_speed": 0}, "playback_speed"),
        ({"window_title": ""}, "window_title"),
        ({"initial_override": ""}, "initial_override"),
    ],
)
def test_calibration_config_rejects_invalid_public_options(kwargs, message):
    with pytest.raises(ValueError, match=message):
        CalibrationConfig(**kwargs)


def test_launch_calibrator_is_blocking_and_returns_final_camera_edge_results(tmp_path, monkeypatch):
    scene = write_profile_mcap(_config(), tmp_path / "scene.mcap")
    invocation = {}

    def fake_run(controller, *, renderer_mode, window_title):
        invocation.update(
            renderer_mode=renderer_mode,
            window_title=window_title,
            frame_count=controller.frame_count,
            playback_rate=controller.playback_rate,
        )
        controller.adjust_translation("x", 1.25)
        controller.dispatch("set_intrinsics_enabled", enabled=True)
        controller.set_intrinsic("fx", 42)
        return 0

    monkeypatch.setattr("lidar_camera_calibrator.ui.main.run_controller", fake_run)
    result = launch_calibrator(
        scene,
        CalibrationConfig(
            frame_limit=1, preload_count=1, renderer_mode="disabled",
            playback_speed=2.5, window_title="Calibration Test",
        ),
    )

    assert isinstance(result, CalibrationResult)
    assert invocation == {
        "renderer_mode": "disabled",
        "window_title": "Calibration Test",
        "frame_count": 1,
        "playback_rate": 2.5,
    }
    assert set(result.camera_edges) == {"front", "rear"}
    assert result.changed_cameras == ("front",)
    assert result.camera_edges["front"].matrix[0, 3] == pytest.approx(1.25)
    assert result.camera_edges["rear"].changed is False
    assert result.intrinsics["front"][0, 0] == pytest.approx(42)
    assert not result.matrices["front"].flags.writeable
    assert not result.intrinsics["front"].flags.writeable


def test_launch_returns_independent_updates_for_multiple_cameras(tmp_path, monkeypatch):
    scene = write_profile_mcap(_config(), tmp_path / "multi-camera.mcap")

    def fake_run(controller, **_):
        controller.adjust_translation("x", .5)
        controller.select_camera("rear")
        controller.adjust_translation("y", .75)
        return 0

    monkeypatch.setattr("lidar_camera_calibrator.ui.main.run_controller", fake_run)
    result = launch_calibrator(scene, CalibrationConfig(preload_count=0))
    assert result.changed_cameras == ("front", "rear")
    assert result.camera_edges["front"].matrix[0, 3] == pytest.approx(.5)
    assert result.camera_edges["front"].matrix[1, 3] == 0
    assert result.camera_edges["rear"].matrix[0, 3] == 0
    assert result.camera_edges["rear"].matrix[1, 3] == pytest.approx(.75)


def test_result_updates_inverse_oriented_camera_edge_in_its_declared_direction(tmp_path, monkeypatch):
    valid = _config(camera_names=("front",))
    reverse_edge = SourceAdapterConfig(
        lidar=valid.lidar,
        cameras=valid.cameras,
        static_transforms=(
            valid.static_transforms[0],
            TransformSpec(FrameRef.lidar(), FrameRef.camera("front"), np.eye(4)),
        ),
        imu=valid.imu,
    )
    scene = write_profile_mcap(reverse_edge, tmp_path / "inverse-edge.mcap")

    def fake_run(controller, **_):
        controller.adjust_translation("x", 1)
        return 0

    monkeypatch.setattr("lidar_camera_calibrator.ui.main.run_controller", fake_run)
    result = launch_calibrator(scene, CalibrationConfig(preload_count=0))
    edge = result.camera_edges["front"]
    assert edge.source == FrameRef.camera("front")
    assert edge.target == FrameRef.lidar()
    assert edge.matrix[0, 3] == pytest.approx(-1)


def test_export_reload_applies_all_modified_cameras_from_one_file(tmp_path, monkeypatch):
    scene_a = write_profile_mcap(_config(), tmp_path / "scene-a.mcap")
    source_reader = SceneMcapReader(scene_a)
    from lidar_camera_calibrator.app import WorkspaceController
    source_controller = WorkspaceController(source_reader, validation_camera="front")
    source_controller.adjust_translation("x", .4)
    source_controller.adjust_rotation("yaw", 3)
    source_controller.dispatch("set_intrinsics_enabled", enabled=True)
    source_controller.set_intrinsic("fx", 31)
    source_controller.select_camera("rear")
    source_controller.adjust_translation("y", -.2)
    override_path = tmp_path / "all-modified-cameras.json"
    override_path.write_text(source_controller.copy_calibration() + "\n", encoding="utf-8")

    loaded = SceneCalibrationOverride.load(override_path)
    assert set(loaded.camera_edges) == {"front", "rear"}
    assert set(loaded.intrinsics) == {"front"}

    # A second independently written scene with the same semantic graph accepts
    # the single multi-camera artifact as its initial working calibration.
    scene_b = write_profile_mcap(_config(), tmp_path / "scene-b.mcap")
    observed = {}

    def fake_run(controller, **_):
        snapshot = controller.snapshot()
        observed["dirty"] = snapshot.dirty
        observed["undo"] = snapshot.can_undo
        observed["intrinsics_enabled"] = snapshot.working_calibration.intrinsics_enabled
        observed["front"] = controller.transform_for_camera("front").copy()
        observed["rear"] = controller.transform_for_camera("rear").copy()
        return 0

    monkeypatch.setattr("lidar_camera_calibrator.ui.main.run_controller", fake_run)
    result = launch_calibrator(
        scene_b, CalibrationConfig(initial_override=override_path, preload_count=0)
    )
    assert observed == {
        "dirty": True,
        "undo": False,
        "intrinsics_enabled": True,
        "front": pytest.approx(source_controller.transform_for_camera("front")),
        "rear": pytest.approx(source_controller.transform_for_camera("rear")),
    }
    assert set(result.changed_cameras) == {"front", "rear"}
    assert result.intrinsics["front"][0, 0] == pytest.approx(31)


def test_initial_override_rejects_unknown_camera_and_edge_direction(tmp_path, monkeypatch):
    scene = write_profile_mcap(_config(), tmp_path / "scene.mcap")
    source = SceneMcapReader(scene)
    from lidar_camera_calibrator.app import WorkspaceController
    controller = WorkspaceController(source, validation_camera="front")
    controller.adjust_translation("x", .1)
    payload = __import__("json").loads(controller.copy_calibration())

    payload["camera_edges"]["unknown"] = payload["camera_edges"].pop("front")
    payload["camera_edges"]["unknown"]["target"]["camera_name"] = "unknown"
    unknown = tmp_path / "unknown.json"
    unknown.write_text(__import__("json").dumps(payload), encoding="utf-8")
    with pytest.raises(CalibrationOverrideError, match="absent from this scene"):
        launch_calibrator(scene, CalibrationConfig(initial_override=unknown))

    payload = __import__("json").loads(controller.copy_calibration())
    edge = payload["camera_edges"]["front"]
    edge["target"], edge["source"] = edge["source"], edge["target"]
    reversed_path = tmp_path / "reversed.json"
    reversed_path.write_text(__import__("json").dumps(payload), encoding="utf-8")
    with pytest.raises(CalibrationOverrideError, match="direction does not match"):
        launch_calibrator(scene, CalibrationConfig(initial_override=reversed_path))


@pytest.mark.skipif(sys.platform == "win32", reason="Windows intentionally uses synchronous decoding")
def test_reader_view_stages_ten_then_replenishes_rolling_forty_frame_cache():
    from lidar_camera_calibrator.api import _ReaderView

    release_after_ten = Event()

    class Reader:
        calibration = object()
        def __len__(self): return 80
        def frame(self, index):
            if index >= 10:
                release_after_ten.wait(timeout=2)
            time.sleep(.002)
            return {"frame": index}

    view = _ReaderView(Reader(), None, 2)
    try:
        deadline = time.monotonic() + 2
        while view.buffered_count < 10 and time.monotonic() < deadline:
            time.sleep(.01)
        assert view.buffered_count == 10
        assert view._capacity == 40
        assert view.buffer_target == 40

        release_after_ten.set()
        deadline = time.monotonic() + 2
        while view.buffered_count < 40 and time.monotonic() < deadline:
            time.sleep(.01)
        assert view.buffered_count == 40
        assert view.frame(0) == {"frame": 0}
        deadline = time.monotonic() + 2
        while not view.is_ready(40) and time.monotonic() < deadline:
            time.sleep(.01)
        assert view.frame(1) == {"frame": 1}
        assert view.frame(40) == {"frame": 40}
        assert len(view._cache) <= 40

        assert view.frame(60) == {"frame": 60}
        deadline = time.monotonic() + 2
        while not view.is_ready(79) and time.monotonic() < deadline:
            time.sleep(.01)
        assert view.frame(79) == {"frame": 79}
        assert view.buffered_count > 40
        assert view.was_loaded(0)
        assert not view._errors
        assert len(view._cache) <= 40
    finally:
        release_after_ten.set()
        view.shutdown()
    assert view._executor is None


def test_reader_view_surfaces_background_decode_failure():
    from lidar_camera_calibrator.api import _ReaderView

    class Reader:
        calibration = object()
        def __len__(self): return 20
        def frame(self, index):
            if index == 10:
                raise ValueError("bad frame payload")
            return {"frame": index}

    view = _ReaderView(Reader(), None, 2)
    try:
        deadline = time.monotonic() + 2
        while not view.is_ready(10) and time.monotonic() < deadline:
            time.sleep(.01)
        with pytest.raises(RuntimeError, match="failed to decode scene frame 10") as error:
            view.frame(10)
        assert isinstance(error.value.__cause__, ValueError)
    finally:
        view.shutdown()


def test_reader_view_preserves_windows_synchronous_safety(monkeypatch):
    import lidar_camera_calibrator.api as api

    class Reader:
        calibration = object()
        def __len__(self): return 24
        def frame(self, index): return {"frame": index}

    monkeypatch.setattr(api.sys, "platform", "win32")
    view = api._ReaderView(Reader(), None, 2)
    try:
        assert view._executor is None
        assert view.buffer_target == 24
        assert view.buffered_count == 24
        assert view.is_ready(18)
        assert view.frame(18) == {"frame": 18}
        assert len(view._cache) <= 10
    finally:
        view.shutdown()


def test_launch_rejects_non_config_before_starting_ui(tmp_path):
    scene = write_profile_mcap(_config(), tmp_path / "scene.mcap")
    with pytest.raises(TypeError, match="CalibrationConfig"):
        launch_calibrator(scene, {})


def test_public_result_models_defensively_copy_matrices():
    original = np.eye(4)
    changed = np.eye(4); changed[0, 3] = 1
    edge = CameraEdgeCalibration("front", FrameRef.camera("front"), FrameRef.lidar(), original, changed)
    result = CalibrationResult({"front": edge}, {"front": np.eye(3)})
    original[0, 0] = 99
    changed[0, 3] = 99
    assert result.camera_edges["front"].original_matrix[0, 0] == 1
    assert result.camera_edges["front"].matrix[0, 3] == 1
    assert result.changed_cameras == ("front",)
