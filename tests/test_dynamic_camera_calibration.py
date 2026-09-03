from __future__ import annotations

import json

import numpy as np
import pytest

from lidar_camera_calibrator.app import WorkspaceController
from lidar_camera_calibrator.profile import SceneMcapReader, write_profile_mcap
from test_profile_writer import _config


def _controller(tmp_path, names=("front", "rear")):
    scene = write_profile_mcap(_config(camera_names=names), tmp_path / "scene.mcap")
    reader = SceneMcapReader(scene)
    return reader, WorkspaceController(reader, validation_camera=names[0])


def test_extrinsic_offsets_are_independent_per_selected_camera(tmp_path):
    _, controller = _controller(tmp_path)
    controller.adjust_translation("x", .1)
    front_transform = controller.transform_for_camera("front")

    controller.select_camera("rear")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == 0
    controller.adjust_translation("x", .2)
    controller.adjust_rotation("yaw", 1.5)
    rear_transform = controller.transform_for_camera("rear")

    controller.select_camera("front")
    snapshot = controller.snapshot()
    assert snapshot.working_calibration.translation_offset_metres[0] == pytest.approx(.1)
    assert snapshot.working_calibration.rotation_offset_degrees[2] == 0
    assert front_transform[0, 3] == pytest.approx(.1)
    assert rear_transform[0, 3] == pytest.approx(.2)
    assert not np.array_equal(front_transform, rear_transform)


def test_undo_history_restores_the_camera_that_was_edited(tmp_path):
    _, controller = _controller(tmp_path)
    controller.adjust_translation("x", .1)
    controller.select_camera("rear")
    controller.adjust_translation("x", .2)
    controller.dispatch("undo")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == 0

    controller.select_camera("front")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == pytest.approx(.1)
    controller.dispatch("undo")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == 0
    controller.dispatch("redo")
    assert controller.snapshot().working_calibration.translation_offset_metres[0] == pytest.approx(.1)


def test_reset_commands_affect_only_the_selected_camera(tmp_path):
    _, controller = _controller(tmp_path)
    controller.dispatch("set_intrinsics_enabled", enabled=True)
    controller.adjust_translation("x", .1)
    controller.set_intrinsic("fx", 20)
    controller.select_camera("rear")
    controller.adjust_translation("x", .2)
    controller.set_intrinsic("fx", 30)
    controller.dispatch("reset_all")

    assert controller.transform_for_camera("rear")[0, 3] == 0
    assert controller.working.intrinsics("rear")[0, 0] == controller.loaded.cameras["rear"].intrinsics[0, 0]
    assert controller.transform_for_camera("front")[0, 3] == pytest.approx(.1)
    assert controller.working.intrinsics("front")[0, 0] == 20


def test_canonical_export_contains_only_changed_edges_with_explicit_directions(tmp_path):
    _, controller = _controller(tmp_path)
    controller.adjust_translation("x", .1)
    controller.select_camera("rear")
    controller.adjust_rotation("yaw", 2)
    controller.dispatch("set_intrinsics_enabled", enabled=True)
    controller.set_intrinsic("fx", 25)

    payload = json.loads(controller.copy_calibration())
    assert payload["profile_name"] == "lidar-camera-scene"
    assert payload["profile_version"] == 1
    assert set(payload["camera_edges"]) == {"front", "rear"}
    assert set(payload["intrinsics"]) == {"rear"}
    for camera_name, edge in payload["camera_edges"].items():
        assert edge["target"] == {"role": "camera", "camera_name": camera_name}
        assert edge["source"] == {"role": "lidar"}
        assert np.asarray(edge["matrix"]).shape == (4, 4)


def test_renderer_models_include_every_camera_and_selected_working_transform(tmp_path):
    names = ("front", "rear", "left", "right")
    _, controller = _controller(tmp_path, names)
    controller.select_camera("left")
    controller.adjust_translation("z", .3)
    renderer = controller.snapshot().renderer_input
    assert set(renderer.projection_model["camera_models"]) == set(names)
    assert renderer.camera_id == "left"
    assert renderer.projection_model["transform"][2, 3] == pytest.approx(.3)
    assert renderer.projection_model["camera_models"]["left"]["transform"][2, 3] == pytest.approx(.3)
    assert renderer.projection_model["camera_models"]["front"]["transform"][2, 3] == 0
