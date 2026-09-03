from __future__ import annotations

import base64
import json
from pathlib import Path

from jsonschema import Draft202012Validator
from mcap.reader import make_reader
import numpy as np
import pytest

from lidar_camera_calibrator import write_profile_mcap
from lidar_camera_calibrator.profile import (
    CameraInput,
    EgoPoseStreamSpec,
    FrameRef,
    PointCloudInput,
    ProfileWriteError,
    SourceAdapterConfig,
    SourceDataError,
    SynchronizationError,
    TimedTransform,
    TransformSpec,
    load_schema,
)

_SCHEMA_RESOURCE_BY_TITLE = {
    load_schema(name)["title"]: load_schema(name)
    for name in (
        "scene-manifest", "static-transform-tree", "point-cloud-frame",
        "camera-calibration", "camera-image-frame", "ego-pose-frame", "scene-frame",
    )
}


def _config(camera_names=("front", "rear"), *, point_frames=None, camera_times=(0.49, 1.51)):
    lidar_times = (0.5, 1.5)
    point_frames = point_frames or (
        [[1, 2, 3, .5], [4, 5, 6, .6]],
        [[2, 3, 4, .7]],
    )
    cameras = tuple(
        CameraInput(
            name=name,
            timestamps=camera_times,
            images=(
                np.full((2, 3, 3), index, dtype=np.uint8),
                np.full((2, 3, 3), index + 1, dtype=np.uint8),
            ),
            image_format="numpy-rgb",
            intrinsics=np.array([[10 + index, 0, 1], [0, 11 + index, 1], [0, 0, 1.0]]),
            image_size=(3, 2),
        )
        for index, name in enumerate(camera_names)
    )
    first_pose = np.eye(4)
    last_pose = np.eye(4); last_pose[0, 3] = 2
    return SourceAdapterConfig(
        lidar=PointCloudInput(lidar_times, point_frames),
        cameras=cameras,
        static_transforms=(
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),
            *(TransformSpec(FrameRef.camera(camera.name), FrameRef.lidar(), np.eye(4)) for camera in cameras),
        ),
        imu=EgoPoseStreamSpec((TimedTransform(0, first_pose), TimedTransform(2, last_pose))),
    )


def _messages(path: Path):
    with path.open("rb") as stream:
        reader = make_reader(stream)
        header = reader.get_header()
        messages = list(reader.iter_messages())
    return header, messages


def test_writer_materializes_complete_schema_valid_scene_profile(tmp_path):
    output = write_profile_mcap(_config(), tmp_path / "scene.mcap")
    assert output == tmp_path / "scene.mcap"
    header, records = _messages(output)
    assert header.profile == "lidar-camera-scene/1"

    by_topic = {}
    for schema, channel, message in records:
        payload = json.loads(message.data)
        by_topic.setdefault(channel.topic, []).append(payload)
        assert schema is not None
        assert channel.message_encoding == "json"
        assert schema.encoding == "jsonschema"
        expected_schema = _SCHEMA_RESOURCE_BY_TITLE[schema.name]
        assert json.loads(schema.data) == expected_schema
        Draft202012Validator(expected_schema).validate(payload)

    assert len(by_topic["/scene/manifest"]) == 1
    assert len(by_topic["/scene/static_transforms"]) == 1
    assert len(by_topic["/scene/lidar/points"]) == 2
    assert len(by_topic["/scene/imu/pose"]) == 2
    assert len(by_topic["/scene/frames"]) == 2
    for camera in ("front", "rear"):
        assert len(by_topic[f"/scene/cameras/{camera}/calibration"]) == 1
        assert len(by_topic[f"/scene/cameras/{camera}/image"]) == 2

    manifest = by_topic["/scene/manifest"][0]
    assert manifest["frame_count"] == 2
    assert [camera["name"] for camera in manifest["cameras"]] == ["front", "rear"]

    cloud = by_topic["/scene/lidar/points"][0]
    points = np.frombuffer(base64.b64decode(cloud["data"]), dtype="<f4").reshape(cloud["shape"])
    np.testing.assert_allclose(points, [[1, 2, 3, .5], [4, 5, 6, .6]])

    first_pose = np.asarray(by_topic["/scene/imu/pose"][0]["world_from_imu"]).reshape(4, 4)
    second_pose = np.asarray(by_topic["/scene/imu/pose"][1]["world_from_imu"]).reshape(4, 4)
    assert first_pose[0, 3] == pytest.approx(.5)
    assert second_pose[0, 3] == pytest.approx(1.5)

    image = by_topic["/scene/cameras/front/image"][0]
    assert base64.b64decode(image["data"]).startswith(b"\x89PNG\r\n\x1a\n")
    assert image["synchronization_delta_ns"] == -10_000_000


def test_writer_supports_arbitrary_camera_count_without_fixed_names(tmp_path):
    names = ("front", "rear", "left", "right")
    output = write_profile_mcap(_config(names), tmp_path / "four-camera-scene.mcap")
    _, records = _messages(output)
    topics = {channel.topic for _, channel, _ in records}
    for name in names:
        assert f"/scene/cameras/{name}/image" in topics
        assert f"/scene/cameras/{name}/calibration" in topics


def test_writer_preserves_existing_destination_and_removes_temporary_on_bad_point(tmp_path):
    destination = tmp_path / "scene.mcap"
    destination.write_bytes(b"existing-valid-output")
    bad = _config(point_frames=(
        [[1, 2, 3, .5]],
        [[1, 2, 3]],
    ))
    with pytest.raises(SourceDataError, match="POINT_SHAPE_INVALID"):
        write_profile_mcap(bad, destination)
    assert destination.read_bytes() == b"existing-valid-output"
    assert list(tmp_path.glob(".scene.mcap.*.tmp")) == []


def test_writer_preserves_destination_when_selected_image_payload_is_malformed(tmp_path):
    valid = _config(camera_names=("front",))
    camera = valid.cameras[0]
    malformed_camera = CameraInput(
        name=camera.name,
        timestamps=camera.timestamps,
        images=(camera.images[0], np.zeros((3, 2, 3), dtype=np.uint8)),
        image_format=camera.image_format,
        intrinsics=camera.intrinsics,
        image_size=camera.image_size,
    )
    malformed = SourceAdapterConfig(
        lidar=valid.lidar,
        cameras=(malformed_camera,),
        static_transforms=valid.static_transforms,
        imu=valid.imu,
    )
    destination = tmp_path / "scene.mcap"
    destination.write_bytes(b"previous-profile")
    with pytest.raises(SourceDataError) as error:
        write_profile_mcap(malformed, destination)
    assert error.value.code == "IMAGE_SHAPE_INVALID"
    assert destination.read_bytes() == b"previous-profile"
    assert list(tmp_path.glob(".scene.mcap.*.tmp")) == []


def test_writer_fails_atomically_when_any_camera_is_unsynchronized(tmp_path):
    destination = tmp_path / "scene.mcap"
    with pytest.raises(SynchronizationError) as error:
        write_profile_mcap(_config(camera_times=(0.0, 1.0)), destination)
    assert error.value.code == "CAMERA_SYNC_MISSING"
    assert not destination.exists()
    assert list(tmp_path.glob(".scene.mcap.*.tmp")) == []


def test_writer_fails_atomically_when_imu_does_not_cover_lidar_timeline(tmp_path):
    valid = _config()
    destination = tmp_path / "scene.mcap"
    bad = SourceAdapterConfig(
        lidar=valid.lidar,
        cameras=valid.cameras,
        static_transforms=valid.static_transforms,
        imu=EgoPoseStreamSpec((TimedTransform(.6, np.eye(4)), TimedTransform(2, np.eye(4)))),
    )
    with pytest.raises(SynchronizationError) as error:
        write_profile_mcap(bad, destination)
    assert error.value.code == "IMU_SYNC_MISSING"
    assert not destination.exists()
    assert list(tmp_path.glob(".scene.mcap.*.tmp")) == []


def test_writer_requires_mcap_suffix_and_typed_config(tmp_path):
    with pytest.raises(ProfileWriteError, match="PROFILE_PATH_INVALID"):
        write_profile_mcap(_config(), tmp_path / "scene.bin")
    with pytest.raises(ProfileWriteError, match="PROFILE_CONFIG_INVALID"):
        write_profile_mcap({}, tmp_path / "scene.mcap")
