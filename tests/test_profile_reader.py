from __future__ import annotations

import base64
import json
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
import numpy as np
import pytest

from lidar_camera_calibrator.profile import (
    FrameRef,
    ProfileValidationError,
    ProfileVersionError,
    SceneMcapReader,
    write_profile_mcap,
)
from test_profile_writer import _config


def _rewrite(
    source: Path,
    destination: Path,
    *,
    profile: str | None = None,
    topic_map=None,
    schema_for_topic=None,
    keep_message=None,
    mutate_payload=None,
    duplicate_message=None,
):
    with source.open("rb") as stream:
        reader = make_reader(stream)
        header = reader.get_header()
        summary = reader.get_summary()
        records = list(reader.iter_messages())
    assert summary is not None
    with destination.open("wb") as stream:
        writer = Writer(stream)
        writer.start(profile=header.profile if profile is None else profile, library="reader-negative-test")
        schema_ids = {
            old_id: writer.register_schema(schema.name, schema.encoding, schema.data)
            for old_id, schema in summary.schemas.items()
        }
        channel_ids = {}
        for old_id, channel in summary.channels.items():
            topic = topic_map(channel.topic) if topic_map else channel.topic
            selected_schema = (
                schema_for_topic(channel.topic, channel.schema_id, summary)
                if schema_for_topic else channel.schema_id
            )
            channel_ids[old_id] = writer.register_channel(
                topic, channel.message_encoding, schema_ids[selected_schema], dict(channel.metadata)
            )
        for _, channel, message in records:
            if keep_message and not keep_message(channel.topic, message):
                continue
            data = message.data
            if mutate_payload:
                data = mutate_payload(channel.topic, message, data)
            repeats = 2 if duplicate_message and duplicate_message(channel.topic, message) else 1
            for _ in range(repeats):
                writer.add_message(
                    channel_ids[channel.id], message.log_time, data,
                    message.publish_time, message.sequence,
                )
        writer.finish()


def _scene(tmp_path: Path) -> Path:
    return write_profile_mcap(_config(), tmp_path / "scene.mcap")


def test_reader_round_trips_normalized_frames_pose_calibration_and_transforms(tmp_path):
    reader = SceneMcapReader(_scene(tmp_path))
    assert len(reader) == 2
    assert reader.camera_names == ("front", "rear")
    assert reader.calibration.dataset == "lidar-camera-scene"
    assert set(reader.calibration.cameras) == {"front", "rear"}

    frame = reader.frame(0)
    assert frame.frame_index == 0
    assert frame.timestamp == pytest.approx(.5)
    np.testing.assert_allclose(frame.lidar_points, [[1, 2, 3, .5], [4, 5, 6, .6]])
    assert set(frame.cameras) == {"front", "rear"}
    assert frame.cameras["front"].image.shape == (2, 3, 3)
    assert not frame.lidar_points.flags.writeable
    assert not frame.cameras["front"].image.flags.writeable

    pose = reader.ego_pose(1)
    assert pose[0, 3] == pytest.approx(1.5)
    assert not pose.flags.writeable
    np.testing.assert_allclose(
        reader.transform_between(FrameRef.camera("front"), FrameRef.imu()), np.eye(4)
    )
    edge_index, edge = reader.calibration_edge("front")
    assert edge_index == 1
    assert FrameRef.camera("front") in {edge.target, edge.source}


def test_reader_fetches_all_frame_sensor_payloads_in_one_mcap_pass(tmp_path, monkeypatch):
    reader = SceneMcapReader(_scene(tmp_path))
    import lidar_camera_calibrator.profile.reader as reader_module
    original = reader_module.make_reader
    calls = []

    def counted(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(reader_module, "make_reader", counted)
    frame = reader.frame(0)
    assert set(frame.cameras) == {"front", "rear"}
    assert len(calls) == 1


def test_reader_uses_schema_named_optional_projection_matrix(tmp_path):
    source = _scene(tmp_path)
    projected = tmp_path / "projected.mcap"
    projection = [9, 0, 1, -4, 0, 8, 2, 3, 0, 0, 1, 0]

    def mutate(topic, message, data):
        if topic == "/scene/cameras/front/calibration":
            payload = json.loads(data)
            payload["projection_matrix"] = projection
            payload["rectification"] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            return json.dumps(payload).encode()
        return data

    _rewrite(source, projected, mutate_payload=mutate)
    reader = SceneMcapReader(projected)
    np.testing.assert_allclose(
        reader.calibration.cameras["front"].projection_matrix,
        np.asarray(projection).reshape(3, 4),
    )


def test_reader_indexes_without_decoding_dynamic_base64_payloads(tmp_path):
    source = _scene(tmp_path)
    corrupt = tmp_path / "corrupt-points.mcap"

    def mutate(topic, message, data):
        if topic == "/scene/lidar/points" and message.sequence == 0:
            payload = json.loads(data); payload["data"] = "%%%not-base64%%%"
            return json.dumps(payload).encode()
        return data

    _rewrite(source, corrupt, mutate_payload=mutate)
    reader = SceneMcapReader(corrupt)
    assert len(reader) == 2
    with pytest.raises(ProfileValidationError) as error:
        reader.frame(0)
    assert error.value.code == "PROFILE_BASE64_INVALID"
    assert error.value.path == "lidar.frames[0].data"


def test_reader_rejects_unsupported_header_profile(tmp_path):
    source = _scene(tmp_path)
    invalid = tmp_path / "version-2.mcap"
    _rewrite(source, invalid, profile="lidar-camera-scene/2")
    with pytest.raises(ProfileVersionError) as error:
        SceneMcapReader(invalid)
    assert error.value.code == "PROFILE_VERSION_UNSUPPORTED"
    assert error.value.path == "header.profile"


def test_reader_rejects_missing_and_duplicate_dynamic_messages(tmp_path):
    source = _scene(tmp_path)
    missing = tmp_path / "missing-image.mcap"
    _rewrite(
        source, missing,
        keep_message=lambda topic, message: not (
            topic == "/scene/cameras/rear/image" and message.sequence == 1
        ),
    )
    with pytest.raises(ProfileValidationError) as error:
        SceneMcapReader(missing)
    assert error.value.code == "PROFILE_FRAME_MESSAGES_INCOMPLETE"
    assert error.value.path == "/scene/cameras/rear/image"

    duplicate = tmp_path / "duplicate-points.mcap"
    _rewrite(
        source, duplicate,
        duplicate_message=lambda topic, message: topic == "/scene/lidar/points" and message.sequence == 0,
    )
    with pytest.raises(ProfileValidationError) as error:
        SceneMcapReader(duplicate)
    assert error.value.code == "PROFILE_FRAME_MESSAGES_INCOMPLETE"
    assert error.value.path == "/scene/lidar/points"


def test_reader_rejects_renamed_topic_and_wrong_schema(tmp_path):
    source = _scene(tmp_path)
    renamed = tmp_path / "renamed-topic.mcap"
    _rewrite(
        source, renamed,
        topic_map=lambda topic: "/scene/lidar/cloud" if topic == "/scene/lidar/points" else topic,
    )
    with pytest.raises(ProfileValidationError) as error:
        SceneMcapReader(renamed)
    assert error.value.code == "PROFILE_TOPIC_UNEXPECTED"
    assert error.value.path == "/scene/lidar/cloud"

    wrong_schema = tmp_path / "wrong-schema.mcap"

    def select_schema(topic, original, summary):
        if topic != "/scene/lidar/points":
            return original
        return next(
            schema_id for schema_id, schema in summary.schemas.items()
            if schema.name == "lidar-camera.SceneFrame"
        )

    _rewrite(source, wrong_schema, schema_for_topic=select_schema)
    with pytest.raises(ProfileValidationError) as error:
        SceneMcapReader(wrong_schema)
    assert error.value.code == "PROFILE_SCHEMA_IDENTITY_INVALID"
    assert error.value.path == "/scene/lidar/points"


def test_reader_rejects_malformed_frame_index_before_opening_viewer(tmp_path):
    source = _scene(tmp_path)
    invalid = tmp_path / "bad-frame-index.mcap"

    def mutate(topic, message, data):
        if topic == "/scene/frames" and message.sequence == 1:
            payload = json.loads(data); payload.pop("timestamp_ns")
            return json.dumps(payload).encode()
        return data

    _rewrite(source, invalid, mutate_payload=mutate)
    with pytest.raises(ProfileValidationError) as error:
        SceneMcapReader(invalid)
    assert error.value.code == "PROFILE_MESSAGE_SCHEMA_INVALID"
    assert error.value.path == "/scene/frames[1]"


@pytest.mark.parametrize(
    ("topic_to_mutate", "mutation", "expected_code"),
    [
        (
            "/scene/frames",
            lambda payload: payload["cameras"][0].__setitem__("synchronization_delta_ns", 123),
            "PROFILE_SYNC_DELTA_INVALID",
        ),
        (
            "/scene/cameras/front/calibration",
            lambda payload: payload["intrinsics"].__setitem__(0, -1),
            "PROFILE_INTRINSICS_INVALID",
        ),
    ],
)
def test_reader_rejects_semantically_invalid_schema_valid_metadata(
    tmp_path, topic_to_mutate, mutation, expected_code
):
    source = _scene(tmp_path)
    invalid = tmp_path / f"{expected_code.lower()}.mcap"

    def mutate(topic, message, data):
        if topic == topic_to_mutate and message.sequence == 0:
            payload = json.loads(data)
            mutation(payload)
            return json.dumps(payload).encode()
        return data

    _rewrite(source, invalid, mutate_payload=mutate)
    with pytest.raises(ProfileValidationError) as error:
        SceneMcapReader(invalid)
    assert error.value.code == expected_code


def test_reader_rejects_truncated_or_non_mcap_input(tmp_path):
    invalid = tmp_path / "not-scene.mcap"
    invalid.write_bytes(b"not an mcap")
    with pytest.raises(ProfileValidationError) as error:
        SceneMcapReader(invalid)
    assert error.value.code == "PROFILE_CONTAINER_INVALID"
