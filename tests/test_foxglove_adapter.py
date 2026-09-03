from __future__ import annotations

import base64
import io
import json

from mcap.writer import Writer
import numpy as np
from PIL import Image
import pytest

from lidar_camera_calibrator import SceneMcapReader, foxglove_source_config, write_profile_mcap
from lidar_camera_calibrator.profile import SourceDataError


def _stamp(value):
    return {"sec": int(value), "nsec": int((value - int(value)) * 1e9)}


def _foxglove_file(path, *, image_schema="foxglove.CompressedImage"):
    png = io.BytesIO()
    Image.new("RGB", (4, 3), (1, 2, 3)).save(png, format="PNG")
    points = np.array([[1, 2, 3, .5], [4, 5, 6, .7]], dtype="<f4")
    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start(profile="", library="test")
        schemas = {
            name: writer.register_schema(name=name, encoding="jsonschema", data=b"{}")
            for name in (
                "foxglove.FrameTransform", "foxglove.CameraCalibration",
                image_schema, "foxglove.PointCloud",
            )
        }
        channels = {
            "/tf": writer.register_channel("/tf", "json", schemas["foxglove.FrameTransform"]),
            "/camera/front/calibration": writer.register_channel(
                "/camera/front/calibration", "json", schemas["foxglove.CameraCalibration"]
            ),
            "/camera/front/image": writer.register_channel(
                "/camera/front/image", "json", schemas[image_schema]
            ),
            "/lidar/points": writer.register_channel(
                "/lidar/points", "json", schemas["foxglove.PointCloud"]
            ),
        }
        def add(topic, message, time):
            writer.add_message(
                channel_id=channels[topic], log_time=int(time * 1e9), publish_time=int(time * 1e9),
                data=json.dumps(message).encode(),
            )
        identity_tf = {
            "timestamp": _stamp(1.0),
            "translation": {"x": 0, "y": 0, "z": 0},
            "rotation": {"x": 0, "y": 0, "z": 0, "w": 1},
        }
        add("/tf", {**identity_tf, "parent_frame_id": "world", "child_frame_id": "imu"}, 1)
        add("/tf", {**identity_tf, "parent_frame_id": "imu", "child_frame_id": "lidar"}, 1)
        camera_tf = dict(identity_tf)
        camera_tf["translation"] = {"x": -1, "y": 0, "z": 0}
        add("/tf", {**camera_tf, "parent_frame_id": "lidar", "child_frame_id": "front"}, 1)
        add("/camera/front/calibration", {
            "timestamp": _stamp(1), "frame_id": "front", "width": 4, "height": 3,
            "K": [10, 0, 2, 0, 10, 1, 0, 0, 1],
            "R": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            "P": [10, 0, 2, 0, 0, 10, 1, 0, 0, 0, 1, 0],
        }, 1)
        for time in (1.0, 1.1):
            add("/lidar/points", {
                "timestamp": _stamp(time), "frame_id": "lidar", "point_stride": 16,
                "fields": [
                    {"name": name, "offset": index * 4, "type": 7}
                    for index, name in enumerate(("x", "y", "z", "intensity"))
                ],
                "data": base64.b64encode(points).decode(),
            }, time)
            add("/camera/front/image", {
                "timestamp": _stamp(time), "frame_id": "front", "format": "png",
                "data": base64.b64encode(png.getvalue()).decode(),
            }, time)
        writer.finish()


def test_foxglove_json_base64_adapter_round_trip(tmp_path):
    source = tmp_path / "foxglove.mcap"
    _foxglove_file(source)
    config = foxglove_source_config(source)
    assert [camera.name for camera in config.cameras] == ["front"]
    np.testing.assert_allclose(config.lidar.validated_frame(0)[1], [4, 5, 6, .7])
    # TF stores parent-from-child, therefore child camera-from-parent is +1 x.
    np.testing.assert_allclose(config.static_transforms[1].matrix[:3, 3], [1, 0, 0])

    output = write_profile_mcap(config, tmp_path / "scene.mcap")
    reader = SceneMcapReader(output)
    assert len(reader) == 2
    assert reader.frame(0).cameras["front"].image.shape == (3, 4, 3)


def test_foxglove_adapter_rejects_wrong_schema(tmp_path):
    source = tmp_path / "foxglove.mcap"
    _foxglove_file(source, image_schema="custom.Image")
    with pytest.raises(SourceDataError) as error:
        foxglove_source_config(source)
    assert error.value.code == "FOXGLOVE_SCHEMA_MISMATCH"
    assert error.value.path == "/camera/front/image"
