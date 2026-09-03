from __future__ import annotations

from copy import deepcopy
from io import BytesIO

import numpy as np
import pytest
from PIL import Image

from lidar_camera_calibrator.profile import (
    CameraInput,
    ConfigurationError,
    EgoPoseStreamSpec,
    FrameRef,
    PointCloudInput,
    SourceAdapterConfig,
    SourceDataError,
    SynchronizationConfig,
    TimedTransform,
    TransformGraphError,
    TransformSpec,
    calibration_edge_for_camera,
)


def _camera(name="front"):
    return CameraInput(
        name=name,
        timestamps=(0.49, 1.49),
        images=(np.zeros((2, 3, 3), dtype=np.uint8), np.ones((2, 3, 3), dtype=np.uint8)),
        image_format="numpy-rgb",
        intrinsics=np.array([[10, 0, 1], [0, 11, 1], [0, 0, 1.0]]),
        image_size=(3, 2),
    )


def _imu():
    first = np.eye(4)
    second = np.eye(4); second[0, 3] = 2
    return EgoPoseStreamSpec((TimedTransform(0, first), TimedTransform(2, second)))


def _valid_mapping():
    return {
        "lidar": {"timestamps": [0.5], "frames": [[[1, 2, 3, .5]]]},
        "cameras": [{
            "name": "front", "timestamps": [0.49],
            "images": [np.zeros((2, 3, 3), dtype=np.uint8)],
            "image_format": "numpy-rgb",
            "intrinsics": [[10, 0, 1], [0, 11, 1], [0, 0, 1]],
            "image_size": [3, 2],
        }],
        "static_transforms": [
            {"target": {"role": "lidar"}, "source": {"role": "imu"}, "matrix": np.eye(4)},
            {"target": {"role": "camera", "camera_name": "front"}, "source": {"role": "lidar"}, "matrix": np.eye(4)},
        ],
        "imu": {"samples": [
            {"timestamp": 0, "matrix": np.eye(4)},
            {"timestamp": 1, "matrix": np.eye(4)},
        ]},
    }


def _config(cameras=None, transforms=None):
    cameras = tuple(cameras or (_camera(),))
    transforms = tuple(transforms or (
        TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),
        *(TransformSpec(FrameRef.camera(camera.name), FrameRef.lidar(), np.eye(4)) for camera in cameras),
    ))
    return SourceAdapterConfig(
        lidar=PointCloudInput((0.5, 1.5), (
            [[1, 2, 3, .5]], [[2, 3, 4, .6]],
        )),
        cameras=cameras,
        static_transforms=transforms,
        imu=_imu(),
    )


def test_valid_config_supports_arbitrary_camera_array_and_readonly_geometry():
    cameras = tuple(_camera(f"camera_{index}") for index in range(4))
    config = _config(cameras)
    assert len(config.cameras) == 4
    assert config.synchronization == SynchronizationConfig()
    assert not config.static_transforms[0].matrix.flags.writeable
    assert config.lidar.validated_frame(0).shape == (1, 4)
    assert not config.cameras[0].validated_image(0).flags.writeable


def test_xyz_input_adds_zero_intensity_and_rejects_nonfinite_or_wrong_shape():
    lidar = PointCloudInput((0,), ([[1, 2, 3]],), format="xyz-f32")
    np.testing.assert_array_equal(lidar.validated_frame(0), [[1, 2, 3, 0]])
    with pytest.raises(SourceDataError) as error:
        PointCloudInput((0,), ([[1, 2, 3]],)).validated_frame(0)
    assert error.value.code == "POINT_SHAPE_INVALID"
    with pytest.raises(SourceDataError, match="POINT_NONFINITE"):
        PointCloudInput((0,), ([[1, 2, np.nan, 1]],)).validated_frame(0)


def test_camera_payload_shape_dtype_and_name_are_strict():
    camera = _camera()
    bad_shape = CameraInput(
        "front", (0,), (np.zeros((3, 2, 3), dtype=np.uint8),), "numpy-rgb",
        camera.intrinsics, (3, 2),
    )
    with pytest.raises(SourceDataError, match="IMAGE_SHAPE_INVALID"):
        bad_shape.validated_image(0)
    bad_dtype = CameraInput(
        "front", (0,), (np.zeros((2, 3, 3), dtype=np.float32),), "numpy-rgb",
        camera.intrinsics, (3, 2),
    )
    with pytest.raises(SourceDataError, match="IMAGE_DTYPE_INVALID"):
        bad_dtype.validated_image(0)
    with pytest.raises(ConfigurationError, match="CAMERA_NAME_INVALID"):
        CameraInput("front/camera", (0,), (np.zeros((2, 3, 3), dtype=np.uint8),), "numpy-rgb", camera.intrinsics, (3, 2))


def test_missing_image_payload_and_empty_required_streams_fail_early():
    camera = _camera()
    with pytest.raises(ConfigurationError) as mismatch:
        CameraInput(
            "front", (0, 1), (np.zeros((2, 3, 3), dtype=np.uint8),),
            "numpy-rgb", camera.intrinsics, (3, 2),
        )
    assert mismatch.value.code == "SOURCE_LENGTH_MISMATCH"
    assert mismatch.value.path == "cameras['front'].images"

    with pytest.raises(ConfigurationError, match="LIDAR_EMPTY"):
        PointCloudInput((), ())
    with pytest.raises(ConfigurationError, match="CAMERA_EMPTY"):
        CameraInput("front", (), (), "numpy-rgb", camera.intrinsics, (3, 2))
    valid = _config()
    with pytest.raises(ConfigurationError, match="CAMERAS_EMPTY"):
        SourceAdapterConfig(
            lidar=valid.lidar, cameras=(), static_transforms=valid.static_transforms, imu=valid.imu
        )


def test_corrupt_compressed_images_and_wrong_configured_encoding_are_rejected():
    intrinsics = _camera().intrinsics
    png_stream = BytesIO()
    Image.new("RGB", (3, 2), color=(1, 2, 3)).save(png_stream, format="PNG")
    valid_png = png_stream.getvalue()
    png_camera = CameraInput("front", (0,), (valid_png,), "png", intrinsics, (3, 2))
    assert png_camera.validated_image(0) == valid_png

    corrupt = CameraInput(
        "front", (0,), (b"\x89PNG\r\n\x1a\ncorrupt",), "png", intrinsics, (3, 2)
    )
    with pytest.raises(SourceDataError) as decode_error:
        corrupt.validated_image(0)
    assert decode_error.value.code == "IMAGE_DECODE_FAILED"

    wrong_encoding = CameraInput("front", (0,), (valid_png,), "jpeg", intrinsics, (3, 2))
    with pytest.raises(SourceDataError) as encoding_error:
        wrong_encoding.validated_image(0)
    assert encoding_error.value.code == "IMAGE_ENCODING_INVALID"


def test_invalid_intrinsics_shape_focal_lengths_and_homogeneous_row_are_rejected():
    image = (np.zeros((2, 3, 3), dtype=np.uint8),)
    with pytest.raises(ConfigurationError) as shape:
        CameraInput("front", (0,), image, "numpy-rgb", np.eye(4), (3, 2))
    assert shape.value.code == "MATRIX_SHAPE_INVALID"
    assert shape.value.path == "cameras['front'].intrinsics"

    for intrinsics in (
        [[-1, 0, 1], [0, 10, 1], [0, 0, 1]],
        [[10, 0, 1], [0, 0, 1], [0, 0, 1]],
        [[10, 0, 1], [0, 10, 1], [0, 1, 1]],
    ):
        with pytest.raises(ConfigurationError, match="INTRINSICS_INVALID"):
            CameraInput("front", (0,), image, "numpy-rgb", intrinsics, (3, 2))


def test_negative_timestamps_and_unsupported_formats_are_rejected():
    with pytest.raises(ConfigurationError, match="TIMESTAMP_INVALID"):
        PointCloudInput((-0.1,), ([[1, 2, 3, 1]],))
    with pytest.raises(ConfigurationError, match="TIMESTAMP_INVALID"):
        TimedTransform(-0.1, np.eye(4))
    with pytest.raises(ConfigurationError, match="POINT_FORMAT_INVALID"):
        PointCloudInput((0,), ([[1, 2, 3, 1]],), format="pointcloud2")
    camera = _camera()
    with pytest.raises(ConfigurationError, match="IMAGE_FORMAT_INVALID"):
        CameraInput("front", (0,), (b"data",), "bmp", camera.intrinsics, (3, 2))


def test_empty_point_frame_is_rejected_when_payload_is_validated():
    lidar = PointCloudInput((0,), (np.empty((0, 4), dtype=np.float32),))
    with pytest.raises(SourceDataError) as error:
        lidar.validated_frame(0)
    assert error.value.code == "POINT_FRAME_EMPTY"
    assert error.value.path == "lidar.frames[0]"


def test_valid_strict_mapping_builds_the_same_typed_configuration():
    config = SourceAdapterConfig.from_mapping(_valid_mapping())
    assert isinstance(config.lidar, PointCloudInput)
    assert config.cameras[0].name == "front"
    assert config.synchronization == SynchronizationConfig()


@pytest.mark.parametrize(
    ("scope", "correct", "misspelled", "expected_path"),
    [
        ("config", "synchronization", "syncronization", "config.syncronization"),
        ("lidar", "format", "formt", "lidar.formt"),
        ("camera", "image_format", "image_formt", "cameras[0].image_formt"),
        ("transform", "matrix", "matrx", "static_transforms[0].matrx"),
        ("frame", "camera_name", "camera_nam", "static_transforms[1].target.camera_nam"),
        ("imu", "samples", "sample", "imu.sample"),
        ("sample", "timestamp", "timstamp", "imu.samples[0].timstamp"),
        ("sync", "max_delta_ms", "max_delta_m", "synchronization.max_delta_m"),
    ],
)
def test_every_nested_mapping_level_rejects_misspelled_fields(scope, correct, misspelled, expected_path):
    data = deepcopy(_valid_mapping())
    data["lidar"]["format"] = "xyzi-f32"
    data["synchronization"] = {"method": "nearest", "max_delta_ms": 50}
    containers = {
        "config": data,
        "lidar": data["lidar"],
        "camera": data["cameras"][0],
        "transform": data["static_transforms"][0],
        "frame": data["static_transforms"][1]["target"],
        "imu": data["imu"],
        "sample": data["imu"]["samples"][0],
        "sync": data["synchronization"],
    }
    container = containers[scope]
    container[misspelled] = container.pop(correct)
    with pytest.raises(ConfigurationError) as error:
        SourceAdapterConfig.from_mapping(data)
    assert error.value.code == "CONFIG_UNKNOWN_FIELD"
    assert error.value.path == expected_path
    assert correct in str(error.value)


def test_config_mapping_rejects_missing_and_misspelled_fields_with_paths():
    with pytest.raises(ConfigurationError) as missing:
        SourceAdapterConfig.from_mapping({"lidar": {}, "cameras": [], "static_transforms": []})
    assert missing.value.code == "CONFIG_REQUIRED_FIELD_MISSING"
    assert missing.value.path == "config.imu"

    with pytest.raises(ConfigurationError) as typo:
        SourceAdapterConfig.from_mapping({
            "lidar": {"timestamps": [0], "frames": [[[0, 0, 0, 0]]]},
            "cameras": [{"name": "front", "timestamps": [0], "images": [np.zeros((2, 3, 3), dtype=np.uint8)], "image_formt": "numpy-rgb", "intrinsics": np.eye(3), "image_size": [3, 2]}],
            "static_transforms": [],
            "imu": {"samples": []},
        })
    assert typo.value.code == "CONFIG_UNKNOWN_FIELD"
    assert typo.value.path == "cameras[0].image_formt"
    assert "image_format" in str(typo.value)


@pytest.mark.parametrize(
    ("field", "value", "path"),
    [
        ("lidar", {}, "lidar"),
        ("cameras", ["front"], "cameras[0]"),
        ("static_transforms", [np.eye(4)], "static_transforms[0]"),
        ("imu", {}, "imu"),
        ("synchronization", {}, "synchronization"),
    ],
)
def test_direct_configuration_rejects_wrong_object_types(field, value, path):
    values = {
        "lidar": PointCloudInput((0.5,), ([[1, 2, 3, 1]],)),
        "cameras": (_camera(),),
        "static_transforms": (
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),
            TransformSpec(FrameRef.camera("front"), FrameRef.lidar(), np.eye(4)),
        ),
        "imu": _imu(),
        "synchronization": SynchronizationConfig(),
    }
    values[field] = value
    with pytest.raises(ConfigurationError) as error:
        SourceAdapterConfig(**values)
    assert error.value.path == path


def test_duplicate_camera_and_timestamp_order_fail_before_writing():
    with pytest.raises(ConfigurationError, match="CAMERA_NAME_DUPLICATE"):
        _config((_camera("front"), _camera("front")))
    with pytest.raises(ConfigurationError, match="TIMESTAMP_ORDER_INVALID"):
        PointCloudInput((1, 1), ([[0, 0, 0, 0]], [[0, 0, 0, 0]]))


def test_rigid_matrix_validation_is_direction_explicit_and_strict():
    invalid = np.eye(4); invalid[3, 0] = 1
    with pytest.raises(ConfigurationError, match="RIGID_HOMOGENEOUS_ROW_INVALID"):
        TransformSpec(FrameRef.lidar(), FrameRef.imu(), invalid)
    reflection = np.eye(4); reflection[0, 0] = -1
    with pytest.raises(ConfigurationError, match="RIGID_ROTATION_INVALID"):
        TransformSpec(FrameRef.lidar(), FrameRef.imu(), reflection)


def test_transform_tree_rejects_missing_unknown_duplicate_and_cycle():
    with pytest.raises(TransformGraphError, match="TRANSFORM_PATH_MISSING"):
        _config(transforms=(TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),))

    with pytest.raises(TransformGraphError, match="TRANSFORM_FRAME_UNKNOWN"):
        _config(transforms=(
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),
            TransformSpec(FrameRef.camera("other"), FrameRef.lidar(), np.eye(4)),
        ))

    with pytest.raises(TransformGraphError, match="TRANSFORM_EDGE_DUPLICATE"):
        _config(transforms=(
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),
            TransformSpec(FrameRef.imu(), FrameRef.lidar(), np.eye(4)),
            TransformSpec(FrameRef.camera("front"), FrameRef.lidar(), np.eye(4)),
        ))

    with pytest.raises(TransformGraphError, match="TRANSFORM_GRAPH_CYCLE"):
        _config(transforms=(
            TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),
            TransformSpec(FrameRef.camera("front"), FrameRef.lidar(), np.eye(4)),
            TransformSpec(FrameRef.camera("front"), FrameRef.imu(), np.eye(4)),
        ))


def test_camera_adjacent_calibration_edge_is_resolved_without_editable_flag():
    through_lidar = _config()
    index, edge = calibration_edge_for_camera(through_lidar.static_transforms, "front")
    assert index == 1
    assert {edge.source, edge.target} == {FrameRef.camera("front"), FrameRef.lidar()}

    direct = _config(transforms=(
        TransformSpec(FrameRef.lidar(), FrameRef.imu(), np.eye(4)),
        TransformSpec(FrameRef.camera("front"), FrameRef.imu(), np.eye(4)),
    ))
    _, edge = calibration_edge_for_camera(direct.static_transforms, "front")
    assert {edge.source, edge.target} == {FrameRef.camera("front"), FrameRef.imu()}
