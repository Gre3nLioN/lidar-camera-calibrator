"""Adapter for Foxglove JSON/base64 MCAP recordings."""
from __future__ import annotations

import base64
from collections import deque
import json
from pathlib import Path
from typing import Any, Sequence

from mcap.reader import make_reader
import numpy as np

from ..profile import (
    CameraInput, ConfigurationError, EgoPoseStreamSpec, FrameRef, PointCloudInput,
    SourceAdapterConfig, SourceDataError, SynchronizationConfig, TimedTransform,
    TransformSpec,
)


class _FoxglovePoints(Sequence[np.ndarray]):
    def __init__(self, messages: Sequence[dict[str, Any]]) -> None:
        self._messages = tuple(messages)

    def __len__(self) -> int:
        return len(self._messages)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return type(self)(self._messages[index])
        message = self._messages[index]
        try:
            raw = base64.b64decode(message["data"], validate=True)
            stride = int(message["point_stride"])
            fields = {field["name"]: field for field in message["fields"]}
            count, remainder = divmod(len(raw), stride)
            if remainder or stride <= 0:
                raise ValueError("payload length is not divisible by point_stride")
            points = np.empty((count, 4), dtype=np.float32)
            for output_column, name in enumerate(("x", "y", "z", "intensity")):
                if name not in fields:
                    if name == "intensity":
                        points[:, output_column] = 0
                        continue
                    raise ValueError(f"missing point field {name!r}")
                field = fields[name]
                if int(field["type"]) != 7:
                    raise ValueError(f"point field {name!r} is not FLOAT32 (type 7)")
                offset = int(field["offset"])
                points[:, output_column] = np.ndarray(
                    (count,), dtype="<f4", buffer=raw, offset=offset, strides=(stride,)
                )
            return points
        except (KeyError, TypeError, ValueError, MemoryError) as exc:
            raise SourceDataError(
                "FOXGLOVE_POINT_INVALID", f"lidar.frames[{index}]",
                "Foxglove point cloud payload is malformed", hint=str(exc),
            ) from exc


class _FoxgloveImages(Sequence[bytes]):
    def __init__(self, messages: Sequence[dict[str, Any]]) -> None:
        self._messages = tuple(messages)

    def __len__(self) -> int:
        return len(self._messages)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return type(self)(self._messages[index])
        try:
            return base64.b64decode(self._messages[index]["data"], validate=True)
        except (KeyError, TypeError, ValueError) as exc:
            raise SourceDataError(
                "FOXGLOVE_IMAGE_INVALID", f"cameras.images[{index}]",
                "Foxglove compressed image payload is malformed", hint=str(exc),
            ) from exc


def foxglove_source_config(
    path: str | Path,
    *,
    frame_limit: int | None = None,
    max_delta_ms: float = 50.0,
    world_frame: str = "world",
    imu_frame: str = "imu",
    tf_topic: str = "/tf",
    lidar_topic: str = "/lidar/points",
) -> SourceAdapterConfig:
    """Normalize standard Foxglove JSON messages into canonical source config.

    Supported schemas are ``foxglove.FrameTransform``, ``CameraCalibration``,
    ``CompressedImage``, and ``PointCloud``. Point fields must be FLOAT32.
    """
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ConfigurationError(
            "FOXGLOVE_PATH_INVALID", "path", "Foxglove MCAP path must be a file",
            actual=str(source),
        )
    if frame_limit is not None and (
        isinstance(frame_limit, bool) or int(frame_limit) != frame_limit or frame_limit <= 0
    ):
        raise ConfigurationError(
            "FOXGLOVE_FRAME_LIMIT_INVALID", "frame_limit",
            "frame_limit must be None or a positive integer", actual=frame_limit,
        )

    messages: dict[str, list[dict[str, Any]]] = {}
    schemas: dict[str, str] = {}
    try:
        with source.open("rb") as stream:
            reader = make_reader(stream)
            for schema, channel, message in reader.iter_messages():
                if channel.message_encoding != "json":
                    continue
                try:
                    decoded = json.loads(message.data)
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise SourceDataError(
                        "FOXGLOVE_JSON_INVALID", channel.topic,
                        "Foxglove JSON message could not be decoded", hint=str(exc),
                    ) from exc
                messages.setdefault(channel.topic, []).append(decoded)
                schemas[channel.topic] = schema.name
    except SourceDataError:
        raise
    except Exception as exc:
        raise SourceDataError(
            "FOXGLOVE_MCAP_INVALID", "path", "Foxglove MCAP could not be read",
            actual=str(source), hint=str(exc),
        ) from exc

    _require_schema(schemas, lidar_topic, "foxglove.PointCloud")
    _require_schema(schemas, tf_topic, "foxglove.FrameTransform")
    lidar_messages = messages.get(lidar_topic, ())
    if frame_limit is not None:
        lidar_messages = lidar_messages[:int(frame_limit)]
    if not lidar_messages:
        raise SourceDataError(
            "FOXGLOVE_LIDAR_EMPTY", lidar_topic, "Foxglove MCAP has no LiDAR messages"
        )
    lidar_timestamps = tuple(_timestamp(message, lidar_topic) for message in lidar_messages)
    lidar_frame = _consistent_frame(lidar_messages, lidar_topic)

    tf_messages = messages.get(tf_topic, ())
    static_edges: dict[tuple[str, str], np.ndarray] = {}
    world_imu_samples = []
    for index, message in enumerate(tf_messages):
        try:
            parent = str(message["parent_frame_id"])
            child = str(message["child_frame_id"])
            matrix = _transform_matrix(message)
        except (KeyError, TypeError, ValueError) as exc:
            raise SourceDataError(
                "FOXGLOVE_TF_INVALID", f"{tf_topic}[{index}]",
                "Foxglove frame transform is malformed", hint=str(exc),
            ) from exc
        if (parent, child) == (world_frame, imu_frame):
            world_imu_samples.append((_timestamp(message, tf_topic), matrix))
        else:
            previous = static_edges.get((parent, child))
            if previous is not None and not np.allclose(previous, matrix, atol=1e-8):
                raise SourceDataError(
                    "FOXGLOVE_DYNAMIC_SENSOR_TF", f"{tf_topic}[{index}]",
                    "sensor transforms must be static for calibration", actual=[parent, child],
                )
            static_edges[(parent, child)] = matrix

    graph = _transform_graph(static_edges)
    lidar_from_imu = _resolve_transform(graph, lidar_frame, imu_frame)
    calibration_topics = sorted(
        topic for topic, schema in schemas.items()
        if schema == "foxglove.CameraCalibration"
    )
    if not calibration_topics:
        raise SourceDataError(
            "FOXGLOVE_CAMERAS_EMPTY", "cameras",
            "Foxglove MCAP has no CameraCalibration channels",
        )

    cameras = []
    camera_transforms = []
    seen_names = set()
    for calibration_topic in calibration_topics:
        calibration_messages = messages[calibration_topic]
        if len(calibration_messages) != 1:
            raise SourceDataError(
                "FOXGLOVE_CALIBRATION_COUNT_INVALID", calibration_topic,
                "camera calibration channel must contain exactly one message",
                expected=1, actual=len(calibration_messages),
            )
        calibration = calibration_messages[0]
        raw_frame = str(calibration.get("frame_id", ""))
        camera_name = raw_frame
        if not camera_name or camera_name in seen_names:
            raise SourceDataError(
                "FOXGLOVE_CAMERA_NAME_INVALID", calibration_topic,
                "camera frame_id must be non-empty and unique", actual=camera_name,
            )
        seen_names.add(camera_name)
        image_topic = calibration_topic.removesuffix("/calibration") + "/image"
        _require_schema(schemas, image_topic, "foxglove.CompressedImage")
        image_messages = messages.get(image_topic, ())
        if not image_messages:
            raise SourceDataError(
                "FOXGLOVE_IMAGES_EMPTY", image_topic, "camera image channel is empty"
            )
        image_frame = _consistent_frame(image_messages, image_topic)
        if image_frame != raw_frame:
            raise SourceDataError(
                "FOXGLOVE_CAMERA_FRAME_MISMATCH", image_topic,
                "image and calibration frame_id values differ",
                expected=raw_frame, actual=image_frame,
            )
        try:
            projection = np.asarray(calibration["P"], dtype=float).reshape(3, 4)
            rectification = np.asarray(calibration["R"], dtype=float).reshape(3, 3)
            intrinsics = projection[:, :3]
            rectified_from_raw = np.eye(4)
            rectified_from_raw[:3, :3] = rectification
            rectified_from_raw[:3, 3] = np.linalg.solve(intrinsics, projection[:, 3])
            camera_from_lidar = (
                rectified_from_raw @ _resolve_transform(graph, raw_frame, lidar_frame)
            )
            width, height = int(calibration["width"]), int(calibration["height"])
            formats = {str(message["format"]).lower().replace("jpg", "jpeg") for message in image_messages}
            if len(formats) != 1 or next(iter(formats)) not in {"png", "jpeg"}:
                raise ValueError(f"unsupported or inconsistent image formats: {sorted(formats)!r}")
            image_format = next(iter(formats))
        except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as exc:
            raise SourceDataError(
                "FOXGLOVE_CALIBRATION_INVALID", calibration_topic,
                "Foxglove camera calibration is malformed", hint=str(exc),
            ) from exc
        cameras.append(CameraInput(
            camera_name,
            tuple(_timestamp(message, image_topic) for message in image_messages),
            _FoxgloveImages(image_messages), image_format, intrinsics, (width, height),
        ))
        camera_transforms.append(TransformSpec(
            FrameRef.camera(camera_name), FrameRef.lidar(), camera_from_lidar
        ))

    if not world_imu_samples:
        raise SourceDataError(
            "FOXGLOVE_IMU_POSE_MISSING", tf_topic,
            f"no {world_frame!r}-from-{imu_frame!r} localization transform exists",
        )
    if len(world_imu_samples) == 1:
        pose = world_imu_samples[0][1]
        end = lidar_timestamps[-1] if len(lidar_timestamps) > 1 else lidar_timestamps[0] + 1e-6
        world_imu_samples = [(lidar_timestamps[0], pose), (end, pose)]
    else:
        world_imu_samples.sort(key=lambda item: item[0])

    return SourceAdapterConfig(
        PointCloudInput(lidar_timestamps, _FoxglovePoints(lidar_messages)),
        tuple(cameras),
        (TransformSpec(FrameRef.lidar(), FrameRef.imu(), lidar_from_imu), *camera_transforms),
        EgoPoseStreamSpec(tuple(TimedTransform(timestamp, matrix) for timestamp, matrix in world_imu_samples)),
        SynchronizationConfig("nearest", max_delta_ms),
    )


def _require_schema(schemas: dict[str, str], topic: str, expected: str) -> None:
    actual = schemas.get(topic)
    if actual != expected:
        raise SourceDataError(
            "FOXGLOVE_SCHEMA_MISMATCH", topic,
            "Foxglove topic is missing or has the wrong schema",
            expected=expected, actual=actual,
        )


def _timestamp(message: dict[str, Any], path: str) -> float:
    try:
        timestamp = message["timestamp"]
        return int(timestamp["sec"]) + int(timestamp["nsec"]) / 1e9
    except (KeyError, TypeError, ValueError) as exc:
        raise SourceDataError(
            "FOXGLOVE_TIMESTAMP_INVALID", path,
            "Foxglove timestamp requires integer sec and nsec", hint=str(exc),
        ) from exc


def _consistent_frame(messages: Sequence[dict[str, Any]], path: str) -> str:
    frames = {str(message.get("frame_id", "")) for message in messages}
    if len(frames) != 1 or not next(iter(frames)):
        raise SourceDataError(
            "FOXGLOVE_FRAME_ID_INVALID", path,
            "all messages must have one non-empty frame_id", actual=sorted(frames),
        )
    return next(iter(frames))


def _transform_matrix(message: dict[str, Any]) -> np.ndarray:
    translation = message["translation"]
    rotation = message["rotation"]
    x, y, z, w = (float(rotation[key]) for key in ("x", "y", "z", "w"))
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm <= 0:
        raise ValueError("quaternion is not finite and non-zero")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ]
    matrix[:3, 3] = [float(translation[key]) for key in ("x", "y", "z")]
    return matrix


def _transform_graph(edges: dict[tuple[str, str], np.ndarray]):
    graph: dict[str, list[tuple[str, np.ndarray]]] = {}
    for (parent, child), parent_from_child in edges.items():
        graph.setdefault(child, []).append((parent, parent_from_child))
        graph.setdefault(parent, []).append((child, np.linalg.inv(parent_from_child)))
    return graph


def _resolve_transform(graph, target: str, source: str) -> np.ndarray:
    if target == source:
        return np.eye(4)
    queue = deque([(source, np.eye(4))])
    visited = {source}
    while queue:
        node, node_from_source = queue.popleft()
        for neighbor, neighbor_from_node in graph.get(node, ()):
            if neighbor in visited:
                continue
            result = neighbor_from_node @ node_from_source
            if neighbor == target:
                return result
            visited.add(neighbor)
            queue.append((neighbor, result))
    raise SourceDataError(
        "FOXGLOVE_TRANSFORM_PATH_MISSING", "tf",
        "Foxglove TF graph has no required transform path",
        expected={"target": target, "source": source},
    )
