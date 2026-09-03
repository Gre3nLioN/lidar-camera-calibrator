"""Strict, indexed, lazy reader for canonical scene MCAP files."""
from __future__ import annotations

import base64
from collections import Counter, defaultdict
from dataclasses import dataclass
from io import BytesIO
import json
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

from jsonschema import Draft202012Validator, ValidationError
from mcap.reader import make_reader
import numpy as np
from PIL import Image, UnidentifiedImageError

from ..models import CameraCalibration, CameraFrame, FrameBundle, LoadedCalibration
from .config import FrameRef, TransformSpec
from .errors import (
    ConfigurationError, ProfileError, ProfileValidationError, ProfileVersionError,
    TransformGraphError,
)
from .schemas import PROFILE_NAME, PROFILE_VERSION, load_schema
from .transforms import calibration_edge_for_camera, validate_transform_tree


@dataclass(frozen=True)
class SceneFrameRecord:
    frame_index: int
    timestamp_ns: int
    cameras: tuple[Mapping[str, Any], ...]


_SCHEMA_BY_TOPIC = {
    "/scene/manifest": "scene-manifest",
    "/scene/static_transforms": "static-transform-tree",
    "/scene/lidar/points": "point-cloud-frame",
    "/scene/imu/pose": "ego-pose-frame",
    "/scene/frames": "scene-frame",
}


class SceneMcapReader:
    """Normalized frame source that retains only static metadata and frame indexes."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        if not self.path.is_file():
            raise ProfileValidationError(
                "PROFILE_NOT_FOUND", "scene_mcap", "canonical scene MCAP does not exist",
                actual=str(self.path),
            )
        self._manifest: Mapping[str, Any]
        self._frames: tuple[SceneFrameRecord, ...]
        self._transforms: tuple[TransformSpec, ...]
        self._camera_payloads: Mapping[str, Mapping[str, Any]]
        self._open_and_index()

    @property
    def manifest(self) -> Mapping[str, Any]:
        return self._manifest

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(camera["name"] for camera in self._manifest["cameras"])

    @property
    def static_transforms(self) -> tuple[TransformSpec, ...]:
        return self._transforms

    def __len__(self) -> int:
        return len(self._frames)

    def frame(self, index: int) -> FrameBundle:
        record = self._frame_record(index)
        topics = ["/scene/lidar/points", *(
            f"/scene/cameras/{name}/image" for name in self.camera_names
        )]
        frame_payloads = self._payloads_at(record.timestamp_ns, topics, index)
        points_payload = frame_payloads["/scene/lidar/points"]
        self._validate_frame_identity(points_payload, record, "lidar")
        points = self._decode_points(points_payload, index)

        cameras: dict[str, CameraFrame] = {}
        camera_records = {item["camera_name"]: item for item in record.cameras}
        for camera_name in self.camera_names:
            topic = f"/scene/cameras/{camera_name}/image"
            payload = frame_payloads[topic]
            self._validate_frame_identity(payload, record, f"cameras['{camera_name}']")
            if payload["camera_name"] != camera_name:
                raise ProfileValidationError(
                    "PROFILE_CAMERA_MISMATCH", f"cameras['{camera_name}'].frames[{index}]",
                    "camera image payload names the wrong camera",
                    expected=camera_name, actual=payload["camera_name"],
                )
            indexed = camera_records[camera_name]
            for key in ("source_timestamp_ns", "synchronization_delta_ns"):
                if payload[key] != indexed[key]:
                    raise ProfileValidationError(
                        "PROFILE_FRAME_INDEX_MISMATCH", f"cameras['{camera_name}'].frames[{index}].{key}",
                        "camera payload disagrees with the scene frame index",
                        expected=indexed[key], actual=payload[key],
                    )
            image = self._decode_image(camera_name, payload, index)
            cameras[camera_name] = CameraFrame(
                camera_name, payload["source_timestamp_ns"] / 1_000_000_000,
                image, index,
            )
        return FrameBundle(
            index, record.timestamp_ns / 1_000_000_000, points, cameras
        )

    def ego_pose(self, index: int) -> np.ndarray:
        record = self._frame_record(index)
        payload = self._one_payload("/scene/imu/pose", record.timestamp_ns, "imu", index)
        self._validate_frame_identity(payload, record, "imu")
        matrix = np.asarray(payload["world_from_imu"], dtype=np.float64).reshape(4, 4)
        if not np.isfinite(matrix).all():
            raise ProfileValidationError(
                "PROFILE_POSE_NONFINITE", f"imu.frames[{index}].world_from_imu",
                "ego pose contains non-finite values",
            )
        matrix.setflags(write=False)
        return matrix

    def calibration_edge(self, camera_name: str) -> tuple[int, TransformSpec]:
        return calibration_edge_for_camera(self._transforms, camera_name)

    def _frame_record(self, index: int) -> SceneFrameRecord:
        index = int(index)
        if index < 0 or index >= len(self):
            raise IndexError(index)
        return self._frames[index]

    def _open_and_index(self) -> None:
        try:
            with self.path.open("rb") as stream:
                reader = make_reader(stream, validate_crcs=True)
                header = reader.get_header()
                expected_profile = f"{PROFILE_NAME}/{PROFILE_VERSION}"
                if header.profile != expected_profile:
                    raise ProfileVersionError(
                        "PROFILE_VERSION_UNSUPPORTED", "header.profile",
                        "unsupported canonical scene profile",
                        expected=expected_profile, actual=header.profile,
                    )
                manifest_records = list(reader.iter_messages(topics="/scene/manifest"))
                if len(manifest_records) != 1:
                    raise ProfileValidationError(
                        "PROFILE_MANIFEST_COUNT_INVALID", "/scene/manifest",
                        "profile must contain exactly one manifest",
                        expected=1, actual=len(manifest_records),
                    )
                manifest = self._decode_record(
                    manifest_records[0], "scene-manifest", "/scene/manifest"
                )
                self._validate_manifest_identity(manifest)
                expected_topics = self._expected_topics(manifest)

                messages_by_topic: dict[str, list[tuple[Any, Any, Any]]] = defaultdict(list)
                dynamic_metadata: dict[str, list[tuple[int, int]]] = defaultdict(list)
                dynamic_topics = {"/scene/lidar/points", "/scene/imu/pose", *(
                    camera["image_topic"] for camera in manifest["cameras"]
                )}
                observed_channels: dict[int, Any] = {}
                for record in reader.iter_messages():
                    schema, channel, message = record
                    if channel.topic not in expected_topics:
                        raise ProfileValidationError(
                            "PROFILE_TOPIC_UNEXPECTED", channel.topic,
                            "canonical profile contains an unexpected topic",
                            expected=sorted(expected_topics), actual=channel.topic,
                        )
                    self._validate_record_contract(record, expected_topics[channel.topic], channel.topic)
                    # Dynamic JSON/base64 records can be several MiB each. Retain
                    # only identity metadata; frame() seeks and decodes payloads
                    # on demand. Keeping every record here made startup O(file size)
                    # in resident memory despite the reader's lazy public contract.
                    if channel.topic in dynamic_topics:
                        dynamic_metadata[channel.topic].append((message.log_time, message.sequence))
                    else:
                        messages_by_topic[channel.topic].append(record)
                    observed_channels[channel.id] = channel
                self._validate_channel_set(reader.get_summary(), observed_channels, expected_topics)
                self._finish_index(manifest, messages_by_topic, dynamic_metadata, expected_topics)
        except ProfileError:
            raise
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError, ValidationError) as exc:
            raise ProfileValidationError(
                "PROFILE_READ_FAILED", "scene_mcap", "canonical scene MCAP is malformed",
                actual=str(self.path), hint=str(exc),
            ) from exc
        except Exception as exc:
            raise ProfileValidationError(
                "PROFILE_CONTAINER_INVALID", "scene_mcap", "MCAP container could not be read",
                actual=str(self.path), hint=str(exc),
            ) from exc

    def _finish_index(
        self,
        manifest: Mapping[str, Any],
        messages: Mapping[str, list[tuple[Any, Any, Any]]],
        dynamic_metadata: Mapping[str, list[tuple[int, int]]],
        expected_topics: Mapping[str, str],
    ) -> None:
        static_payload = self._exact_static_payload(
            messages, "/scene/static_transforms", "static-transform-tree"
        )
        camera_payloads = {}
        for camera in manifest["cameras"]:
            name = camera["name"]
            payload = self._exact_static_payload(
                messages, camera["calibration_topic"], "camera-calibration"
            )
            if payload["camera_name"] != name:
                raise ProfileValidationError(
                    "PROFILE_CAMERA_MISMATCH", camera["calibration_topic"],
                    "camera calibration payload names the wrong camera",
                    expected=name, actual=payload["camera_name"],
                )
            camera_payloads[name] = payload
        transforms = self._parse_transforms(static_payload, tuple(camera_payloads))
        frame_records = self._parse_frame_records(messages["/scene/frames"], manifest)
        expected_times = [record.timestamp_ns for record in frame_records]

        dynamic_topics = ["/scene/lidar/points", "/scene/imu/pose"] + [
            camera["image_topic"] for camera in manifest["cameras"]
        ]
        expected_counter = Counter(expected_times)
        expected_sequence = {record.timestamp_ns: record.frame_index for record in frame_records}
        for topic in dynamic_topics:
            topic_records = dynamic_metadata.get(topic, ())
            actual_times = Counter(log_time for log_time, _ in topic_records)
            if actual_times != expected_counter:
                missing = list((expected_counter - actual_times).elements())
                duplicate = list((actual_times - expected_counter).elements())
                raise ProfileValidationError(
                    "PROFILE_FRAME_MESSAGES_INCOMPLETE", topic,
                    "topic must contain exactly one message at every scene timestamp",
                    expected={"count": len(expected_times), "timestamps_ns": expected_times},
                    actual={"count": sum(actual_times.values()), "missing": missing, "extra_or_duplicate": duplicate},
                )
            for log_time, sequence in topic_records:
                if sequence != expected_sequence[log_time]:
                    raise ProfileValidationError(
                        "PROFILE_MESSAGE_SEQUENCE_INVALID", topic,
                        "MCAP message sequence must equal the scene frame index",
                        expected=expected_sequence[log_time], actual=sequence,
                    )

        self._manifest = MappingProxyType(dict(manifest))
        self._frames = tuple(frame_records)
        self._transforms = transforms
        self._camera_payloads = MappingProxyType({
            name: MappingProxyType(dict(payload)) for name, payload in camera_payloads.items()
        })
        self.calibration = self._normalized_calibration()

    def _parse_frame_records(
        self, records: list[tuple[Any, Any, Any]], manifest: Mapping[str, Any]
    ) -> list[SceneFrameRecord]:
        expected_count = manifest["frame_count"]
        if len(records) != expected_count:
            raise ProfileValidationError(
                "PROFILE_FRAME_INDEX_COUNT_INVALID", "/scene/frames",
                "scene frame-index count disagrees with the manifest",
                expected=expected_count, actual=len(records),
            )
        camera_names = self.camera_names_from_manifest(manifest)
        parsed: list[SceneFrameRecord] = []
        for ordinal, record in enumerate(records):
            payload = self._decode_record(record, "scene-frame", f"/scene/frames[{ordinal}]")
            if payload["frame_index"] != ordinal:
                raise ProfileValidationError(
                    "PROFILE_FRAME_INDEX_INVALID", f"/scene/frames[{ordinal}].frame_index",
                    "scene frame indexes must be contiguous and ordered",
                    expected=ordinal, actual=payload["frame_index"],
                )
            if payload["timestamp_ns"] != record[2].log_time:
                raise ProfileValidationError(
                    "PROFILE_TIMESTAMP_MISMATCH", f"/scene/frames[{ordinal}].timestamp_ns",
                    "payload timestamp disagrees with MCAP log time",
                    expected=record[2].log_time, actual=payload["timestamp_ns"],
                )
            names = tuple(item["camera_name"] for item in payload["cameras"])
            if names != camera_names:
                raise ProfileValidationError(
                    "PROFILE_CAMERA_SET_MISMATCH", f"/scene/frames[{ordinal}].cameras",
                    "scene frame must list every manifest camera in order",
                    expected=camera_names, actual=names,
                )
            for camera in payload["cameras"]:
                expected_delta = camera["source_timestamp_ns"] - payload["timestamp_ns"]
                if camera["synchronization_delta_ns"] != expected_delta:
                    raise ProfileValidationError(
                        "PROFILE_SYNC_DELTA_INVALID",
                        f"/scene/frames[{ordinal}].cameras['{camera['camera_name']}']",
                        "synchronization delta must equal source timestamp minus scene timestamp",
                        expected=expected_delta, actual=camera["synchronization_delta_ns"],
                    )
            parsed.append(SceneFrameRecord(
                ordinal, payload["timestamp_ns"],
                tuple(MappingProxyType(dict(item)) for item in payload["cameras"]),
            ))
        timestamps = [record.timestamp_ns for record in parsed]
        if any(right <= left for left, right in zip(timestamps, timestamps[1:])):
            raise ProfileValidationError(
                "PROFILE_TIMESTAMP_ORDER_INVALID", "/scene/frames",
                "scene timestamps must be strictly increasing", actual=timestamps,
            )
        return parsed

    def _parse_transforms(
        self, payload: Mapping[str, Any], camera_names: tuple[str, ...]
    ) -> tuple[TransformSpec, ...]:
        transforms = []
        try:
            for item in payload["transforms"]:
                transforms.append(TransformSpec(
                    target=FrameRef.from_mapping(item["target"]),
                    source=FrameRef.from_mapping(item["source"]),
                    matrix=np.asarray(item["matrix"], dtype=float).reshape(4, 4),
                ))
            validate_transform_tree(transforms, camera_names)
        except (ConfigurationError, TransformGraphError, ValueError) as exc:
            raise ProfileValidationError(
                "PROFILE_TRANSFORM_TREE_INVALID", "/scene/static_transforms",
                "canonical static transform tree is invalid", hint=str(exc),
            ) from exc
        return tuple(transforms)

    def _normalized_calibration(self) -> LoadedCalibration:
        primary = self.camera_names[0]
        primary_from_lidar = self.transform_between(FrameRef.camera(primary), FrameRef.lidar())
        cameras = {}
        for name, payload in self._camera_payloads.items():
            intrinsics = np.asarray(payload["intrinsics"], dtype=float).reshape(3, 3)
            if (
                not np.isfinite(intrinsics).all()
                or intrinsics[0, 0] <= 0
                or intrinsics[1, 1] <= 0
                or not np.allclose(intrinsics[2], [0, 0, 1], atol=1e-9)
            ):
                raise ProfileValidationError(
                    "PROFILE_INTRINSICS_INVALID", f"cameras['{name}'].intrinsics",
                    "camera intrinsics must have positive focal lengths and row [0, 0, 1]",
                    actual=payload["intrinsics"],
                )
            projection = np.asarray(
                payload.get("projection_matrix", np.column_stack((intrinsics, np.zeros(3)))), dtype=float
            ).reshape(3, 4)
            rectification = np.asarray(payload.get("rectification", np.eye(3)), dtype=float).reshape(3, 3)
            cameras[name] = CameraCalibration(
                camera_id=name,
                image_size=tuple(payload["image_size"]),
                intrinsics=intrinsics,
                projection_matrix=projection,
                camera_from_camera00=self.transform_between(
                    FrameRef.camera(name), FrameRef.camera(primary)
                ),
                rectification=rectification,
            )
        return LoadedCalibration(
            primary_from_lidar,
            cameras,
            dataset=PROFILE_NAME,
            sequence=self.path.stem,
            lidar_to_cameras={
                name: self.transform_between(FrameRef.camera(name), FrameRef.lidar())
                for name in self.camera_names
            },
        )

    def transform_between(self, target: FrameRef, source: FrameRef) -> np.ndarray:
        if target == source:
            return np.eye(4)
        adjacency: dict[FrameRef, list[tuple[FrameRef, np.ndarray]]] = defaultdict(list)
        for edge in self._transforms:
            adjacency[edge.source].append((edge.target, edge.matrix))
            adjacency[edge.target].append((edge.source, np.linalg.inv(edge.matrix)))
        queue = [(source, np.eye(4))]
        visited = {source}
        while queue:
            current, current_from_source = queue.pop(0)
            for neighbor, neighbor_from_current in adjacency[current]:
                if neighbor in visited:
                    continue
                neighbor_from_source = neighbor_from_current @ current_from_source
                if neighbor == target:
                    return neighbor_from_source
                visited.add(neighbor)
                queue.append((neighbor, neighbor_from_source))
        raise ProfileValidationError(
            "PROFILE_TRANSFORM_PATH_MISSING", "static_transforms",
            "requested semantic transform path is missing",
            expected={"target": repr(target), "source": repr(source)},
        )

    def _payloads_at(
        self, timestamp_ns: int, topics: list[str], index: int
    ) -> dict[str, Mapping[str, Any]]:
        """Read all requested payloads through one indexed MCAP seek/decompression pass."""
        try:
            with self.path.open("rb") as stream:
                records = list(make_reader(stream, validate_crcs=True).iter_messages(
                    topics=topics, start_time=timestamp_ns, end_time=timestamp_ns + 1
                ))
        except Exception as exc:
            raise ProfileValidationError(
                "PROFILE_FRAME_READ_FAILED", f"frames[{index}]",
                "frame messages could not be read", hint=str(exc),
            ) from exc
        records_by_topic: dict[str, list[tuple[Any, Any, Any]]] = defaultdict(list)
        for record in records:
            records_by_topic[record[1].topic].append(record)
        payloads = {}
        expected_topics = self._expected_topics(self._manifest)
        for topic in topics:
            topic_records = records_by_topic.get(topic, ())
            if len(topic_records) != 1:
                raise ProfileValidationError(
                    "PROFILE_FRAME_MESSAGE_COUNT_INVALID", f"{topic}.frames[{index}]",
                    "expected exactly one indexed frame message",
                    expected=1, actual=len(topic_records),
                )
            payloads[topic] = self._decode_record(topic_records[0], expected_topics[topic], topic)
        return payloads

    def _one_payload(self, topic: str, timestamp_ns: int, path: str, index: int) -> Mapping[str, Any]:
        return self._payloads_at(timestamp_ns, [topic], index)[topic]

    def _decode_points(self, payload: Mapping[str, Any], index: int) -> np.ndarray:
        try:
            raw = base64.b64decode(payload["data"], validate=True)
        except (ValueError, TypeError) as exc:
            raise ProfileValidationError(
                "PROFILE_BASE64_INVALID", f"lidar.frames[{index}].data",
                "point data is not valid base64", hint=str(exc),
            ) from exc
        shape = tuple(payload["shape"])
        expected_bytes = shape[0] * shape[1] * 4
        if len(raw) != expected_bytes:
            raise ProfileValidationError(
                "PROFILE_POINT_LENGTH_INVALID", f"lidar.frames[{index}].data",
                "decoded point byte length disagrees with shape and dtype",
                expected=expected_bytes, actual=len(raw),
            )
        points = np.frombuffer(raw, dtype="<f4").reshape(shape).copy()
        if not np.isfinite(points).all():
            raise ProfileValidationError(
                "PROFILE_POINT_NONFINITE", f"lidar.frames[{index}]",
                "decoded point frame contains non-finite values",
            )
        points.setflags(write=False)
        return points

    def _decode_image(self, camera_name: str, payload: Mapping[str, Any], index: int) -> np.ndarray:
        path = f"cameras['{camera_name}'].frames[{index}].data"
        try:
            raw = base64.b64decode(payload["data"], validate=True)
            with Image.open(BytesIO(raw)) as image:
                if image.format.lower() != payload["format"]:
                    raise ProfileValidationError(
                        "PROFILE_IMAGE_ENCODING_MISMATCH", path,
                        "decoded image encoding disagrees with payload format",
                        expected=payload["format"], actual=image.format.lower(),
                    )
                decoded = np.asarray(image.convert("RGB")).copy()
        except ProfileError:
            raise
        except (ValueError, TypeError, OSError, UnidentifiedImageError) as exc:
            raise ProfileValidationError(
                "PROFILE_IMAGE_DECODE_FAILED", path,
                "camera image is not valid encoded image data", hint=str(exc),
            ) from exc
        expected_size = tuple(self._camera_payloads[camera_name]["image_size"])
        if decoded.shape != (expected_size[1], expected_size[0], 3):
            raise ProfileValidationError(
                "PROFILE_IMAGE_SIZE_MISMATCH", path,
                "decoded image dimensions disagree with camera calibration",
                expected=(expected_size[1], expected_size[0], 3), actual=decoded.shape,
            )
        decoded.setflags(write=False)
        return decoded

    @staticmethod
    def _validate_frame_identity(
        payload: Mapping[str, Any], record: SceneFrameRecord, path: str
    ) -> None:
        if payload["frame_index"] != record.frame_index:
            raise ProfileValidationError(
                "PROFILE_FRAME_INDEX_MISMATCH", f"{path}.frames[{record.frame_index}].frame_index",
                "payload frame index disagrees with scene frame index",
                expected=record.frame_index, actual=payload["frame_index"],
            )
        if payload["timestamp_ns"] != record.timestamp_ns:
            raise ProfileValidationError(
                "PROFILE_TIMESTAMP_MISMATCH", f"{path}.frames[{record.frame_index}].timestamp_ns",
                "payload timestamp disagrees with scene frame index",
                expected=record.timestamp_ns, actual=payload["timestamp_ns"],
            )

    @staticmethod
    def camera_names_from_manifest(manifest: Mapping[str, Any]) -> tuple[str, ...]:
        return tuple(camera["name"] for camera in manifest["cameras"])

    @staticmethod
    def _validate_manifest_identity(manifest: Mapping[str, Any]) -> None:
        if manifest["profile_name"] != PROFILE_NAME or manifest["profile_version"] != PROFILE_VERSION:
            raise ProfileVersionError(
                "PROFILE_VERSION_UNSUPPORTED", "/scene/manifest",
                "manifest declares an unsupported profile identity",
                expected={"profile_name": PROFILE_NAME, "profile_version": PROFILE_VERSION},
                actual={"profile_name": manifest["profile_name"], "profile_version": manifest["profile_version"]},
            )
        names = [camera["name"] for camera in manifest["cameras"]]
        if len(names) != len(set(names)):
            raise ProfileValidationError(
                "PROFILE_CAMERA_DUPLICATE", "/scene/manifest.cameras",
                "manifest camera names must be unique", actual=names,
            )

    @staticmethod
    def _expected_topics(manifest: Mapping[str, Any]) -> dict[str, str]:
        topics = dict(_SCHEMA_BY_TOPIC)
        declared = manifest["topics"]
        fixed = {
            "static_transforms": "/scene/static_transforms",
            "lidar_points": "/scene/lidar/points",
            "imu_pose": "/scene/imu/pose",
            "frames": "/scene/frames",
        }
        if declared != fixed:
            raise ProfileValidationError(
                "PROFILE_TOPIC_MAP_INVALID", "/scene/manifest.topics",
                "manifest canonical topic map is invalid", expected=fixed, actual=declared,
            )
        for camera in manifest["cameras"]:
            name = camera["name"]
            expected_image = f"/scene/cameras/{name}/image"
            expected_calibration = f"/scene/cameras/{name}/calibration"
            if camera["image_topic"] != expected_image or camera["calibration_topic"] != expected_calibration:
                raise ProfileValidationError(
                    "PROFILE_CAMERA_TOPIC_INVALID", f"/scene/manifest.cameras['{name}']",
                    "camera topics do not follow the canonical naming convention",
                    expected={"image_topic": expected_image, "calibration_topic": expected_calibration},
                    actual=camera,
                )
            topics[expected_image] = "camera-image-frame"
            topics[expected_calibration] = "camera-calibration"
        return topics

    def _exact_static_payload(
        self, messages: Mapping[str, list[tuple[Any, Any, Any]]], topic: str, schema_name: str
    ) -> Mapping[str, Any]:
        records = messages.get(topic, ())
        if len(records) != 1:
            raise ProfileValidationError(
                "PROFILE_STATIC_MESSAGE_COUNT_INVALID", topic,
                "static canonical topic must contain exactly one message",
                expected=1, actual=len(records),
            )
        return self._decode_record(records[0], schema_name, topic)

    @staticmethod
    def _validate_channel_set(summary: Any, observed: Mapping[int, Any], expected: Mapping[str, str]) -> None:
        channels = summary.channels if summary is not None else observed
        topics = [channel.topic for channel in channels.values()]
        if Counter(topics) != Counter(expected.keys()):
            raise ProfileValidationError(
                "PROFILE_CHANNEL_SET_INVALID", "channels",
                "canonical profile must contain exactly one channel for every required topic",
                expected=sorted(expected), actual=sorted(topics),
            )

    def _decode_record(
        self, record: tuple[Any, Any, Any], schema_name: str, path: str
    ) -> Mapping[str, Any]:
        self._validate_record_contract(record, schema_name, path)
        try:
            payload = json.loads(record[2].data)
            Draft202012Validator(load_schema(schema_name)).validate(payload)
        except (UnicodeDecodeError, json.JSONDecodeError, ValidationError) as exc:
            detail = getattr(exc, "message", str(exc))
            raise ProfileValidationError(
                "PROFILE_MESSAGE_SCHEMA_INVALID", path,
                "message is not valid canonical JSON for its schema", hint=detail,
            ) from exc
        return payload

    @staticmethod
    def _validate_record_contract(
        record: tuple[Any, Any, Any], schema_name: str, path: str
    ) -> None:
        schema, channel, _ = record
        expected_schema = load_schema(schema_name)
        if channel.message_encoding != "json":
            raise ProfileValidationError(
                "PROFILE_MESSAGE_ENCODING_INVALID", path,
                "canonical channel must use JSON message encoding",
                expected="json", actual=channel.message_encoding,
            )
        expected_metadata = {"profile": PROFILE_NAME, "profile_version": str(PROFILE_VERSION)}
        if channel.metadata != expected_metadata:
            raise ProfileValidationError(
                "PROFILE_CHANNEL_METADATA_INVALID", path,
                "canonical channel metadata is invalid",
                expected=expected_metadata, actual=channel.metadata,
            )
        message = record[2]
        if message.publish_time != message.log_time:
            raise ProfileValidationError(
                "PROFILE_MESSAGE_TIME_INVALID", path,
                "canonical publish time must equal log time",
                expected=message.log_time, actual=message.publish_time,
            )
        if schema is None or schema.name != expected_schema["title"] or schema.encoding != "jsonschema":
            raise ProfileValidationError(
                "PROFILE_SCHEMA_IDENTITY_INVALID", path,
                "channel references the wrong canonical schema",
                expected={"name": expected_schema["title"], "encoding": "jsonschema"},
                actual=None if schema is None else {"name": schema.name, "encoding": schema.encoding},
            )
        try:
            embedded = json.loads(schema.data)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProfileValidationError(
                "PROFILE_SCHEMA_RESOURCE_INVALID", path,
                "embedded channel schema is not valid JSON", hint=str(exc),
            ) from exc
        if embedded != expected_schema:
            raise ProfileValidationError(
                "PROFILE_SCHEMA_RESOURCE_MISMATCH", path,
                "embedded schema differs from the canonical packaged schema",
                expected=expected_schema["$id"], actual=embedded.get("$id"),
            )
