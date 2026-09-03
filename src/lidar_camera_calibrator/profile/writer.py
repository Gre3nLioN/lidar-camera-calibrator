"""Atomic writer for the canonical, synchronized scene MCAP profile."""
from __future__ import annotations

import base64
from importlib.metadata import PackageNotFoundError, version
from io import BytesIO
import json
import os
from pathlib import Path
import tempfile
from typing import Any

import numpy as np
from mcap.writer import Writer

from .config import CameraInput, FrameRef, SourceAdapterConfig
from .errors import ProfileError, ProfileWriteError
from .schemas import PROFILE_NAME, PROFILE_VERSION, SCHEMA_NAMES, load_schema
from .synchronization import interpolate_imu, synchronize_camera

_MANIFEST_TOPIC = "/scene/manifest"
_TRANSFORMS_TOPIC = "/scene/static_transforms"
_LIDAR_TOPIC = "/scene/lidar/points"
_IMU_TOPIC = "/scene/imu/pose"
_FRAMES_TOPIC = "/scene/frames"


def write_profile_mcap(config: SourceAdapterConfig, output_path: str | os.PathLike[str]) -> Path:
    """Validate, synchronize, and atomically write one canonical scene MCAP."""
    if not isinstance(config, SourceAdapterConfig):
        raise ProfileWriteError(
            "PROFILE_CONFIG_INVALID", "config", "write_profile_mcap requires SourceAdapterConfig",
            expected="SourceAdapterConfig", actual=type(config).__name__,
        )
    destination = Path(output_path).expanduser()
    if destination.suffix.lower() != ".mcap":
        raise ProfileWriteError(
            "PROFILE_PATH_INVALID", "output_path", "canonical scene output must use the .mcap suffix",
            actual=str(destination),
        )
    try:
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary_handle = tempfile.NamedTemporaryFile(
            mode="wb", prefix=f".{destination.name}.", suffix=".tmp",
            dir=destination.parent, delete=False,
        )
        temporary_path = Path(temporary_handle.name)
        temporary_handle.close()
    except OSError as exc:
        raise ProfileWriteError(
            "PROFILE_OUTPUT_UNAVAILABLE", "output_path", "could not create temporary profile output",
            actual=str(destination), hint=str(exc),
        ) from exc

    try:
        _write_profile(config, temporary_path)
        os.replace(temporary_path, destination)
    except ProfileError:
        temporary_path.unlink(missing_ok=True)
        raise
    except Exception as exc:
        temporary_path.unlink(missing_ok=True)
        raise ProfileWriteError(
            "PROFILE_WRITE_FAILED", "output_path", "canonical scene MCAP writing failed",
            actual=str(destination), hint=str(exc),
        ) from exc
    return destination


def _write_profile(config: SourceAdapterConfig, path: Path) -> None:
    lidar_times = config.lidar.timestamps
    camera_matches = {
        camera.name: synchronize_camera(
            lidar_times, camera.timestamps, config.synchronization, camera.name
        )
        for camera in config.cameras
    }
    imu_poses = interpolate_imu(lidar_times, config.imu)
    first_time_ns = _timestamp_ns(lidar_times[0])

    with path.open("wb") as stream:
        writer = Writer(stream)
        writer.start(profile=f"{PROFILE_NAME}/{PROFILE_VERSION}", library=_library_identity())
        schema_ids = _register_schemas(writer)
        channels = _register_channels(writer, schema_ids, config)

        _publish(writer, channels["manifest"], first_time_ns, _manifest(config))
        _publish(writer, channels["static_transforms"], first_time_ns, _static_transforms(config))
        for camera in config.cameras:
            _publish(
                writer, channels[f"calibration:{camera.name}"], first_time_ns,
                _camera_calibration(camera),
            )

        for frame_index, timestamp in enumerate(lidar_times):
            timestamp_ns = _timestamp_ns(timestamp)
            points = config.lidar.validated_frame(frame_index)
            _publish(
                writer, channels["lidar"], timestamp_ns,
                _point_cloud_frame(frame_index, timestamp_ns, points),
                sequence=frame_index,
            )
            _publish(
                writer, channels["imu"], timestamp_ns,
                _ego_pose_frame(frame_index, timestamp_ns, imu_poses[frame_index].matrix),
                sequence=frame_index,
            )
            frame_cameras = []
            for camera in config.cameras:
                match = camera_matches[camera.name][frame_index]
                source_image = camera.validated_image(match.source_index)
                image_format, image_payload = _canonical_image(camera, source_image)
                source_timestamp_ns = _timestamp_ns(match.source_timestamp)
                delta_ns = source_timestamp_ns - timestamp_ns
                _publish(
                    writer, channels[f"image:{camera.name}"], timestamp_ns,
                    _camera_image_frame(
                        camera.name, frame_index, timestamp_ns, source_timestamp_ns,
                        delta_ns, image_format, image_payload,
                    ),
                    sequence=frame_index,
                )
                frame_cameras.append({
                    "camera_name": camera.name,
                    "source_timestamp_ns": source_timestamp_ns,
                    "synchronization_delta_ns": delta_ns,
                })
            _publish(
                writer, channels["frames"], timestamp_ns,
                {"frame_index": frame_index, "timestamp_ns": timestamp_ns, "cameras": frame_cameras},
                sequence=frame_index,
            )
        writer.finish()


def _register_schemas(writer: Writer) -> dict[str, int]:
    registered = {}
    for name in SCHEMA_NAMES:
        schema = load_schema(name)
        registered[name] = writer.register_schema(
            name=schema["title"], encoding="jsonschema", data=_json_bytes(schema)
        )
    return registered


def _register_channels(
    writer: Writer, schema_ids: dict[str, int], config: SourceAdapterConfig
) -> dict[str, int]:
    metadata = {"profile": PROFILE_NAME, "profile_version": str(PROFILE_VERSION)}

    def channel(topic: str, schema: str) -> int:
        return writer.register_channel(
            topic=topic, message_encoding="json", schema_id=schema_ids[schema],
            metadata=metadata,
        )

    channels = {
        "manifest": channel(_MANIFEST_TOPIC, "scene-manifest"),
        "static_transforms": channel(_TRANSFORMS_TOPIC, "static-transform-tree"),
        "lidar": channel(_LIDAR_TOPIC, "point-cloud-frame"),
        "imu": channel(_IMU_TOPIC, "ego-pose-frame"),
        "frames": channel(_FRAMES_TOPIC, "scene-frame"),
    }
    for camera in config.cameras:
        channels[f"calibration:{camera.name}"] = channel(
            f"/scene/cameras/{camera.name}/calibration", "camera-calibration"
        )
        channels[f"image:{camera.name}"] = channel(
            f"/scene/cameras/{camera.name}/image", "camera-image-frame"
        )
    return channels


def _manifest(config: SourceAdapterConfig) -> dict[str, Any]:
    return {
        "profile_name": PROFILE_NAME,
        "profile_version": PROFILE_VERSION,
        "frame_count": len(config.lidar.timestamps),
        "cameras": [
            {
                "name": camera.name,
                "image_topic": f"/scene/cameras/{camera.name}/image",
                "calibration_topic": f"/scene/cameras/{camera.name}/calibration",
            }
            for camera in config.cameras
        ],
        "synchronization": {
            "timeline": "lidar",
            "camera_method": config.synchronization.method,
            "max_delta_ms": config.synchronization.max_delta_ms,
            "imu_method": "interpolate",
        },
        "topics": {
            "static_transforms": _TRANSFORMS_TOPIC,
            "lidar_points": _LIDAR_TOPIC,
            "imu_pose": _IMU_TOPIC,
            "frames": _FRAMES_TOPIC,
        },
        "created_by": {"library": "lidar-camera-calibrator", "version": _package_version()},
    }


def _static_transforms(config: SourceAdapterConfig) -> dict[str, Any]:
    return {
        "transforms": [
            {
                "target": _frame_ref(transform.target),
                "source": _frame_ref(transform.source),
                "matrix": transform.matrix.reshape(-1).tolist(),
            }
            for transform in config.static_transforms
        ]
    }


def _frame_ref(frame: FrameRef) -> dict[str, str]:
    result = {"role": frame.role.value}
    if frame.camera_name is not None:
        result["camera_name"] = frame.camera_name
    return result


def _camera_calibration(camera: CameraInput) -> dict[str, Any]:
    return {
        "camera_name": camera.name,
        "image_size": list(camera.image_size),
        "intrinsics": camera.intrinsics.reshape(-1).tolist(),
    }


def _point_cloud_frame(
    frame_index: int, timestamp_ns: int, points: np.ndarray
) -> dict[str, Any]:
    little_endian = np.ascontiguousarray(points, dtype="<f4")
    return {
        "frame_index": frame_index,
        "timestamp_ns": timestamp_ns,
        "shape": [len(little_endian), 4],
        "dtype": "float32-le",
        "fields": ["x", "y", "z", "intensity"],
        "data": base64.b64encode(little_endian.tobytes()).decode("ascii"),
    }


def _ego_pose_frame(frame_index: int, timestamp_ns: int, matrix: np.ndarray) -> dict[str, Any]:
    return {
        "frame_index": frame_index,
        "timestamp_ns": timestamp_ns,
        "world_from_imu": np.asarray(matrix).reshape(-1).tolist(),
    }


def _camera_image_frame(
    camera_name: str,
    frame_index: int,
    timestamp_ns: int,
    source_timestamp_ns: int,
    delta_ns: int,
    image_format: str,
    payload: bytes,
) -> dict[str, Any]:
    return {
        "camera_name": camera_name,
        "frame_index": frame_index,
        "timestamp_ns": timestamp_ns,
        "source_timestamp_ns": source_timestamp_ns,
        "synchronization_delta_ns": delta_ns,
        "format": image_format,
        "data": base64.b64encode(payload).decode("ascii"),
    }


def _canonical_image(camera: CameraInput, value: Any) -> tuple[str, bytes]:
    if camera.image_format in {"png", "jpeg"}:
        return camera.image_format, bytes(value)
    from PIL import Image
    stream = BytesIO()
    Image.fromarray(np.asarray(value), mode="RGB").save(stream, format="PNG")
    return "png", stream.getvalue()


def _publish(
    writer: Writer,
    channel_id: int,
    timestamp_ns: int,
    payload: dict[str, Any],
    *,
    sequence: int = 0,
) -> None:
    writer.add_message(
        channel_id=channel_id,
        log_time=timestamp_ns,
        publish_time=timestamp_ns,
        sequence=sequence,
        data=_json_bytes(payload),
    )


def _timestamp_ns(timestamp: float) -> int:
    return int(round(float(timestamp) * 1_000_000_000))


def _json_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def _package_version() -> str:
    try:
        return version("lidar-camera-calibrator")
    except PackageNotFoundError:
        return "0+unknown"


def _library_identity() -> str:
    return f"lidar-camera-calibrator/{_package_version()}"
