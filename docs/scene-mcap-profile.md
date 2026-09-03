# Canonical scene MCAP reference

## Identity and API

The viewer accepts only MCAP profile `lidar-camera-scene`, version `1`. Convert source data before launch:

```python
from lidar_camera_calibrator import CalibrationConfig, launch_calibrator, write_profile_mcap

write_profile_mcap(source_config, "scene.mcap")
result = launch_calibrator("scene.mcap", CalibrationConfig())
```

`write_profile_mcap` creates a sibling temporary file, completes and closes it, then atomically replaces the destination. A conversion failure never replaces an existing destination.

A single exported JSON file contains every modified camera. Reload it as initial working state without changing the scene MCAP:

```python
result = launch_calibrator(
    "scene.mcap",
    CalibrationConfig(initial_override="calibration.json"),
)
```

The loader strictly checks `lidar-camera-scene/1`, embedded camera identities, edge directions, rigid matrices, intrinsics, and compatibility with the opened scene. Imported changes start dirty with empty undo history; intrinsic controls are enabled when imported intrinsics exist.

## Required topics

| Topic | Cardinality | Packaged schema |
|---|---:|---|
| `/scene/manifest` | 1 | `scene-manifest.schema.json` |
| `/scene/static_transforms` | 1 | `static-transform-tree.schema.json` |
| `/scene/lidar/points` | one per frame | `point-cloud-frame.schema.json` |
| `/scene/imu/pose` | one per frame | `ego-pose-frame.schema.json` |
| `/scene/cameras/{name}/calibration` | one per camera | `camera-calibration.schema.json` |
| `/scene/cameras/{name}/image` | one per camera and frame | `camera-image-frame.schema.json` |
| `/scene/frames` | one per frame | `scene-frame.schema.json` |

Schemas are package resources under `lidar_camera_calibrator/profile/schema_resources/`. Channels use JSON messages and reference the exact packaged schema identity. Point data is little-endian float32 XYZI encoded as base64; images contain PNG or JPEG bytes encoded as base64.

## Transform semantics

Every matrix follows:

```text
P_target = T_target_from_source @ P_source
```

The static graph contains one IMU, one LiDAR, and every named camera. It must be connected and acyclic. The editable edge for a camera is the camera-adjacent edge on its unique path toward IMU. Calibration results and exports retain the original edge direction.

## Synchronization

- LiDAR timestamps define the canonical frame timeline and are never dropped.
- Each camera resolves exactly one image per LiDAR frame.
- Camera methods are `nearest`, `previous`, or `next`; the default is nearest within 50 ms.
- Equal nearest deltas select the earlier image.
- IMU translation is linearly interpolated at each LiDAR timestamp.
- IMU rotation uses quaternion SLERP.
- IMU clock offsets, extrapolation, and configurable gap handling are intentionally absent in v1.
- Any missing or malformed selected payload fails the complete conversion; partial frames are forbidden.

## Error handling

Configuration, source, synchronization, transform, profile, and profile-version failures use typed exceptions derived from `CalibrationProfileError`. Diagnostics expose stable `code`, `path`, `expected`, `actual`, and optional `hint` fields. Applications should catch the narrowest useful type:

```python
from lidar_camera_calibrator.profile import CalibrationProfileError

try:
    write_profile_mcap(config, "scene.mcap")
except CalibrationProfileError as error:
    print(error.code, error.path, error.hint)
```

Unknown mapping fields fail before output creation and include close-name suggestions. Corrupt canonical MCAP files fail while indexing or dynamically decoding the requested payload; the reader never silently repairs, drops, or guesses data.

## Reader memory model

`SceneMcapReader` retains static metadata and message indexes. Point, image, and pose payloads are decoded when a frame is requested. The UI frame buffer retains a bounded working window rather than materializing the complete scene.
