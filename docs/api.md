# Python API

The package is designed to be embedded in a Python data workflow. Source conversion and UI launch are separate operations.

## Canonical workflow

```python
from lidar_camera_calibrator import CalibrationConfig, launch_calibrator, write_profile_mcap

scene_path = write_profile_mcap(source_config, "scene.mcap")
result = launch_calibrator(scene_path, CalibrationConfig())
```

## Source configuration

### `SourceAdapterConfig`

```python
SourceAdapterConfig(
    lidar: PointCloudInput,
    cameras: Sequence[CameraInput],
    static_transforms: Sequence[TransformSpec],
    imu: EgoPoseStreamSpec,
    synchronization: SynchronizationConfig = SynchronizationConfig(),
)
```

The generic integration contract. It requires one LiDAR stream, one or more cameras, a connected sensor tree, and world-from-IMU samples.

### `PointCloudInput`

```python
PointCloudInput(timestamps, frames, format="xyzi-f32")
```

Supported formats are `xyzi-f32` (`N×4`) and `xyz-f32` (`N×3`). Frames may be ordinary NumPy arrays or lazily returned by an application-owned sequence.

### `CameraInput`

```python
CameraInput(name, timestamps, images, image_format, intrinsics, image_size)
```

Supported image formats are `numpy-rgb`, `png`, and `jpeg`. `intrinsics` is a pinhole `3×3` matrix and `image_size` is `(width, height)`.

### `EgoPoseStreamSpec` and `TimedTransform`

```python
EgoPoseStreamSpec([
    TimedTransform(timestamp_seconds, world_from_imu_matrix),
    ...,
])
```

At least two strictly ordered samples are required. Translation is interpolated linearly and rotation with quaternion SLERP at LiDAR timestamps. Samples must bracket the complete timeline.

### `FrameRef` and `TransformSpec`

```python
TransformSpec(
    target=FrameRef.camera("front"),
    source=FrameRef.lidar(),
    matrix=T_camera_from_lidar,
)
```

All matrices use `P_target = T_target_from_source @ P_source`. The transform graph must be connected and acyclic across IMU, LiDAR, and all cameras.

### `SynchronizationConfig`

```python
SynchronizationConfig(method="nearest", max_delta_ms=50.0)
```

Methods are `nearest`, `previous`, and `next`. Every camera must match every LiDAR frame within the configured limit.

## Profile writer and reader

### `write_profile_mcap(config, output_path)`

Validates, synchronizes, and atomically writes `lidar-camera-scene/1`. An existing destination is preserved if conversion fails.

### `SceneMcapReader(path)`

Strict lazy reader for canonical scenes.

Useful members:

- `len(reader)` — frame count;
- `reader.camera_names` — ordered camera identities;
- `reader.calibration` — immutable normalized calibration;
- `reader.frame(index)` — LiDAR and all camera payloads for one frame;
- `reader.ego_pose(index)` — interpolated `world_from_imu` matrix;
- `reader.transform_between(target, source)` — semantic transform lookup;
- `reader.calibration_edge(camera_name)` — camera-adjacent editable edge.

## Viewer

### `CalibrationConfig`

```python
CalibrationConfig(
    frame_limit=None,
    preload_count=2,
    renderer_mode="cpu",
    playback_speed=4.0,
    window_title=None,
    initial_override=None,
)
```

`renderer_mode="cpu"` is the complete default workspace. `launch_calibrator` blocks until the window closes.

### `launch_calibrator(scene_mcap, config=None)`

Opens a canonical scene and returns `CalibrationResult`. It can own a new `QGuiApplication` or run a blocking nested event loop when embedded in an existing Qt application.

### `CalibrationResult`

Immutable result with:

- `camera_edges`: mapping of camera names to `CameraEdgeCalibration`;
- `intrinsics`: final `3×3` matrices;
- `matrices`: convenience mapping to final edge matrices;
- `changed_cameras`: cameras whose edge matrix changed.

Each `CameraEdgeCalibration` retains `target`, `source`, `original_matrix`, and final `matrix`, so direction is never inferred from a generic key.

## Included adapters

### `foxglove_source_config(path, **options)`

Reads Foxglove JSON/base64 MCAP with `foxglove.FrameTransform`, `foxglove.CameraCalibration`, `foxglove.CompressedImage`, and `foxglove.PointCloud` schemas. Point `x`, `y`, and `z` fields must be FLOAT32; intensity is optional.

### `kitti_source_config(calibration_root, sequence_root, **options)`

Development/regression adapter for raw KITTI drives. It is not required for normal package use and does not define the generic API.

## Errors

Profile/configuration exceptions expose stable context fields such as `code`, `path`, `expected`, `actual`, and `hint`:

```python
from lidar_camera_calibrator import ProfileError

try:
    write_profile_mcap(source_config, "scene.mcap")
except ProfileError as error:
    print(error.code, error.path, error.hint)
```

Malformed data, incomplete synchronization, invalid transform graphs, incompatible overrides, and corrupt profiles fail explicitly. The package does not silently drop frames.
