# Integrating your data

The package supports **any source that your application can translate into the generic sensor contract**. It does not require a particular folder structure, recorder, vehicle, camera count, camera name, or a KITTI dataset. KITTI is an optional tested pipeline; it is not the integration model.

## Required semantic data

A valid source provides:

1. LiDAR timestamps and `N×3` or `N×4` float point arrays;
2. one or more named cameras with timestamps, images, image sizes, and pinhole intrinsics;
3. static rigid transforms connecting IMU, LiDAR, and every camera in one acyclic tree;
4. timestamped `world_from_imu` poses covering every LiDAR timestamp;
5. a camera synchronization method and maximum accepted delta.

LiDAR timestamps define the canonical timeline. Every camera must match every LiDAR frame. Missing or malformed data fails conversion—partial frames and silent drops are not allowed.

## Python and NumPy data

The shortest integration uses ordinary sequences:

```python
import numpy as np
from lidar_camera_calibrator import (
    CameraInput,
    EgoPoseStreamSpec,
    FrameRef,
    PointCloudInput,
    SourceAdapterConfig,
    TimedTransform,
    TransformSpec,
    write_profile_mcap,
)

timestamps = [0.0, 0.1, 0.2]
point_frames = [
    np.asarray([[5.0, 0.0, 0.0, 0.8]], dtype=np.float32)
    for _ in timestamps
]
images = [np.zeros((720, 1280, 3), dtype=np.uint8) for _ in timestamps]
K = np.array([[900, 0, 640], [0, 900, 360], [0, 0, 1]], dtype=float)

T_lidar_from_imu = np.eye(4)
T_front_from_lidar = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1],
], dtype=float)
poses = [TimedTransform(t, np.eye(4)) for t in timestamps]

source = SourceAdapterConfig(
    lidar=PointCloudInput(timestamps, point_frames, format="xyzi-f32"),
    cameras=[
        CameraInput(
            "front",
            timestamps,
            images,
            "numpy-rgb",
            K,
            (1280, 720),
        )
    ],
    static_transforms=[
        TransformSpec(FrameRef.lidar(), FrameRef.imu(), T_lidar_from_imu),
        TransformSpec(FrameRef.camera("front"), FrameRef.lidar(), T_front_from_lidar),
    ],
    imu=EgoPoseStreamSpec(poses),
)

write_profile_mcap(source, "scene.mcap")
```

See `examples/numpy_scene.py` for an executable arbitrary two-camera scene.

## Raw files and custom loaders

Your files do not need to use a supported standard. Read them in application code and expose the resulting values to `SourceAdapterConfig`.

Examples include:

- binary point files plus PNG/JPEG folders;
- proprietary vehicle logs;
- database/blob-backed frames;
- ROS bag exports decoded by your own integration;
- network or object-store data materialized before profile writing.

Inputs need only behave like finite Python sequences with `len()` and indexed access. They may load values lazily, so a large dataset does not have to be eagerly materialized. Validation occurs when the writer requests each selected frame.

`scripts/raw_numpy_to_calibrator.py` demonstrates raw `.bin`, PNG, timestamps, calibration text, and pose files loaded into plain Python lists. It intentionally does **not** call the KITTI adapter—the source files happen to be KITTI samples, but the package boundary is generic.

## Foxglove MCAP

Standard Foxglove JSON/base64 recordings can be normalized directly:

```python
from lidar_camera_calibrator import foxglove_source_config, write_profile_mcap

source = foxglove_source_config(
    "recording.mcap",
    tf_topic="/tf",
    lidar_topic="/lidar/points",
    world_frame="world",
    imu_frame="imu",
    max_delta_ms=50.0,
)
write_profile_mcap(source, "scene.mcap")
```

Supported Foxglove schemas:

- `foxglove.FrameTransform`;
- `foxglove.CameraCalibration`;
- `foxglove.CompressedImage`;
- `foxglove.PointCloud`.

Camera calibration/image topics are paired by the conventional `/calibration` → `/image` replacement. Point `x`, `y`, and `z` fields must be FLOAT32. Intensity is optional and defaults to zero. Camera rectification and projection offsets are folded into canonical transforms.

This adapter reads standard Foxglove messages; it does not accept every arbitrary MCAP schema. For custom MCAP schemas, decode them in your application and build `SourceAdapterConfig`.

## KITTI development adapter

`kitti_source_config` exists to test the package against a known public dataset:

```python
from lidar_camera_calibrator import kitti_source_config, write_profile_mcap

source = kitti_source_config(calibration_root, sequence_root)
write_profile_mcap(source, "scene.mcap")
```

KITTI naming, OXTS conversion, calibration text parsing, and directory assumptions are isolated inside this optional adapter. They are not requirements imposed on user datasets.

## Transform graph

Use direction-explicit matrices:

```text
P_target = T_target_from_source @ P_source
```

For example, `TransformSpec(FrameRef.camera("front"), FrameRef.lidar(), matrix)` means the matrix transforms LiDAR-frame points into the front-camera frame.

The writer rejects disconnected graphs, cycles, duplicate camera names, non-rigid transforms, and ambiguous camera paths. The camera-adjacent edge toward IMU becomes that camera’s editable calibration edge.

## Synchronization

```python
from lidar_camera_calibrator import SynchronizationConfig

sync = SynchronizationConfig(method="nearest", max_delta_ms=50.0)
```

Available methods are `nearest`, `previous`, and `next`. IMU translation and rotation are interpolated to LiDAR timestamps. V1 intentionally has no clock-offset, pose extrapolation, or configurable IMU-gap behavior.
