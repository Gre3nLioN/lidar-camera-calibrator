# LiDAR Camera Calibrator

A PySide6 desktop application for **manual refinement of static LiDAR-to-camera calibration** over synchronized sequences.

It is built for teams that need to inspect and tighten a calibration after a vehicle rig is assembled: for high-quality labeling, sensor validation, map/vehicle experiments, or testing a new calibration hypothesis. Most rigs do not change frequently, so a careful manual refinement pass can be a practical way to get the alignment needed for a specific dataset or workflow.

## Use your own data

Normalize raw/proprietary files, Python/NumPy arrays, custom lazy loaders, or standard Foxglove MCAP into the generic sensor contract. KITTI is included as an optional, tested example pipeline.

The generic integration requires timestamped LiDAR point clouds, one or more camera streams with intrinsics, a connected sensor transform tree, and world-from-IMU poses. The package validates and synchronizes the result before atomically writing portable `lidar-camera-scene/1` MCAP.

## Features

- manual extrinsic and optional intrinsic refinement for arbitrary named cameras;
- synchronized LiDAR, camera, and interpolated ego-pose timeline;
- navigable CPU-rendered 3D point-cloud view and camera overlays;
- undo/redo, numerical editing, resets, comparison view, and JSON export/reload;
- direction-explicit transforms using `P_target = T_target_from_source @ P_source`;
- strict, versioned MCAP profile with package-embeddable Python APIs.

## Install

```bash
uv add lidar-camera-calibrator
```

## Workflow

```python
from lidar_camera_calibrator import CalibrationConfig, launch_calibrator, write_profile_mcap

source_config = build_source_config_from_your_data()
write_profile_mcap(source_config, "scene.mcap")
result = launch_calibrator("scene.mcap", CalibrationConfig())
```

The viewer accepts canonical scene MCAP:

```bash
python -m lidar_camera_calibrator scene.mcap
```

## Documentation and source

- Documentation: https://gre3nlion.github.io/lidar-camera-calibrator/
- Source and screenshots: https://github.com/Gre3nLioN/lidar-camera-calibrator
- License: Apache-2.0
