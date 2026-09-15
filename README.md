# LiDAR Camera Calibrator

A PySide6 desktop application for **manual refinement of static LiDAR-to-camera calibration** over synchronized sequences.

It is built for teams that need to inspect and tighten a calibration after a vehicle rig is assembled: for high-quality labeling, sensor validation, map/vehicle experiments, or testing a new calibration hypothesis. Most rigs do not change frequently, so a careful manual refinement pass can be a practical way to get the alignment needed for a specific dataset or workflow.

> **Use the data you already have.** Normalize raw files, Python/NumPy arrays, custom lazy loaders, or standard Foxglove MCAP into the generic sensor contract. KITTI is included as an optional, tested example pipeline.

![Main workspace: navigable LiDAR scene, synchronized camera previews, scene inspector, and timeline.](docs/assets/main-workspace.png)

![Calibration workspace: projected LiDAR points on the selected camera with live extrinsic and intrinsic controls.](docs/assets/calibration-workspace.png)

## Features

- manual extrinsic and optional intrinsic refinement for arbitrary named cameras;
- synchronized LiDAR, camera, and interpolated ego-pose timeline;
- navigable CPU-rendered 3D point-cloud view and camera overlays;
- direction-explicit transforms using `P_target = T_target_from_source @ P_source`;
- undo/redo, numerical editing, resets, comparison view, and JSON export/reload;
- strict, versioned `lidar-camera-scene/1` MCAP boundary;
- staged 10-frame startup and continuously replenished 40-frame rolling cache;
- package-embeddable Python API with no required application-specific CLI.

## Installation

Published releases can be installed with uv:

```bash
uv add lidar-camera-calibrator
```

For development from a checkout:

```bash
uv sync --locked
uv run python examples/numpy_scene.py --frames 100 --launch
```

## Manual calibration workflow

Convert application-owned data to the canonical scene profile, then launch the viewer:

```python
from lidar_camera_calibrator import CalibrationConfig, launch_calibrator, write_profile_mcap

source_config = build_source_config_from_your_data()
write_profile_mcap(source_config, "scene.mcap")
result = launch_calibrator(
    "scene.mcap",
    CalibrationConfig(initial_override="calibration.json"),
)
```

`SourceAdapterConfig` accepts ordinary Python sequences and NumPy arrays. Your integration decides how proprietary logs, folders, databases, ROS exports, or other formats are read. The package validates and synchronizes the resulting sensor values before atomically writing canonical MCAP.

For standard Foxglove JSON/base64 MCAP:

```python
from lidar_camera_calibrator import foxglove_source_config, write_profile_mcap

source = foxglove_source_config("recording.mcap")
write_profile_mcap(source, "scene.mcap")
```

The viewer itself deliberately accepts only canonical scene MCAP:

```bash
python -m lidar_camera_calibrator scene.mcap
```

## Documentation

- [Getting started](docs/getting-started.md)
- [Integrating your data](docs/data-integration.md)
- [Python API](docs/api.md)
- [Canonical MCAP profile](docs/scene-mcap-profile.md)
- [Platform support](docs/platform-support.md)
- [Contributing](CONTRIBUTING.md)

Build the GitHub Pages site locally with:

```bash
uv sync --only-group docs
uv run --no-sync mkdocs serve
```

## Platform status

| Platform | Status |
|---|---|
| Linux | Validated development platform |
| Windows | Supported CPU path; synchronous preparation is retained for safety, with full release workflow validation pending |
| macOS | Expected to work with the CPU/PySide6 stack, but currently experimental until CI and real GUI smoke testing are complete |

No native GPU renderer is currently shipped. The complete CPU workspace is the default on every platform.

## Development data

KITTI-specific helpers exist to exercise a known public dataset and regression-test transforms, synchronization, conversion, and rendering. They are optional:

```python
from lidar_camera_calibrator import kitti_source_config
```

New integrations should normally use `SourceAdapterConfig` directly or add a focused source adapter that produces it.

## License

Licensed under the [Apache License 2.0](LICENSE).
