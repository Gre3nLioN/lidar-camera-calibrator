# Getting started

## Requirements

- Python 3.11 or newer
- Linux, Windows, or macOS
- a desktop environment supported by PySide6/Qt 6
- [uv](https://docs.astral.sh/uv/) for the documented package workflow

Linux is the validated development platform. See [Platform support](platform-support.md) before deploying on Windows or macOS.

## Install a release

After the package is published to PyPI:

```bash
uv init my-calibration-project
cd my-calibration-project
uv add lidar-camera-calibrator
```

Run Python through the managed environment:

```bash
uv run python -m lidar_camera_calibrator scene.mcap
```

## Install a development checkout

```bash
git clone https://github.com/Gre3nLioN/lidar-camera-calibrator.git
cd lidar-camera-calibrator
uv sync --locked
```

Run the generic NumPy example:

```bash
uv run python examples/numpy_scene.py --frames 100 --launch
```

This example has no KITTI dependency. It constructs arbitrary named cameras, point clouds, poses, and transforms directly in Python.

## Create a canonical scene

The viewer does not infer semantics from arbitrary files. First translate your source into `SourceAdapterConfig`:

```python
from lidar_camera_calibrator import write_profile_mcap

source = build_source_config_from_your_dataset()
scene_path = write_profile_mcap(source, "scene.mcap")
```

Writing is atomic. Validation or encoding failure leaves an existing destination unchanged.

See [Integrating data](data-integration.md) for direct NumPy, raw-file, custom-loader, and Foxglove workflows.

## Launch a canonical scene

From Python:

```python
from lidar_camera_calibrator import CalibrationConfig, launch_calibrator

result = launch_calibrator(
    "scene.mcap",
    CalibrationConfig(
        playback_speed=4.0,
        renderer_mode="cpu",
        window_title="Vehicle A calibration",
    ),
)

for camera_name, edge in result.camera_edges.items():
    print(camera_name, edge.target.label, edge.source.label, edge.matrix)
```

Or launch the package module:

```bash
uv run python -m lidar_camera_calibrator scene.mcap
```

## Reload calibration JSON

The UI exports one canonical JSON file containing every modified camera. Reopen it as working state without changing the MCAP:

```python
result = launch_calibrator(
    "scene.mcap",
    CalibrationConfig(initial_override="calibration.json"),
)
```

Overrides are checked against the profile identity, camera names, camera-adjacent edge identities and directions, rigid-transform constraints, and pinhole intrinsics.

## Runtime buffering

On non-Windows systems, the viewer:

1. prepares a 10-frame startup tier;
2. expands to a 40-frame rolling look-ahead cache;
3. replenishes the cache as playback or seeking advances;
4. preserves cumulative decoded-frame indicators on the timeline.

Windows deliberately keeps decoding/preparation synchronous where required by the established Qt/NumPy safety path. Decode failures stop playback and display the affected frame instead of waiting indefinitely.
