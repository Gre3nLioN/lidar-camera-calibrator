# Migration to canonical scene MCAP

## Raw KITTI

The former `wiki/projects/kitti` writer emitted generic Foxglove channels and could silently skip unsynchronized camera images. Its `write_mcap(...)` compatibility entry point now delegates to `kitti_source_config` and `write_profile_mcap`, producing strict `lidar-camera-scene/1` output.

Preferred conversion and launch:

```bash
uv run --active python scripts/kitti_to_calibrator.py \
  --calibration /data/2011_09_26 \
  --sequence /data/2011_09_26_drive_0001_sync \
  --output output/scene.mcap
```

The KITTI adapter:

- reads Velodyne and image payloads lazily;
- derives rectified direct camera-from-LiDAR transforms from `R_rect_00` and `P_rect_XX`;
- converts OXTS geodetic localization to world-from-IMU poses normalized at the first frame;
- associates synchronized OXTS packet indexes with the LiDAR timeline;
- fails on missing files, count mismatches, invalid calibration, or synchronization gaps.

## Existing Foxglove JSON/base64 MCAP

Standard Foxglove JSON recordings can be normalized explicitly:

```python
from lidar_camera_calibrator import foxglove_source_config, write_profile_mcap

source = foxglove_source_config(
    "legacy.mcap",
    tf_topic="/tf",
    lidar_topic="/lidar/points",
    world_frame="world",
    imu_frame="imu",
)
write_profile_mcap(source, "scene.mcap")
```

The adapter requires the standard schema names `foxglove.FrameTransform`, `foxglove.CameraCalibration`, `foxglove.CompressedImage`, and `foxglove.PointCloud`. Camera topics pair by replacing `/calibration` with `/image`. FLOAT32 `x`, `y`, and `z` point fields are mandatory; intensity is optional and defaults to zero. Calibration rectification and projection offsets are folded into each canonical camera transform.

A single static world-from-IMU transform becomes a constant pose over the LiDAR timeline. Dynamic world-from-IMU transforms retain their source timestamps and must bracket the complete LiDAR range.

## Direct NumPy integration

Run the complete two-camera example:

```bash
uv run --active python examples/numpy_scene.py --frames 100 --launch
```

Use `SourceAdapterConfig` directly when points and RGB arrays already exist in memory. See `examples/numpy_scene.py` for synthetic construction. To see real raw files loaded eagerly into plain lists—without calling the KITTI adapter—run:

```bash
uv run --active python scripts/raw_numpy_to_calibrator.py --frames 10
```

That script reads `.bin`, PNG, timestamp, calibration, and OXTS files itself, creates lists of NumPy arrays, then calls `SourceAdapterConfig`, `write_profile_mcap`, and `launch_calibrator` in order.

## Viewer invocation

Raw paths and arbitrary MCAP files are no longer accepted by the viewer:

```bash
python -m lidar_camera_calibrator scene.mcap
```

Use an adapter first. This boundary keeps synchronization, transform interpretation, and malformed-data policy deterministic and testable.
