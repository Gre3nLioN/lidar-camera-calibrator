#!/usr/bin/env python3
"""Convert a raw KITTI sync drive to canonical scene MCAP and open the calibrator."""
from __future__ import annotations

import argparse
from pathlib import Path

from lidar_camera_calibrator import CalibrationConfig, launch_calibrator, write_profile_mcap
from lidar_camera_calibrator.adapters import kitti_source_config


PROJECT_ROOT = Path(__file__).resolve().parents[1]
KITTI_ROOT = PROJECT_ROOT.parent / "kitti"
DEFAULT_CALIBRATION = KITTI_ROOT / "2011_09_26_calib" / "2011_09_26"
DEFAULT_SEQUENCE = (
    KITTI_ROOT / "2011_09_26_drive_0001_sync" / "2011_09_26"
    / "2011_09_26_drive_0001_sync"
)
DEFAULT_OUTPUT = PROJECT_ROOT / "output" / "scene.mcap"


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Convert raw KITTI data to canonical scene.mcap and launch calibration."
    )
    result.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    result.add_argument("--sequence", type=Path, default=DEFAULT_SEQUENCE)
    result.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    result.add_argument(
        "--cameras", nargs="+", default=None, metavar="NAME",
        help="KITTI camera directories to include, for example image_02 image_03 (default: all)",
    )
    result.add_argument("--frame-limit", type=int, default=None)
    result.add_argument("--max-delta-ms", type=float, default=50.0)
    result.add_argument("--preload-count", type=int, default=2)
    result.add_argument(
        "--renderer", choices=("cpu", "production", "disabled"), default="cpu"
    )
    result.add_argument("--playback-speed", type=float, default=4.0)
    result.add_argument("--title", default="KITTI LiDAR Camera Calibration")
    result.add_argument(
        "--no-launch", action="store_true",
        help="write and validate scene.mcap without opening the desktop viewer",
    )
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    print(f"KITTI calibration: {args.calibration}")
    print(f"KITTI sequence:    {args.sequence}")
    print(f"Canonical output:  {args.output}")
    config = kitti_source_config(
        args.calibration,
        args.sequence,
        camera_names=args.cameras,
        frame_limit=args.frame_limit,
        max_delta_ms=args.max_delta_ms,
    )
    print(
        f"Converting {len(config.lidar.timestamps)} LiDAR frames and "
        f"{len(config.cameras)} cameras..."
    )
    output = write_profile_mcap(config, args.output)
    size_mb = output.stat().st_size / (1024 * 1024)
    print(f"Wrote canonical profile: {output} ({size_mb:.1f} MiB)")
    if args.no_launch:
        return 0
    print("Opening calibrator; close the window to return final overrides...")
    result = launch_calibrator(
        output,
        CalibrationConfig(
            preload_count=args.preload_count,
            renderer_mode=args.renderer,
            playback_speed=args.playback_speed,
            window_title=args.title,
        ),
    )
    print(
        "Changed cameras: "
        + (", ".join(result.changed_cameras) if result.changed_cameras else "none")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
