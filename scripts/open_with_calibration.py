#!/usr/bin/env python3
"""Open one canonical scene with a previously exported multi-camera calibration."""
from pathlib import Path

from lidar_camera_calibrator import CalibrationConfig, launch_calibrator

PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Edit these two paths when embedding this workflow in another application.
SCENE_PATH = PROJECT_ROOT / "output" / "raw-numpy-scene.mcap"
CALIBRATION_PATH = PROJECT_ROOT / "output" / "test-calibration.json"


def main() -> int:
    result = launch_calibrator(
        SCENE_PATH,
        CalibrationConfig(
            initial_override=CALIBRATION_PATH,
            renderer_mode="cpu",
            window_title="Reloaded Multi-Camera Calibration",
        ),
    )
    print(
        "Final modified cameras: "
        + (", ".join(result.changed_cameras) if result.changed_cameras else "none")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
