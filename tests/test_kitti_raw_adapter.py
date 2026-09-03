from __future__ import annotations

from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image
import pytest

from lidar_camera_calibrator import FrameRef, SceneMcapReader, write_profile_mcap
from lidar_camera_calibrator.adapters import kitti_source_config
from lidar_camera_calibrator.profile import ConfigurationError, SourceDataError


_TIMESTAMPS = (
    "2011-09-26 13:02:25.951199337\n"
    "2011-09-26 13:02:26.054281661\n"
)


def _write_fixture(root: Path):
    calibration = root / "calibration"
    sequence = root / "sequence"
    calibration.mkdir()
    (calibration / "calib_imu_to_velo.txt").write_text(
        "R: 1 0 0 0 1 0 0 0 1\nT: 0 0 0\n", encoding="utf-8"
    )
    (calibration / "calib_velo_to_cam.txt").write_text(
        "R: 1 0 0 0 1 0 0 0 1\nT: 1 2 3\n", encoding="utf-8"
    )
    (calibration / "calib_cam_to_cam.txt").write_text(
        "calib_time: ignored text\n"
        "R_rect_00: 1 0 0 0 1 0 0 0 1\n"
        "S_rect_00: 3 2\n"
        "P_rect_00: 10 0 1 0 0 10 1 0 0 0 1 0\n"
        "S_rect_01: 3 2\n"
        "P_rect_01: 10 0 1 -5 0 10 1 0 0 0 1 0\n",
        encoding="utf-8",
    )
    lidar_data = sequence / "velodyne_points" / "data"
    lidar_data.mkdir(parents=True)
    (sequence / "velodyne_points" / "timestamps.txt").write_text(_TIMESTAMPS, encoding="utf-8")
    np.array([[1, 2, 3, .5]], dtype="<f4").tofile(lidar_data / "0000000000.bin")
    np.array([[4, 5, 6, .7]], dtype="<f4").tofile(lidar_data / "0000000001.bin")

    for camera_index in (0, 1):
        camera_root = sequence / f"image_{camera_index:02d}"
        data = camera_root / "data"
        data.mkdir(parents=True)
        camera_root.joinpath("timestamps.txt").write_text(_TIMESTAMPS, encoding="utf-8")
        for frame_index in range(2):
            Image.new("RGB", (3, 2), color=(camera_index, frame_index, 1)).save(
                data / f"{frame_index:010d}.png"
            )

    oxts_data = sequence / "oxts" / "data"
    oxts_data.mkdir(parents=True)
    (sequence / "oxts" / "timestamps.txt").write_text(_TIMESTAMPS, encoding="utf-8")
    oxts_data.joinpath("0000000000.txt").write_text(
        "49.0 8.0 100.0 0 0 0 0 0\n", encoding="utf-8"
    )
    oxts_data.joinpath("0000000001.txt").write_text(
        "49.000001 8.000001 100.1 0 0 0.01 0 0\n", encoding="utf-8"
    )
    return calibration, sequence


def test_raw_kitti_adapter_is_lazy_and_round_trips_canonical_scene(tmp_path):
    calibration, sequence = _write_fixture(tmp_path)
    config = kitti_source_config(
        calibration, sequence, camera_names=("image_00", "image_01")
    )
    assert len(config.lidar.timestamps) == 2
    assert [camera.name for camera in config.cameras] == ["image_00", "image_01"]
    assert config.lidar.frames.__class__.__name__ == "_PointFileSequence"
    assert config.cameras[0].images.__class__.__name__ == "_ByteFileSequence"
    np.testing.assert_allclose(config.lidar.validated_frame(0), [[1, 2, 3, .5]])

    scene = write_profile_mcap(config, tmp_path / "scene.mcap")
    reader = SceneMcapReader(scene)
    assert reader.camera_names == ("image_00", "image_01")
    np.testing.assert_allclose(reader.frame(1).lidar_points, [[4, 5, 6, .7]])
    np.testing.assert_allclose(
        reader.transform_between(FrameRef.camera("image_00"), FrameRef.lidar())[:3, 3],
        [1, 2, 3],
    )
    np.testing.assert_allclose(
        reader.transform_between(FrameRef.camera("image_01"), FrameRef.lidar())[:3, 3],
        [.5, 2, 3],
    )
    assert np.linalg.norm(reader.ego_pose(1)[:3, 3]) > 0


def test_raw_kitti_adapter_frame_limit_and_camera_selection(tmp_path):
    calibration, sequence = _write_fixture(tmp_path)
    config = kitti_source_config(
        calibration, sequence, camera_names=("image_01",), frame_limit=1
    )
    assert len(config.lidar.timestamps) == 1
    assert tuple(camera.name for camera in config.cameras) == ("image_01",)
    assert len(config.imu.samples) == 2  # interpolation contract retains a bracket


def test_kitti_conversion_script_writes_profile_without_launching(tmp_path):
    calibration, sequence = _write_fixture(tmp_path)
    output = tmp_path / "script-scene.mcap"
    script = Path(__file__).resolve().parents[1] / "scripts/kitti_to_calibrator.py"
    completed = subprocess.run(
        [
            sys.executable, str(script),
            "--calibration", str(calibration),
            "--sequence", str(sequence),
            "--output", str(output),
            "--cameras", "image_01",
            "--frame-limit", "1",
            "--no-launch",
        ],
        cwd=script.parent.parent,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "Wrote canonical profile" in completed.stdout
    assert SceneMcapReader(output).camera_names == ("image_01",)


def test_raw_kitti_adapter_reports_unknown_camera_and_count_mismatch(tmp_path):
    calibration, sequence = _write_fixture(tmp_path)
    with pytest.raises(ConfigurationError, match="KITTI_CAMERA_NOT_FOUND"):
        kitti_source_config(calibration, sequence, camera_names=("image_09",))

    (sequence / "image_00" / "data" / "0000000001.png").unlink()
    with pytest.raises(SourceDataError) as error:
        kitti_source_config(calibration, sequence, camera_names=("image_00",))
    assert error.value.code == "KITTI_SOURCE_COUNT_MISMATCH"
    assert error.value.path == "cameras['image_00']"
