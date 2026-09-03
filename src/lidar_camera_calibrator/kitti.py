"""Read-only adapter for raw KITTI odometry/sync sequences (directory or zip)."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from io import BytesIO
from pathlib import Path
import re
from typing import Iterable
import zipfile

import numpy as np
from .models import CameraCalibration, CameraFrame, FrameBundle, LoadedCalibration
from .transforms import make_transform


def _parse_values(line: str) -> np.ndarray:
    value = line.split(":", 1)[1].strip()
    try:
        return np.fromstring(value, sep=" ")
    except ValueError:
        return np.array([], dtype=float)


def _parse_calibration_text(text: str) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for raw in text.splitlines():
        if ":" not in raw or not raw.strip():
            continue
        key, _ = raw.split(":", 1)
        result[key.strip()] = _parse_values(raw)
    return result


def _matrix(values: np.ndarray, rows: int, cols: int) -> np.ndarray:
    if values.size != rows * cols:
        raise ValueError(f"expected {rows * cols} calibration values, got {values.size}")
    return values.reshape(rows, cols)


def _timestamp(value: str) -> float:
    value = value.strip()
    try:
        return float(value)
    except ValueError:
        # KITTI timestamps are UTC-less ISO strings; relative seconds are enough
        # for nearest-neighbour synchronization.
        return datetime.fromisoformat(value).timestamp()


class _Source:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.zf = zipfile.ZipFile(self.path) if self.path.is_file() and self.path.suffix == ".zip" else None
        self._names = self.zf.namelist() if self.zf else []

    def find(self, suffix: str) -> str | Path:
        if self.zf:
            matches = [n for n in self._names if n.endswith(suffix)]
            if not matches:
                raise FileNotFoundError(f"{suffix} not found in {self.path}")
            return matches[0]
        candidate = self.path / suffix
        if candidate.exists():
            return candidate
        # Permit callers to pass the sequence's enclosing directory.
        matches = list(self.path.rglob(Path(suffix).name))
        if not matches:
            raise FileNotFoundError(candidate)
        return matches[0]

    def read(self, name: str | Path) -> bytes:
        return self.zf.read(str(name)) if self.zf else Path(name).read_bytes()

    def glob(self, pattern: str) -> list[str | Path]:
        if self.zf:
            import fnmatch
            return [n for n in self._names if fnmatch.fnmatch(n, pattern)]
        return list(self.path.rglob(pattern))


@dataclass
class KittiAdapter:
    calibration_path: str | Path
    sequence_path: str | Path
    sync_tolerance_seconds: float = 0.05
    camera_ids: tuple[str, ...] = ("00", "01", "02", "03")

    def __post_init__(self) -> None:
        self.calibration_path = Path(self.calibration_path)
        self.sequence_path = Path(self.sequence_path)
        self._calib_source = _Source(self.calibration_path)
        self._sequence_source = _Source(self.sequence_path)
        self.calibration = self._load_calibration()
        self._lidar_files = self._sorted_frames(self._sequence_source.glob("*velodyne_points/data/*.bin"))
        if not self._lidar_files:
            self._lidar_files = self._sorted_frames(self._sequence_source.glob("*.bin"))
        self._lidar_times = self._load_times("oxts/timestamps.txt", len(self._lidar_files))
        self._camera_files: dict[str, list[str | Path]] = {
            cid: self._sorted_frames(self._sequence_source.glob(f"*image_{cid}/data/*.png"))
            for cid in self.camera_ids
        }
        self._camera_times = {
            cid: self._load_times(f"image_{cid}/timestamps.txt", len(files))
            for cid, files in self._camera_files.items()
        }

    @staticmethod
    def _sorted_frames(files: Iterable[str | Path]) -> list[str | Path]:
        def key(path: str | Path) -> tuple[int, str]:
            match = re.search(r"(\d+)(?:\.[^.]+)?$", str(path))
            return (int(match.group(1)) if match else -1, str(path))
        return sorted(files, key=key)

    def _load_times(self, suffix: str, count: int) -> np.ndarray:
        try:
            values = self._sequence_source.read(self._sequence_source.find(suffix)).decode().splitlines()
            return np.array([_timestamp(v) for v in values], dtype=float)
        except FileNotFoundError:
            return np.arange(count, dtype=float)

    def _load_calibration(self) -> LoadedCalibration:
        cam = _parse_calibration_text(self._calib_source.read(self._calib_source.find("calib_cam_to_cam.txt")).decode())
        velo = _parse_calibration_text(self._calib_source.read(self._calib_source.find("calib_velo_to_cam.txt")).decode())
        velo_t = make_transform(_matrix(velo["R"], 3, 3), velo["T"])
        cameras: dict[str, CameraCalibration] = {}
        for cid in self.camera_ids:
            s = _matrix(cam[f"S_rect_{cid}"], 1, 2).astype(int).ravel()
            p = _matrix(cam[f"P_rect_{cid}"], 3, 4)
            k = p[:, :3].copy()
            # T_00 is identity; R_i/T_i map camera-00 coordinates to camera-i.
            r = _matrix(cam.get(f"R_{cid}", np.eye(3).ravel()), 3, 3)
            t = cam.get(f"T_{cid}", np.zeros(3))
            camera_from_00 = make_transform(r, t)
            rect = _matrix(cam[f"R_rect_{cid}"], 3, 3)
            d = cam.get(f"D_{cid}", np.zeros(5))
            cameras[f"image_{cid}"] = CameraCalibration(
                camera_id=f"image_{cid}", image_size=(int(s[0]), int(s[1])), intrinsics=k,
                projection_matrix=p, camera_from_camera00=camera_from_00,
                rectification=rect, distortion=d,
            )
        sequence = self.sequence_path.stem.replace("_sync", "")
        return LoadedCalibration(velo_t, cameras, sequence=sequence)

    def __len__(self) -> int:
        return len(self._lidar_files)

    def _load_image(self, source: _Source, name: str | Path) -> np.ndarray:
        try:
            from PIL import Image
        except ImportError as exc:
            raise RuntimeError("Pillow is required to load KITTI images") from exc
        with Image.open(BytesIO(source.read(name))) as image:
            return np.asarray(image.convert("RGB"))

    def frame(self, index: int) -> FrameBundle:
        if index < 0 or index >= len(self):
            raise IndexError(index)
        lidar_name = self._lidar_files[index]
        raw = np.frombuffer(self._sequence_source.read(lidar_name), dtype=np.float32)
        points = raw.reshape(-1, 4)
        timestamp = float(self._lidar_times[index])
        cameras: dict[str, CameraFrame | None] = {}
        for cid, files in self._camera_files.items():
            if not files:
                cameras[f"image_{cid}"] = None
                continue
            times = self._camera_times[cid]
            nearest = int(np.argmin(np.abs(times - timestamp))) if len(times) else index
            if len(times) and abs(float(times[nearest] - timestamp)) > self.sync_tolerance_seconds:
                cameras[f"image_{cid}"] = None
            else:
                nearest = min(nearest, len(files) - 1)
                cameras[f"image_{cid}"] = CameraFrame(
                    f"image_{cid}", float(times[nearest]) if len(times) else timestamp,
                    self._load_image(self._sequence_source, files[nearest]), nearest,
                )
        return FrameBundle(index, timestamp, points, cameras)

    def frames(self) -> Iterable[FrameBundle]:
        for index in range(len(self)):
            yield self.frame(index)


def load_kitti(calibration_path: str | Path, sequence_path: str | Path, **kwargs) -> KittiAdapter:
    return KittiAdapter(calibration_path, sequence_path, **kwargs)
