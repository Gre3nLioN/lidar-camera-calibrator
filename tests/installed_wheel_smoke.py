"""Build, clean-install, and validate package resources. Run directly, not via pytest."""
from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import tempfile


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv is required for the installed-wheel smoke test")
    with tempfile.TemporaryDirectory(prefix="lidar-calibrator-wheel-") as temporary:
        root = Path(temporary)
        distribution = root / "dist"
        environment = root / "venv"
        subprocess.run(
            [uv, "build", "--wheel", "--out-dir", str(distribution)],
            cwd=PROJECT_ROOT, check=True,
        )
        wheel = next(distribution.glob("*.whl"))
        subprocess.run([uv, "venv", str(environment), "--python", "3.11"], check=True)
        python = (
            environment / "Scripts" / "python.exe"
            if (environment / "Scripts" / "python.exe").exists()
            else environment / "bin" / "python"
        )
        subprocess.run(
            [uv, "pip", "install", "--python", str(python), str(wheel)], check=True
        )
        probe = """
from importlib.resources import files
from lidar_camera_calibrator import CalibrationConfig, SceneMcapReader, foxglove_source_config, kitti_source_config, launch_calibrator, write_profile_mcap
from lidar_camera_calibrator.profile import SCHEMA_NAMES, load_schema
assert files('lidar_camera_calibrator.ui').joinpath('qml/MainDisabled.qml').is_file()
assert files('lidar_camera_calibrator.ui').joinpath('qml/icons/compare-split.svg').is_file()
assert len(SCHEMA_NAMES) == 7
assert all(load_schema(name)['additionalProperties'] is False for name in SCHEMA_NAMES)
print('clean installed wheel and resources: ok')
"""
        subprocess.run([str(python), "-I", "-c", probe], cwd=root, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
