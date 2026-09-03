"""Versioned canonical scene-profile schema resources."""
from __future__ import annotations

import json
from importlib.resources import files
from typing import Any

PROFILE_NAME = "lidar-camera-scene"
PROFILE_VERSION = 1
SCHEMA_NAMES = (
    "scene-manifest",
    "static-transform-tree",
    "point-cloud-frame",
    "camera-calibration",
    "camera-image-frame",
    "ego-pose-frame",
    "scene-frame",
)


def load_schema(name: str) -> dict[str, Any]:
    if name not in SCHEMA_NAMES:
        raise KeyError(f"unknown scene schema {name!r}")
    resource = files("lidar_camera_calibrator.profile.schema_resources").joinpath(f"{name}.schema.json")
    return json.loads(resource.read_text(encoding="utf-8"))
