import hashlib
import json
from importlib.resources import files

from lidar_camera_calibrator.profile import PROFILE_NAME, PROFILE_VERSION, SCHEMA_NAMES, load_schema


def test_all_versioned_profile_schemas_are_packaged_and_strict():
    assert PROFILE_NAME == "lidar-camera-scene"
    assert PROFILE_VERSION == 1
    assert len(SCHEMA_NAMES) == 7
    for name in SCHEMA_NAMES:
        schema = load_schema(name)
        assert schema["$schema"].endswith("2020-12/schema")
        assert schema["title"].startswith("lidar-camera.")
        assert schema["type"] == "object"
        assert schema["additionalProperties"] is False
        assert schema["required"]


def test_v1_schema_resources_match_frozen_approved_hashes():
    expected = {
        "camera-calibration": "0aaa1d7f82b172d6f19b377071c3f5c3d6a0ae88ffc48893c1634a93b5fad9d6",
        "camera-image-frame": "d41d012dd7a4257b6cc7936d52f66bd6bc61f9da56641dd1caa8261b67033747",
        "ego-pose-frame": "60458d7664434ad441318bd57622719301a97864e7cc3b0756f9fc1086efbc38",
        "point-cloud-frame": "6ddf06824f129e139bac5dea0b269f6a1664b23a5ee83e11ac8b1c35008391cf",
        "scene-frame": "a1d4446952236777ae69819fb08dd313c5dbc745389a95c5068cfd7db04bab26",
        "scene-manifest": "126ab79b87a8ebd2fba14d92161b891436d3e891ac85fe9286d3b3cc04089501",
        "static-transform-tree": "55355c0dcce2f926176ac4e517af609fda1807204788991404c789ed45038ce6",
    }
    root = files("lidar_camera_calibrator.profile.schema_resources")
    actual = {
        name: hashlib.sha256(root.joinpath(f"{name}.schema.json").read_bytes()).hexdigest()
        for name in SCHEMA_NAMES
    }
    assert actual == expected


def test_schema_resources_are_valid_json_and_have_unique_ids():
    root = files("lidar_camera_calibrator.profile.schema_resources")
    schemas = [json.loads(root.joinpath(f"{name}.schema.json").read_text()) for name in SCHEMA_NAMES]
    assert len({schema["$id"] for schema in schemas}) == len(schemas)
    assert load_schema("scene-manifest")["properties"]["profile_name"]["const"] == PROFILE_NAME
