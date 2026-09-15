# Contributing

Thank you for helping improve LiDAR Camera Calibrator.

## Development setup

```bash
git clone https://github.com/Gre3nLioN/lidar-camera-calibrator.git
cd lidar-camera-calibrator
uv sync --locked
uv run pytest
```

Build the documentation with:

```bash
uv sync --only-group docs
uv run --no-sync mkdocs build --strict
```

## Design boundaries

- The viewer accepts only canonical `lidar-camera-scene/1` MCAP.
- Source-specific interpretation belongs in adapters or application code that creates `SourceAdapterConfig`.
- LiDAR timestamps define the canonical timeline.
- Every camera must resolve one valid image for every LiDAR frame.
- IMU/world poses must cover the complete LiDAR timeline.
- Transform names and calculations follow `P_target = T_target_from_source @ P_source`.
- Conversion must fail explicitly rather than silently dropping or repairing data.
- The frozen v1 JSON schemas cannot be changed in place.

## Adding a data source

A source adapter should return `SourceAdapterConfig`; it should not launch the UI or write an alternative viewer format. Add tests for malformed input, synchronization boundaries, transform direction, and a canonical writer/reader round trip.

KITTI is a regression fixture, not an architectural template. Prefer source-neutral names and keep format assumptions inside the adapter.

## Quality checks

Before opening a pull request:

```bash
uv run pytest
uv run python -m compileall -q src tests scripts examples
uv lock --check
uv run --only-group docs mkdocs build --strict
uv run python tests/installed_wheel_smoke.py
```

Include focused tests for behavior changes and update user-facing documentation when public behavior changes.

## Maintainer release notes

Release instructions belong with project maintenance rather than the end-user documentation site. Before publishing, run the full test suite, strict documentation build, isolated wheel smoke test, and `uv build`; publish the matching artifacts to PyPI and a tagged GitHub Release. Preserve the frozen `lidar-camera-scene/1` contract or introduce a new profile version with a migration path.

## Pull requests

Keep changes focused, explain compatibility implications, and identify the platforms actually tested. Do not claim Windows or macOS validation based only on Linux/offscreen results.

Contributions are submitted under the project’s Apache-2.0 license.
