# Release process

LiDAR Camera Calibrator is distributed as a Python wheel/sdist on PyPI and as signed/tagged artifacts attached to GitHub Releases. uv is the canonical build and publishing tool.

## Compatibility promises

- Python package versions follow semantic versioning.
- `lidar-camera-scene/1` is a separately versioned frozen serialization contract.
- Breaking schema or topic changes require a new scene-profile version and migration path.
- Calibration JSON identifies its profile and camera-edge directions explicitly.

## Pre-release checklist

1. Update `CHANGELOG.md` and remove changes from `Unreleased` into the release version.
2. Set the same version in `pyproject.toml`.
3. Run the full tests and documentation build:

   ```bash
   uv sync --locked
   uv run pytest
   uv run python -m compileall -q src tests scripts examples
   uv lock --check
   uv run --only-group docs mkdocs build --strict
   ```

4. Build and clean-install the wheel:

   ```bash
   uv run python tests/installed_wheel_smoke.py
   ```

5. Validate representative source paths:
   - direct NumPy/Python data;
   - raw application-owned file loading;
   - Foxglove standard MCAP conversion;
   - canonical MCAP open, calibration export, and reload.
6. Record real platform coverage. Unexecuted Windows or macOS workflows must remain marked unvalidated.

## Build artifacts

```bash
rm -rf dist
uv build
```

Expected artifacts:

- `dist/lidar_camera_calibrator-<version>-py3-none-any.whl`
- `dist/lidar_camera_calibrator-<version>.tar.gz`

Although the wheel is platform-neutral, runtime dependencies such as PySide6 install platform-specific wheels.

## Publish to TestPyPI first

```bash
uv publish \
  --publish-url https://test.pypi.org/legacy/ \
  --token "$TEST_PYPI_TOKEN" \
  dist/*
```

Install into a clean uv environment and repeat the packaged-resource smoke test before production publication.

## Publish to PyPI

```bash
uv publish --token "$PYPI_TOKEN" dist/*
```

For automation, prefer PyPI trusted publishing from a protected GitHub Actions environment rather than long-lived API tokens.

## Git tag and GitHub Release

```bash
git tag -s v0.1.0 -m "LiDAR Camera Calibrator 0.1.0"
git push origin v0.1.0
gh release create v0.1.0 dist/* \
  --title "LiDAR Camera Calibrator 0.1.0" \
  --generate-notes
```

Do not create a public release until its corresponding PyPI files and repository tag refer to the same commit and version.

## GitHub Pages

`.github/workflows/docs.yml` builds the locked MkDocs environment and deploys the site after documentation changes land on `main`. Enable **GitHub Actions** as the Pages source in repository settings once.
