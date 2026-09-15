# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the Python package follows [Semantic Versioning](https://semver.org/). The serialized MCAP contract is versioned separately as `lidar-camera-scene/1`.

## [Unreleased]

## [0.1.1] - 2026-09-15

### Fixed

- Use a dedicated screenshot-free PyPI project description while retaining screenshots in the GitHub README and documentation site.

## [0.1.0] - 2026-09-02

### Added

- Generic `SourceAdapterConfig` integration for Python sequences, NumPy arrays, and application-owned loaders.
- Atomic canonical `lidar-camera-scene/1` MCAP writer and strict lazy reader.
- Foxglove JSON/base64 MCAP and KITTI development adapters.
- Arbitrary-camera PySide6 calibration workspace with CPU LiDAR and camera overlays.
- Direction-explicit multi-camera JSON export and reload.
- Staged startup and continuously replenished frame buffering.

[Unreleased]: https://github.com/Gre3nLioN/lidar-camera-calibrator/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/Gre3nLioN/lidar-camera-calibrator/releases/tag/v0.1.1
[0.1.0]: https://github.com/Gre3nLioN/lidar-camera-calibrator/releases/tag/v0.1.0
