# Platform support

The complete workspace uses a CPU renderer and Qt/PySide6. A native GPU renderer is not required.

| Platform | Current status | Notes |
|---|---|---|
| Linux | Validated | Primary development and performance-test platform; Wayland, software/offscreen, and normal desktop launches have been exercised. |
| Windows | Supported, validation incomplete | Uses synchronous safety behavior for Qt/NumPy preparation to avoid a previously observed worker-thread heap-corruption failure. Full packaging and GUI workflow validation remains pending. |
| macOS | Experimental/unvalidated | Dependencies publish macOS builds and the architecture is platform-neutral, but CI and a real GUI calibration workflow have not yet been completed. |

## macOS expectations

The package should work on current PySide6-supported macOS versions because MCAP processing, calibration math, image decoding, and CPU rendering are implemented in Python/NumPy/Pillow. This is an engineering expectation, not a release guarantee.

Before macOS is marked supported, the project needs:

- clean wheel installation on Intel or Apple Silicon as applicable;
- canonical profile write/read coverage;
- offscreen QML resource smoke testing;
- a real desktop launch with frame changes and shutdown;
- native file-dialog export and calibration reload validation.

## Windows safety behavior

Do not force asynchronous preparation on Windows for production use without separate stress testing. The default synchronous path may pause briefly on cache misses, but avoids running the known-sensitive Qt/NumPy workload in background workers.

## Headless smoke test

On Linux CI or another Qt installation with an offscreen plugin:

```bash
QT_QPA_PLATFORM=offscreen \
QT_QUICK_BACKEND=software \
uv run python -m lidar_camera_calibrator scene.mcap
```

Headless success does not replace a real desktop interaction test.
