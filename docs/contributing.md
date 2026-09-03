# Contributing

The repository contribution guide is maintained in [`CONTRIBUTING.md`](https://github.com/Gre3nLioN/lidar-camera-calibrator/blob/main/CONTRIBUTING.md).

Contributions are welcome for source adapters, profile tooling, calibration UX, CPU rendering, platform validation, tests, and documentation.

Important project rules:

- do not make the viewer infer arbitrary source semantics;
- normalize sources into `SourceAdapterConfig` before viewing;
- preserve `P_target = T_target_from_source @ P_source`;
- do not silently drop malformed or unsynchronized frames;
- treat the frozen `lidar-camera-scene/1` schemas as immutable.
