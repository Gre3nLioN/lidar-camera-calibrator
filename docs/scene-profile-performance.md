# Scene profile performance baseline

Measured on the development Linux host on 2026-09-01 using the raw KITTI drive, 100 LiDAR frames, cameras `image_02` and `image_03`, and CPU rendering:

| Operation | Result |
|---|---:|
| KITTI config construction | 0.012 s |
| Atomic canonical MCAP write | 3.825 s |
| Output size | 295.9 MiB |
| Conversion peak RSS | 65.9 MiB |
| Reader index after lazy-index optimization | 0.769 s |
| Reader index peak / retained RSS | 56.9 / 53.0 MiB |
| Offscreen open, render, timed close | 2.450 s |
| Viewer process peak RSS | 346.0 MiB |
| Shutdown | clean, status 0 |

Before optimization, reader indexing retained every dynamic MCAP record while validating the profile. The same viewer workload peaked at 706.5 MiB. The reader now retains only `(log_time, sequence)` identities for dynamic point, image, and pose messages and seeks payloads when requested. This reduced viewer peak memory by **51.0%** while retaining complete startup validation and lazy frame decoding.

The offscreen launch used `preload_count=2`, Qt's software backend, and closed after 2.5 seconds. Logs contained no QML reference/type/anchor errors or failed image requests.

`tests/test_100_frame_regression.py` provides a platform-neutral lightweight 100-frame regression. It verifies profile write/index/last-frame decode under a generous non-quadratic runtime ceiling and runs the public blocking launch in an isolated process through clean shutdown. The real KITTI measurement is a baseline, not a universal performance guarantee.
