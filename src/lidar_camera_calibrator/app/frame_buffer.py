"""Small immutable frame buffer with ordered background prefetch."""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from threading import RLock
from typing import Any


class BufferedFrameAdapter:
    """Expose a bounded prefix of an adapter while prefetching after startup."""

    def __init__(self, source: Any, *, frame_limit: int = 5, preload_count: int = 2) -> None:
        if frame_limit <= 0 or preload_count < 0:
            raise ValueError("frame limits must be valid")
        self.source = source
        self.calibration = source.calibration
        self._count = min(int(frame_limit), len(source))
        self._lock = RLock()
        self._frames: dict[int, object] = {}
        self._errors: dict[int, Exception] = {}
        self._futures: dict[int, Future] = {}
        self._executor: ThreadPoolExecutor | None = None
        for index in range(min(int(preload_count), self._count)):
            self._frames[index] = source.frame(index)
        if len(self._frames) < self._count:
            self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="kitti-frame-buffer")
            for index in range(len(self._frames), self._count):
                future = self._executor.submit(source.frame, index)
                self._futures[index] = future
                future.add_done_callback(lambda completed, frame_index=index: self._publish(frame_index, completed))

    def _publish(self, index: int, future: Future) -> None:
        try:
            frame = future.result()
        except Exception as exc:  # surfaced when that frame is requested
            with self._lock:
                self._errors[index] = exc
            return
        with self._lock:
            self._frames[index] = frame

    def __len__(self) -> int:
        return self._count

    @property
    def buffered_count(self) -> int:
        with self._lock:
            return len(self._frames)

    @property
    def buffered_indices(self) -> tuple[int, ...]:
        with self._lock:
            return tuple(sorted(self._frames))

    def is_ready(self, index: int) -> bool:
        with self._lock:
            return int(index) in self._frames

    def frame(self, index: int):
        index = int(index)
        if index < 0 or index >= self._count:
            raise IndexError(index)
        with self._lock:
            frame = self._frames.get(index)
            error = self._errors.get(index)
            future = self._futures.get(index)
        if frame is not None:
            return frame
        if error is not None:
            raise RuntimeError(f"failed to buffer frame {index}") from error
        if future is None:
            raise RuntimeError(f"frame {index} has no buffer task")
        frame = future.result()
        with self._lock:
            self._frames[index] = frame
        return frame

    def shutdown(self) -> None:
        if self._executor is not None:
            self._executor.shutdown(wait=True, cancel_futures=False)
            self._executor = None
        shutdown = getattr(self.source, "shutdown", None)
        if callable(shutdown):
            shutdown()
