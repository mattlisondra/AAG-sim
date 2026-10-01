"""Constant-memory video output for long simulator rollouts."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


def _as_uint8(frame: Any) -> np.ndarray:
    """Convert one simulator frame without retaining the source object."""
    if hasattr(frame, "detach"):
        frame = frame.detach().cpu().numpy()
    array = np.asarray(frame)
    if array.dtype != np.uint8:
        array = (array * 255).astype(np.uint8) if array.max() <= 1 else array.astype(np.uint8)
    return np.ascontiguousarray(array)


class StreamingVideoWriter:
    """Encode frames as they arrive instead of keeping a rollout in RAM."""

    def __init__(self, path: Path, fps: int) -> None:
        self.path = Path(path)
        self.fps = fps
        self.frame_count = 0
        self._writer: Any | None = None

    def append(self, frame: Any) -> None:
        if self._writer is None:
            import imageio.v2 as imageio

            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._writer = imageio.get_writer(str(self.path), format="FFMPEG", fps=self.fps)
        self._writer.append_data(_as_uint8(frame))
        self.frame_count += 1

    def close(self) -> None:
        if self._writer is None:
            return
        self._writer.close()
        self._writer = None
        logger.info("Saved video → %s (%d frames)", self.path, self.frame_count)

    def __enter__(self) -> StreamingVideoWriter:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()
