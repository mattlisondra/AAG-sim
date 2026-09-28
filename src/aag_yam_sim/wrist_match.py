"""Geometry and image transforms for D435i-as-D405 wrist-camera experiments.

The transform in this module is deliberately pinhole-only.  It compensates for
the first-order focal-length difference after the D435i has been moved farther
from the nominal grasp plane.  It cannot recover scene content that was never
inside the D435i RGB field of view, remove rolling-shutter artifacts, or replace
a hand-eye calibration.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import radians, tan
from typing import Any

import numpy as np


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Minimal pinhole camera model in pixels."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        values = np.asarray(
            [self.width, self.height, self.fx, self.fy, self.cx, self.cy], dtype=float
        )
        if not np.isfinite(values).all():
            raise ValueError("intrinsics must be finite")
        if self.width <= 0 or self.height <= 0 or self.fx <= 0 or self.fy <= 0:
            raise ValueError("image dimensions and focal lengths must be positive")

    @classmethod
    def from_fov(
        cls,
        width: int,
        height: int,
        horizontal_deg: float,
        vertical_deg: float,
    ) -> PinholeIntrinsics:
        """Construct centered nominal intrinsics from horizontal and vertical FOV."""
        if not 1.0 < horizontal_deg < 179.0 or not 1.0 < vertical_deg < 179.0:
            raise ValueError("field-of-view angles must be in (1, 179) degrees")
        fx = (width / 2.0) / tan(radians(horizontal_deg) / 2.0)
        fy = (height / 2.0) / tan(radians(vertical_deg) / 2.0)
        return cls(width, height, fx, fy, width / 2.0, height / 2.0)

    @classmethod
    def from_matrix(cls, width: int, height: int, matrix: Any) -> PinholeIntrinsics:
        array = np.asarray(matrix, dtype=float)
        if array.shape != (3, 3) or not np.allclose(array[2], [0.0, 0.0, 1.0]):
            raise ValueError("intrinsic matrix must be a 3x3 pinhole matrix")
        return cls(width, height, array[0, 0], array[1, 1], array[0, 2], array[1, 2])

    def matrix(self) -> np.ndarray:
        return np.asarray(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


# Datasheet nominal color FOVs at 16:9.  Factory intrinsics from the exact
# devices and stream profiles should replace these values on hardware.
D405_COLOR_NOMINAL = PinholeIntrinsics.from_fov(640, 360, 84.0, 58.0)
D435I_COLOR_NOMINAL = PinholeIntrinsics.from_fov(640, 360, 69.4, 42.5)


def rotate_intrinsics_180(intrinsics: PinholeIntrinsics) -> PinholeIntrinsics:
    """Return intrinsics for the same pixels after an exact 180-degree rotation."""
    return PinholeIntrinsics(
        width=intrinsics.width,
        height=intrinsics.height,
        fx=intrinsics.fx,
        fy=intrinsics.fy,
        cx=(intrinsics.width - 1) - intrinsics.cx,
        cy=(intrinsics.height - 1) - intrinsics.cy,
    )


@dataclass(frozen=True)
class WristMatchPlan:
    """Physical distance recommendation and deterministic crop/resize plan."""

    reference: PinholeIntrinsics
    source: PinholeIntrinsics
    reference_distance_mm: float
    actual_distance_mm: float
    minimum_distance_scale: float
    actual_distance_scale: float
    crop_left: int
    crop_top: int
    crop_width: int
    crop_height: int
    output_width: int
    output_height: int
    effective: PinholeIntrinsics

    @property
    def extra_distance_mm(self) -> float:
        return self.actual_distance_mm - self.reference_distance_mm

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["extra_distance_mm"] = self.extra_distance_mm
        return result


def compute_match_plan(
    reference: PinholeIntrinsics,
    source: PinholeIntrinsics,
    *,
    reference_distance_mm: float,
    actual_distance_mm: float | None = None,
) -> WristMatchPlan:
    """Compute a crop that matches reference projection at one working plane.

    The source camera must first be moved farther from the working plane.  The
    minimum feasible distance scale is the larger of ``source.fx/reference.fx``
    and ``source.fy/reference.fy``.  At a smaller scale at least one axis would
    require padding missing scene content rather than cropping real pixels.

    This is a planar first-order match.  Parallax means no single 2-D transform
    can exactly match objects at every depth when the camera center has moved.
    """
    if reference_distance_mm <= 0:
        raise ValueError("reference_distance_mm must be positive")
    if (reference.width, reference.height) != (source.width, source.height):
        raise ValueError("reference and source stream sizes must match before planning")

    ratio_x = source.fx / reference.fx
    ratio_y = source.fy / reference.fy
    minimum_scale = max(ratio_x, ratio_y)
    if actual_distance_mm is None:
        actual_distance_mm = reference_distance_mm * minimum_scale
    if actual_distance_mm <= 0:
        raise ValueError("actual_distance_mm must be positive")
    actual_scale = actual_distance_mm / reference_distance_mm
    if actual_scale + 1e-9 < minimum_scale:
        required = reference_distance_mm * minimum_scale
        raise ValueError(
            "D435i is too close for a crop-only match: "
            f"need at least {required:.2f} mm from the working plane, "
            f"got {actual_distance_mm:.2f} mm"
        )

    crop_width_float = source.width * ratio_x / actual_scale
    crop_height_float = source.height * ratio_y / actual_scale
    crop_width = min(source.width, max(2, int(round(crop_width_float))))
    crop_height = min(source.height, max(2, int(round(crop_height_float))))

    # Choose the crop origin so the source principal point maps to the
    # reference principal point after resize, then clamp for device-specific
    # principal points near the edge.
    left = int(round(source.cx - reference.cx * crop_width / reference.width))
    top = int(round(source.cy - reference.cy * crop_height / reference.height))
    left = min(max(left, 0), source.width - crop_width)
    top = min(max(top, 0), source.height - crop_height)

    scale_x = reference.width / crop_width
    scale_y = reference.height / crop_height
    effective = PinholeIntrinsics(
        width=reference.width,
        height=reference.height,
        fx=source.fx * scale_x,
        fy=source.fy * scale_y,
        cx=(source.cx - left) * scale_x,
        cy=(source.cy - top) * scale_y,
    )
    return WristMatchPlan(
        reference=reference,
        source=source,
        reference_distance_mm=float(reference_distance_mm),
        actual_distance_mm=float(actual_distance_mm),
        minimum_distance_scale=float(minimum_scale),
        actual_distance_scale=float(actual_scale),
        crop_left=left,
        crop_top=top,
        crop_width=crop_width,
        crop_height=crop_height,
        output_width=reference.width,
        output_height=reference.height,
        effective=effective,
    )


def _resize_bilinear_numpy(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Dependency-free bilinear fallback; OpenCV is preferred for live capture."""
    src_height, src_width = image.shape[:2]
    if (src_width, src_height) == (width, height):
        return image.copy()

    x = np.linspace(0.0, src_width - 1.0, width)
    y = np.linspace(0.0, src_height - 1.0, height)
    x0 = np.floor(x).astype(np.int64)
    y0 = np.floor(y).astype(np.int64)
    x1 = np.minimum(x0 + 1, src_width - 1)
    y1 = np.minimum(y0 + 1, src_height - 1)
    wx = (x - x0)[None, :, None]
    wy = (y - y0)[:, None, None]

    top = image[y0[:, None], x0[None, :]] * (1.0 - wx) + image[y0[:, None], x1[None, :]] * wx
    bottom = (
        image[y1[:, None], x0[None, :]] * (1.0 - wx)
        + image[y1[:, None], x1[None, :]] * wx
    )
    resized = top * (1.0 - wy) + bottom * wy
    if np.issubdtype(image.dtype, np.integer):
        resized = np.rint(resized).clip(np.iinfo(image.dtype).min, np.iinfo(image.dtype).max)
    return resized.astype(image.dtype, copy=False)


def apply_match_plan(
    frame: np.ndarray,
    plan: WristMatchPlan,
    *,
    rotate_180: bool = False,
    backend: str = "auto",
) -> np.ndarray:
    """Apply the plan to an RGB or BGR frame without changing channel order."""
    array = np.asarray(frame)
    if array.ndim not in (2, 3):
        raise ValueError(f"frame must be HxW or HxWxC, got shape {array.shape}")
    if array.shape[:2] != (plan.source.height, plan.source.width):
        raise ValueError(
            f"frame is {array.shape[1]}x{array.shape[0]}, expected "
            f"{plan.source.width}x{plan.source.height}"
        )
    if backend not in {"auto", "opencv", "numpy"}:
        raise ValueError("backend must be 'auto', 'opencv', or 'numpy'")

    if rotate_180:
        array = np.ascontiguousarray(array[::-1, ::-1])
    y0, x0 = plan.crop_top, plan.crop_left
    cropped = array[y0 : y0 + plan.crop_height, x0 : x0 + plan.crop_width]

    if backend in {"auto", "opencv"}:
        try:
            import cv2

            return cv2.resize(
                cropped,
                (plan.output_width, plan.output_height),
                interpolation=cv2.INTER_LINEAR,
            )
        except ImportError:
            if backend == "opencv":
                raise RuntimeError("OpenCV is required for backend='opencv'") from None
    return _resize_bilinear_numpy(cropped, plan.output_width, plan.output_height)
