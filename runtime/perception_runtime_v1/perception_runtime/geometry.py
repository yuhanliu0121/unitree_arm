from __future__ import annotations

import math

import numpy as np

from .models import CameraIntrinsics


def deproject_pixels(
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Convert aligned pixels to the optical frame: +X right, +Y down, +Z forward."""
    xs = xs.astype(np.float32, copy=False)
    ys = ys.astype(np.float32, copy=False)
    zs = zs.astype(np.float32, copy=False)
    return np.column_stack(
        (
            (xs - intrinsics.cx) / intrinsics.fx * zs,
            (ys - intrinsics.cy) / intrinsics.fy * zs,
            zs,
        )
    ).astype(np.float32, copy=False)


def robust_pca_dimensions(
    points: np.ndarray,
    low_percentile: float = 5.0,
    high_percentile: float = 95.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return robust center, descending PCA dimensions and PCA axes.

    Percentiles reject isolated flying points without imposing a fixed depth band, so a
    long object that is genuinely tilted through depth is retained.
    """
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
        raise ValueError("points must be N x 3 with at least three samples")
    center = np.median(points, axis=0)
    centered = points - center
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    projected = centered @ vh.T
    low = np.percentile(projected, low_percentile, axis=0)
    high = np.percentile(projected, high_percentile, axis=0)
    dimensions = np.maximum(high - low, 0.0)
    order = np.argsort(dimensions)[::-1]
    return center.astype(np.float32), dimensions[order].astype(np.float32), vh[order].astype(np.float32)


def optical_bearing_deg(position_camera_m: tuple[float, float, float] | np.ndarray) -> float:
    x, _, z = position_camera_m
    return math.degrees(math.atan2(float(x), float(z)))
