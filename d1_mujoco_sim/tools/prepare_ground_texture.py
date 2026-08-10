#!/usr/bin/env python3
"""Rectify the photographed floor patch and turn it into a repeatable texture."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image


# Centre lines of the hand-drawn square in objects/ground_textrue.png, ordered
# top-left, top-right, bottom-right, bottom-left.  Keeping these calibration
# points explicit makes regenerating the asset deterministic.
FRAME_CORNERS = np.array(
    [
        [149.0, 292.0],
        [612.0, 287.0],
        [650.0, 767.0],
        [134.0, 782.0],
    ],
    dtype=np.float64,
)


def _inset_corners(corners: np.ndarray, inset_px: float) -> np.ndarray:
    """Move each corner inward far enough to exclude the black marker."""
    centre = corners.mean(axis=0)
    directions = centre - corners
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    return corners + directions * inset_px


def _perspective_coefficients(
    output_points: np.ndarray,
    source_points: np.ndarray,
) -> tuple[float, ...]:
    """Return Pillow's output-to-source perspective coefficients."""
    rows: list[list[float]] = []
    values: list[float] = []
    for (x, y), (u, v) in zip(output_points, source_points, strict=True):
        rows.append([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y])
        values.append(u)
        rows.append([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y])
        values.append(v)
    return tuple(np.linalg.solve(np.asarray(rows), np.asarray(values)))


def _make_repeatable(image: Image.Image) -> Image.Image:
    """Mirror four half-size patches so all repeated edges match exactly."""
    half_size = (image.width // 2, image.height // 2)
    patch = np.asarray(image.resize(half_size, Image.Resampling.LANCZOS))
    top = np.concatenate((patch, patch[:, ::-1]), axis=1)
    return Image.fromarray(np.concatenate((top, top[::-1]), axis=0))


def prepare_texture(
    source_path: Path,
    output_path: Path,
    *,
    size_px: int = 1024,
    marker_inset_px: float = 10.0,
) -> None:
    source = Image.open(source_path).convert("RGB")
    source_points = _inset_corners(FRAME_CORNERS, marker_inset_px)
    last = float(size_px - 1)
    output_points = np.array(
        [[0.0, 0.0], [last, 0.0], [last, last], [0.0, last]],
        dtype=np.float64,
    )
    coefficients = _perspective_coefficients(output_points, source_points)
    rectified = source.transform(
        (size_px, size_px),
        Image.Transform.PERSPECTIVE,
        coefficients,
        resample=Image.Resampling.BICUBIC,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    _make_repeatable(rectified).save(output_path, optimize=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--size", type=int, default=1024)
    parser.add_argument("--marker-inset", type=float, default=10.0)
    args = parser.parse_args()
    prepare_texture(
        args.source,
        args.output,
        size_px=args.size,
        marker_inset_px=args.marker_inset,
    )


if __name__ == "__main__":
    main()
