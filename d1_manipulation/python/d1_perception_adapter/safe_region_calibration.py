"""Geometry helpers for the four-boundary cube safe-descend calibration."""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np


BOUNDARY_NAMES = (
    "left_lower",
    "left_upper",
    "right_lower",
    "right_upper",
)

SLAB_BOUNDARY_NAMES = (
    "closing_side_a",
    "closing_side_b",
)
LEGACY_SLAB_BOUNDARY_NAMES = ("closing_negative", "closing_positive")


def _unit(vector: Sequence[float], name: str) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float64)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must be a finite 3-vector")
    norm = np.linalg.norm(value)
    if norm < 1e-9:
        raise ValueError(f"{name} has near-zero length")
    return value / norm


def _aligned_mean(vectors: Sequence[Sequence[float]], name: str) -> np.ndarray:
    if not vectors:
        raise ValueError(f"no {name} samples")
    reference = _unit(vectors[0], name)
    aligned = []
    for vector in vectors:
        value = _unit(vector, name)
        if np.dot(value, reference) < 0.0:
            value = -value
        aligned.append(value)
    return _unit(np.mean(aligned, axis=0), f"mean {name}")


def _angle_degrees(first: Sequence[float], second: Sequence[float]) -> float:
    a = _unit(first, "first angle vector")
    b = _unit(second, "second angle vector")
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def build_safe_region(
    samples: Mapping[str, Mapping[str, Sequence[float]]],
    closing_margin_m: float = 0.005,
    finger_margin_m: float = 0.005,
) -> dict:
    """Build a gravity-extruded rectangular region from four boundary samples.

    The samples are the four physically validated corner cases: left/right
    zero closing clearance crossed with lower/upper half-finger engagement.
    Opposite measurements are averaged before forming each scalar bound so a
    single noisy corner does not define the entire rectangle.
    """
    missing = [name for name in BOUNDARY_NAMES if name not in samples]
    if missing:
        raise ValueError(f"missing boundary samples: {', '.join(missing)}")
    if closing_margin_m < 0.0 or finger_margin_m < 0.0:
        raise ValueError("safe-region margins cannot be negative")

    ordered = [samples[name] for name in BOUNDARY_NAMES]
    points = np.asarray(
        [
            sample.get("reference_point_camera_m", sample.get("top_center_camera_m"))
            for sample in ordered
        ],
        dtype=np.float64,
    )
    if points.shape != (4, 3) or not np.all(np.isfinite(points)):
        raise ValueError("top centres must contain four finite 3-vectors")

    extrusion = _aligned_mean(
        [sample["gravity_up_camera"] for sample in ordered], "gravity axis"
    )
    closing_hint = _aligned_mean(
        [sample["closing_axis_camera"] for sample in ordered], "closing axis"
    )
    closing = closing_hint - np.dot(closing_hint, extrusion) * extrusion
    closing = _unit(closing, "horizontal closing axis")

    finger_hint = _aligned_mean(
        [sample["finger_axis_camera"] for sample in ordered], "finger axis"
    )
    finger = finger_hint - np.dot(finger_hint, extrusion) * extrusion
    finger -= np.dot(finger, closing) * closing
    finger = _unit(finger, "horizontal finger axis")
    if np.dot(finger, finger_hint) < 0.0:
        finger = -finger

    closing_values = points @ closing
    finger_values = points @ finger
    raw_closing = sorted((
        float(np.mean(closing_values[[0, 1]])),
        float(np.mean(closing_values[[2, 3]])),
    ))
    raw_finger = sorted((
        float(np.mean(finger_values[[0, 2]])),
        float(np.mean(finger_values[[1, 3]])),
    ))
    safe_closing = [
        raw_closing[0] + closing_margin_m,
        raw_closing[1] - closing_margin_m,
    ]
    safe_finger = [
        raw_finger[0] + finger_margin_m,
        raw_finger[1] - finger_margin_m,
    ]
    if safe_closing[0] >= safe_closing[1]:
        raise ValueError("closing margin consumes the measured closing span")
    if safe_finger[0] >= safe_finger[1]:
        raise ValueError("finger margin consumes the measured finger span")

    closing_mid = 0.5 * sum(safe_closing)
    finger_mid = 0.5 * sum(safe_finger)
    # The component along ``extrusion`` is intentionally zero: an infinite
    # line is represented by the closest point on that line to camera origin.
    centre_line_anchor = closing_mid * closing + finger_mid * finger

    joint_arrays = [
        np.asarray(sample.get("arm_joint_positions_rad", []), dtype=np.float64)
        for sample in ordered
    ]
    comparable_joints = (
        joint_arrays[0].size > 0
        and all(array.shape == joint_arrays[0].shape for array in joint_arrays)
    )
    joint_spread = (
        np.ptp(np.stack(joint_arrays), axis=0) if comparable_joints else np.array([])
    )
    gravity_spread = max(
        _angle_degrees(extrusion, sample["gravity_up_camera"])
        for sample in ordered
    )

    return {
        "schema_version": 1,
        "frame_id": ordered[0].get("camera_frame", "wrist_camera_color_optical_frame"),
        "interpretation": (
            "rectangular cross-section extruded along gravity; bounds are scalar "
            "dot products in the stated camera-frame basis"
        ),
        "basis_camera": {
            "closing_axis": closing.tolist(),
            "finger_length_axis": finger.tolist(),
            "extrusion_axis_gravity_up": extrusion.tolist(),
        },
        "raw_bounds_m": {
            "closing": raw_closing,
            "finger_length": raw_finger,
        },
        "safe_margins_m": {
            "closing": float(closing_margin_m),
            "finger_length": float(finger_margin_m),
        },
        "safe_bounds_m": {
            "closing": safe_closing,
            "finger_length": safe_finger,
        },
        "safe_centre_line": {
            "anchor_camera_m": centre_line_anchor.tolist(),
            "direction_camera": extrusion.tolist(),
        },
        "diagnostics": {
            "raw_closing_width_m": raw_closing[1] - raw_closing[0],
            "raw_finger_width_m": raw_finger[1] - raw_finger[0],
            "maximum_gravity_axis_deviation_deg": gravity_spread,
            "maximum_arm_joint_spread_deg": (
                float(np.degrees(np.max(joint_spread)))
                if joint_spread.size else None
            ),
            "paired_boundary_repeatability_m": {
                "left_closing_difference": float(abs(closing_values[0] - closing_values[1])),
                "right_closing_difference": float(abs(closing_values[2] - closing_values[3])),
                "lower_finger_difference": float(abs(finger_values[0] - finger_values[2])),
                "upper_finger_difference": float(abs(finger_values[1] - finger_values[3])),
            },
            "all_point_coordinates": {
                name: {
                    "closing_m": float(closing_values[index]),
                    "finger_length_m": float(finger_values[index]),
                    "gravity_m": float(np.dot(points[index], extrusion)),
                }
                for index, name in enumerate(BOUNDARY_NAMES)
            },
        },
    }


def build_safe_slab(
    samples: Mapping[str, Mapping[str, Sequence[float]]],
    closing_margin_m: float = 0.0,
) -> dict:
    """Build a gravity/finger-extruded slab from two closing boundaries."""
    names = SLAB_BOUNDARY_NAMES
    if not all(name in samples for name in names) and all(
        name in samples for name in LEGACY_SLAB_BOUNDARY_NAMES
    ):
        names = LEGACY_SLAB_BOUNDARY_NAMES
    missing = [name for name in names if name not in samples]
    if missing:
        raise ValueError(f"missing slab boundary samples: {', '.join(missing)}")
    if closing_margin_m < 0.0:
        raise ValueError("safe-slab margin cannot be negative")

    ordered = [samples[name] for name in names]
    points = np.asarray(
        [
            sample.get("reference_point_camera_m", sample.get("top_center_camera_m"))
            for sample in ordered
        ],
        dtype=np.float64,
    )
    if points.shape != (2, 3) or not np.all(np.isfinite(points)):
        raise ValueError("reference points must contain two finite 3-vectors")

    extrusion = _aligned_mean(
        [sample["gravity_up_camera"] for sample in ordered], "gravity axis"
    )
    closing_hint = _aligned_mean(
        [sample["closing_axis_camera"] for sample in ordered], "closing axis"
    )
    closing = closing_hint - np.dot(closing_hint, extrusion) * extrusion
    closing = _unit(closing, "horizontal closing axis")
    values = points @ closing
    raw_bounds = sorted(map(float, values))
    safe_bounds = [
        raw_bounds[0] + closing_margin_m,
        raw_bounds[1] - closing_margin_m,
    ]
    if safe_bounds[0] >= safe_bounds[1]:
        raise ValueError("closing margin consumes the measured slab span")
    midpoint = 0.5 * sum(safe_bounds)

    return {
        "schema_version": 1,
        "region_type": "closing_interval_extruded_along_finger_and_gravity",
        "frame_id": ordered[0].get(
            "camera_frame", "wrist_camera_color_optical_frame"
        ),
        "interpretation": (
            "one-dimensional closing-axis interval; unconstrained along "
            "finger length and gravity"
        ),
        "basis_camera": {
            "closing_axis": closing.tolist(),
            "extrusion_axis_gravity_up": extrusion.tolist(),
        },
        "raw_bounds_m": {"closing": raw_bounds},
        "safe_margins_m": {"closing": float(closing_margin_m)},
        "safe_bounds_m": {"closing": safe_bounds},
        "safe_centre_plane": {
            "normal_camera": closing.tolist(),
            "coordinate_m": midpoint,
        },
        "diagnostics": {
            "raw_closing_width_m": raw_bounds[1] - raw_bounds[0],
            "boundary_coordinates_m": {
                name: float(values[index])
                for index, name in enumerate(names)
            },
            "gravity_axis_deviation_deg": _angle_degrees(
                ordered[0]["gravity_up_camera"], ordered[1]["gravity_up_camera"]
            ),
        },
    }
