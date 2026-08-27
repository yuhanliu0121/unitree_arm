from pathlib import Path
import sys

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from d1_perception_adapter import build_safe_region, build_safe_slab  # noqa: E402


def _sample(point):
    return {
        "camera_frame": "camera",
        "top_center_camera_m": point,
        "gravity_up_camera": [0.0, 0.0, 1.0],
        "closing_axis_camera": [1.0, 0.0, 0.0],
        "finger_axis_camera": [0.0, 1.0, 0.0],
        "arm_joint_positions_rad": [0.0] * 6,
    }


def test_four_boundary_samples_form_shrunken_extruded_rectangle() -> None:
    region = build_safe_region(
        {
            "left_lower": _sample([-0.010, -0.030, 0.25]),
            "left_upper": _sample([-0.010, 0.040, 0.27]),
            "right_lower": _sample([0.020, -0.030, 0.26]),
            "right_upper": _sample([0.020, 0.040, 0.24]),
        },
        closing_margin_m=0.005,
        finger_margin_m=0.010,
    )
    np.testing.assert_allclose(region["raw_bounds_m"]["closing"], [-0.01, 0.02])
    np.testing.assert_allclose(region["raw_bounds_m"]["finger_length"], [-0.03, 0.04])
    np.testing.assert_allclose(region["safe_bounds_m"]["closing"], [-0.005, 0.015])
    np.testing.assert_allclose(region["safe_bounds_m"]["finger_length"], [-0.02, 0.03])
    np.testing.assert_allclose(
        region["safe_centre_line"]["anchor_camera_m"], [0.005, 0.005, 0.0]
    )


def test_axis_signs_are_aligned_before_averaging() -> None:
    samples = {
        "left_lower": _sample([-0.02, -0.02, 0.2]),
        "left_upper": _sample([-0.02, 0.02, 0.2]),
        "right_lower": _sample([0.02, -0.02, 0.2]),
        "right_upper": _sample([0.02, 0.02, 0.2]),
    }
    samples["left_upper"]["closing_axis_camera"] = [-1.0, 0.0, 0.0]
    samples["right_upper"]["finger_axis_camera"] = [0.0, -1.0, 0.0]
    region = build_safe_region(samples, 0.001, 0.001)
    np.testing.assert_allclose(region["basis_camera"]["closing_axis"], [1, 0, 0])
    np.testing.assert_allclose(region["basis_camera"]["finger_length_axis"], [0, 1, 0])


def test_generic_reference_point_key_is_supported() -> None:
    samples = {
        "left_lower": _sample([-0.04, -0.03, 0.2]),
        "left_upper": _sample([-0.04, 0.03, 0.2]),
        "right_lower": _sample([0.04, -0.03, 0.2]),
        "right_upper": _sample([0.04, 0.03, 0.2]),
    }
    for sample in samples.values():
        sample["reference_point_camera_m"] = sample.pop("top_center_camera_m")
    region = build_safe_region(samples, closing_margin_m=0.0, finger_margin_m=0.0)
    np.testing.assert_allclose(region["safe_bounds_m"]["closing"], [-0.04, 0.04])
    np.testing.assert_allclose(
        region["safe_bounds_m"]["finger_length"], [-0.03, 0.03]
    )


def test_two_boundaries_form_closing_slab() -> None:
    negative = _sample([-0.025, -0.10, 0.20])
    positive = _sample([0.035, 0.12, 0.24])
    negative["reference_point_camera_m"] = negative.pop("top_center_camera_m")
    positive["reference_point_camera_m"] = positive.pop("top_center_camera_m")
    region = build_safe_slab(
        {"closing_side_a": negative, "closing_side_b": positive},
        closing_margin_m=0.005,
    )
    np.testing.assert_allclose(region["safe_bounds_m"]["closing"], [-0.02, 0.03])
    assert region["region_type"] == \
        "closing_interval_extruded_along_finger_and_gravity"


def test_legacy_signed_slab_names_remain_readable() -> None:
    first = _sample([-0.025, 0.0, 0.20])
    second = _sample([0.035, 0.0, 0.24])
    region = build_safe_slab(
        {"closing_negative": first, "closing_positive": second},
        closing_margin_m=0.0,
    )
    np.testing.assert_allclose(region["safe_bounds_m"]["closing"], [-0.025, 0.035])
