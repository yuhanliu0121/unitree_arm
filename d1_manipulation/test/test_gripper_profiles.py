from pathlib import Path

import numpy as np
import pytest
import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"


def _degrees(position_m):
    return -30.0 + position_m / 0.03 * 90.0


@pytest.mark.parametrize(
    "profile,expected",
    [
        (
            "simulation",
            {
                "cube": (-30.0, 20.0),
                "zucchini": (-30.0, -10.0),
                "bowl": (-30.0, -28.0),
            },
        ),
        (
            "real",
            {
                "cube": (35.0, 37.5),
                "zucchini": (-10.0, 0.0),
                "bowl": (-30.0, -28.0),
            },
        ),
    ],
)
def test_gripper_profile_angles(profile, expected):
    document = yaml.safe_load((CONFIG / f"gripper_{profile}.yaml").read_text())
    parameters = document["d1_pick_object"]["ros__parameters"]
    for object_name, (target_deg, threshold_deg) in expected.items():
        target = parameters[f"{object_name}_gripper_closed_m"]
        threshold = parameters[f"{object_name}_gripper_held_threshold_m"]
        assert _degrees(target) == pytest.approx(target_deg, abs=1e-6)
        assert _degrees(threshold) == pytest.approx(threshold_deg, abs=1e-6)
        assert threshold > target


def test_cube_finetune_is_default_and_uses_calibrated_orthogonal_axes():
    common = yaml.safe_load((CONFIG / "observe_target.yaml").read_text())
    parameters = common["d1_pick_object"]["ros__parameters"]
    assert parameters["cube_finetune_enabled"] is True
    assert parameters["cube_finetune_camera_frame"] == \
        "wrist_camera_color_optical_frame"
    closing = np.asarray(parameters["cube_finetune_closing_axis_camera"])
    finger = np.asarray(parameters["cube_finetune_finger_axis_camera"])
    assert np.linalg.norm(closing) == pytest.approx(1.0, abs=1e-6)
    assert np.linalg.norm(finger) == pytest.approx(1.0, abs=1e-6)
    assert float(closing @ finger) == pytest.approx(0.0, abs=1e-6)
    closing_bounds = parameters["cube_finetune_closing_bounds_m"]
    finger_bounds = parameters["cube_finetune_finger_bounds_m"]
    assert closing_bounds[1] - closing_bounds[0] == pytest.approx(
        0.004115908190508388, abs=1e-12
    )
    assert finger_bounds[1] - finger_bounds[0] == pytest.approx(
        0.04062753979416833, abs=1e-12
    )
    assert parameters["cube_finetune_max_step_m"] == pytest.approx(0.008)
    assert parameters["cube_finetune_max_total_m"] == pytest.approx(0.020)
    assert parameters["cube_finetune_max_corrections"] == 3


def test_zucchini_pregrasp_uses_explicit_ground_clearance_search():
    common = yaml.safe_load((CONFIG / "observe_target.yaml").read_text())
    parameters = common["d1_pick_object"]["ros__parameters"]
    assert parameters["zucchini_pregrasp_ground_clearance_max_m"] == \
        pytest.approx(0.090)
    assert parameters["zucchini_pregrasp_ground_clearance_min_m"] == \
        pytest.approx(0.065)
    assert parameters["zucchini_pregrasp_ground_clearance_step_m"] == \
        pytest.approx(0.005)
    assert "zucchini_pregrasp_distance_max_m" not in parameters


def test_zucchini_finetune_uses_physically_validated_closing_slab():
    common = yaml.safe_load((CONFIG / "observe_target.yaml").read_text())
    parameters = common["d1_pick_object"]["ros__parameters"]
    assert parameters["zucchini_finetune_enabled"] is True
    assert parameters["zucchini_finetune_camera_frame"] == \
        "wrist_camera_color_optical_frame"
    closing = np.asarray(parameters["zucchini_finetune_closing_axis_camera"])
    gravity = np.asarray(parameters["zucchini_finetune_gravity_axis_camera"])
    assert np.linalg.norm(closing) == pytest.approx(1.0, abs=1e-6)
    assert np.linalg.norm(gravity) == pytest.approx(1.0, abs=1e-6)
    assert float(closing @ gravity) == pytest.approx(0.0, abs=1e-6)
    bounds = parameters["zucchini_finetune_closing_bounds_m"]
    assert bounds[1] - bounds[0] == pytest.approx(
        0.017038788840476115, abs=1e-12
    )
    assert "zucchini_finetune_target_tolerance_m" not in parameters
    assert parameters["zucchini_finetune_max_corrections"] == 3
