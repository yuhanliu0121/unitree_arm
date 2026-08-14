from pathlib import Path

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
                "zucchini": (0.0, 3.0),
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
