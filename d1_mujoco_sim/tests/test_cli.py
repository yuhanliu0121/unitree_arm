import argparse

import pytest

from d1_mujoco_sim.cli import _parse_arm_pose_rad


def test_parse_initial_arm_pose_rad() -> None:
    assert _parse_arm_pose_rad("[0, -1.54, 1.55, 0, -0.6, 0]") == (
        0.0,
        -1.54,
        1.55,
        0.0,
        -0.6,
        0.0,
    )


@pytest.mark.parametrize(
    "value",
    (
        "[0, 1]",
        "not-a-list",
        '[0, 1, 2, 3, 4, "five"]',
        "[0, 1, 2, 3, 4, NaN]",
    ),
)
def test_parse_initial_arm_pose_rad_rejects_invalid_values(value: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError):
        _parse_arm_pose_rad(value)
