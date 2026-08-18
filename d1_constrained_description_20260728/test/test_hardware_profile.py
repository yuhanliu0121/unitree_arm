from pathlib import Path

from d1_constrained_description import (
    apply_joint_limit_overrides,
    joint_limits_from_urdf,
    load_joint_limit_overrides,
)


ROOT = Path(__file__).parents[1]
URDF = ROOT / "urdf" / "d1_description.urdf"
PROFILES = ROOT / "config" / "hardware_profiles.yaml"


def test_profile_requires_exact_explicit_serial():
    assert load_joint_limit_overrides(PROFILES, "") == {}
    assert load_joint_limit_overrides(PROFILES, "D1095-typo") == {}
    assert set(load_joint_limit_overrides(PROFILES, "D1095")) == {"Joint1", "Joint2"}


def test_d1095_adjusts_only_requested_bounds():
    base = URDF.read_text(encoding="utf-8")
    names = [f"Joint{index}" for index in range(7)]
    base_limits = joint_limits_from_urdf(base, names)
    adjusted = apply_joint_limit_overrides(
        base, load_joint_limit_overrides(PROFILES, "D1095")
    )
    adjusted_limits = joint_limits_from_urdf(adjusted, names)
    assert adjusted_limits[1][0] == -1.553343034275
    assert adjusted_limits[1][1] == base_limits[1][1]
    assert adjusted_limits[2][0] == base_limits[2][0]
    assert adjusted_limits[2][1] == 1.596976265574
    for index in (0, 3, 4, 5, 6):
        assert adjusted_limits[index] == base_limits[index]
