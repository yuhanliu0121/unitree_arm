import importlib.util
from pathlib import Path


MODULE_PATH = (
    Path(__file__).parents[1] / "scripts" / "d1_joint6_bypass_proxy.py"
)
SPEC = importlib.util.spec_from_file_location("joint6_bypass_proxy", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_latches_stable_median_and_rewrites_physical_target():
    state = MODULE.Joint6BypassState(sample_count=5, stability_deg=0.3)
    for value in (10.0, 10.1, 9.9, 10.0, 10.1):
        feedback = state.observe_feedback((0, 1, 2, 3, 4, 5, value))
    assert state.frozen_deg == 10.0
    assert feedback[6] == 10.0

    rewritten, simulated = state.rewrite_command((10, 11, 12, 13, 14, 15, -30))
    assert not simulated
    assert rewritten[6] == 10.0


def test_does_not_latch_unstable_feedback():
    state = MODULE.Joint6BypassState(sample_count=5, stability_deg=0.3)
    for value in (10.0, 10.5, 9.8, 10.4, 10.0):
        state.observe_feedback((0, 0, 0, 0, 0, 0, value))
    assert state.frozen_deg is None


def test_simulates_open_and_retained_close_without_changing_physical_target():
    state = MODULE.Joint6BypassState(sample_count=3, retention_margin_deg=4.0)
    for value in (10.0, 10.0, 10.0):
        state.observe_feedback((1, 2, 3, 4, 5, 6, value))

    rewritten, simulated = state.rewrite_command((1, 2, 3, 4, 5, 6, 60.0))
    assert simulated
    assert rewritten[6] == 10.0
    assert state.virtual_deg == 60.0

    rewritten, simulated = state.rewrite_command((1, 2, 3, 4, 5, 6, 35.0))
    assert simulated
    assert rewritten[6] == 10.0
    assert state.virtual_deg == 39.0

    rewritten, simulated = state.rewrite_command((1, 2, 3, 4, 5, 6, -30.0))
    assert simulated
    assert rewritten[6] == 10.0
    assert state.virtual_deg == -26.0


def test_arm_motion_preserves_virtual_retention_feedback():
    state = MODULE.Joint6BypassState(sample_count=3, retention_margin_deg=4.0)
    for value in (10.0, 10.0, 10.0):
        state.observe_feedback((0, 0, 0, 0, 0, 0, value))
    state.rewrite_command((0, 0, 0, 0, 0, 0, 35.0))
    assert state.virtual_deg == 39.0

    rewritten, simulated = state.rewrite_command((10, 10, 10, 10, 10, 10, 35.0))
    assert not simulated
    assert rewritten[6] == 10.0
    assert state.virtual_deg == 39.0
