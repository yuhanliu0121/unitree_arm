from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_top_observation_separates_planning_and_execution_failures():
    server = (PACKAGE / "src" / "pick_object_server.cpp").read_text()
    assert "no IK-valid top observation pose among" in server
    assert "but MoveIt found no collision-free plan" in server
    assert "top observation planning succeeded, but execution failed:" in server

    for strategy in (
        "yellow_cube_pick_strategy.cpp",
        "zucchini_pick_strategy.cpp",
        "bowl_pick_strategy.cpp",
    ):
        source = (PACKAGE / "src" / strategy).read_text()
        assert "moveCameraTopDown" in source
        assert "top observation pose is not plannable" not in source


def test_no_motion_diagnostic_reports_observations_without_root_cause_claim():
    controller = (
        PACKAGE.parent / "d1_ros2_control" / "src" / "d1_joint_segment_controller.cpp"
    ).read_text()
    assert "fresh joint feedback continued" in controller
    assert "target-directed progress" in controller
    assert "root cause undetermined" in controller
