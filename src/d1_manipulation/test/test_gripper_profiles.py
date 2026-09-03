from pathlib import Path

import numpy as np
import pytest
import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"
SOURCE = Path(__file__).resolve().parents[1] / "src"


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
                "zucchini": (10.0, 13.0),
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
        0.006865495787693202, abs=1e-12
    )
    assert finger_bounds[1] - finger_bounds[0] == pytest.approx(
        0.04062753979416833, abs=1e-12
    )
    assert parameters["cube_finetune_max_step_m"] == pytest.approx(0.008)
    assert parameters["cube_finetune_max_total_m"] == pytest.approx(0.020)
    assert parameters["cube_finetune_gain"] == pytest.approx(0.4)
    assert parameters["cube_finetune_max_corrections"] == 5
    assert parameters["cube_finetune_max_stddev_m"] == pytest.approx(0.002)
    assert parameters["finetune_motion_speed_deg_s"] == pytest.approx(5.0)
    assert parameters["finetune_execution_guard_margin_rad"] == pytest.approx(
        np.deg2rad(4.0), abs=1e-9
    )
    assert parameters["finetune_completion_stable_range_rad"] == pytest.approx(
        np.deg2rad(0.3), abs=1e-9
    )
    assert parameters["finetune_completion_stable_samples"] == 5
    assert parameters["finetune_completion_sample_period_s"] == pytest.approx(0.12)
    assert "cube_finetune_min_improvement_ratio" not in parameters

    observe_parameters = common["d1_observe_target"]["ros__parameters"]
    assert observe_parameters["max_target_pixel_error"] == pytest.approx(50.0)


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
    assert parameters["zucchini_finetune_gain"] == pytest.approx(0.4)
    assert parameters["zucchini_finetune_max_corrections"] == 5
    assert parameters["zucchini_finetune_max_stddev_m"] == pytest.approx(0.002)
    assert "zucchini_finetune_target_tolerance_m" not in parameters
    assert "zucchini_finetune_min_improvement_ratio" not in parameters


@pytest.mark.parametrize(
    "strategy_source",
    ["yellow_cube_pick_strategy.cpp", "zucchini_pick_strategy.cpp"],
)
def test_finetune_preserves_canonical_pregrasp_orientation(strategy_source):
    source = (SOURCE / strategy_source).read_text()
    assert (
        "const auto pregrasp_orientation = plan.pregrasp_pose.orientation;"
        in source
    )
    assert "target.orientation = pregrasp_orientation;" in source
    assert "current_pose.orientation = plan.pregrasp_pose.orientation;" in source


def test_drop_search_expands_from_nominal_height_and_y():
    common = yaml.safe_load((CONFIG / "observe_target.yaml").read_text())
    parameters = common["d1_drop_object"]["ros__parameters"]
    assert parameters["start_state_bounds_tolerance_rad"] == pytest.approx(0.1)
    assert parameters["go2_platform_collision_id"] == "go2_platform"
    assert parameters["drop_keepout_margin_m"] == pytest.approx(0.05)
    assert parameters["base_scene_wait_timeout_s"] == pytest.approx(5.0)
    assert parameters["height_offsets_m"] == pytest.approx(
        [
            0.0, -0.010, 0.010, -0.020, 0.020, -0.030, 0.030,
            -0.040, 0.040, -0.050, 0.050, -0.060, 0.060,
            -0.080, 0.080,
        ]
    )
    assert parameters["y_offsets_m"] == pytest.approx(
        [
            0.0, 0.003, -0.003, 0.006, -0.006, 0.009,
            -0.009, 0.012, -0.012, 0.015, -0.015,
        ]
    )


def test_drop_requires_round_trip_and_exposes_new_target_result():
    source = (SOURCE / "drop_object_server.cpp").read_text()
    action = (
        Path(__file__).resolve().parents[2]
        / "d1_interfaces"
        / "action"
        / "DropObject.action"
    ).read_text()

    assert "FAILURE_TARGET_IN_KEEP_OUT=5" in action
    assert "OUTCOME_NEW_TARGET_REQUIRED=3" in action
    assert "getAttachedObjects({go2_platform_collision_id_})" in source
    assert "planOpenEmptyReturn(plan, held_ids" in source
    assert "ROUND_TRIP_SUCCEEDED" in source
    assert "executePlan(return_plan)" in source
    assert "drop_start_payload_.store(transition.payload_state)" in source
    assert "task payload and MoveIt held-object state disagree" in source
    assert "RECOVERING_TO_STOWED" in source
    assert "RECOVERING_TO_CARRY" in source


def test_base_planning_scene_is_initialized_at_stack_start():
    common = yaml.safe_load((CONFIG / "observe_target.yaml").read_text())
    parameters = common["d1_base_planning_scene"]["ros__parameters"]
    launch = (SOURCE.parent / "launch" / "observe_target.launch.py").read_text()
    observer = (SOURCE / "observe_target_server.cpp").read_text()

    assert parameters["ground_collision_id"] == "ground"
    assert parameters["go2_platform_collision_id"] == "go2_platform"
    assert parameters["go2_platform_dimensions_m"] == pytest.approx(
        [0.753442, 0.338254, 0.255121]
    )
    assert 'executable="base_planning_scene_initializer"' in launch
    assert "failed to attach Go2 planning collision box" not in observer


def test_pick_prevalidates_loaded_lift_and_carry_before_descending():
    source = (SOURCE / "pick_object_server.cpp").read_text()

    precheck = source.index('feedback(handle, "PRECHECK_ESCAPE"')
    descend = source.index('feedback(handle, "DESCEND"', precheck)
    grasp = source.index('feedback(handle, "GRASP"', descend)
    attach = source.index('feedback(handle, "ATTACH_OBJECT"', grasp)
    lift = source.index('feedback(handle, "LIFT"', attach)
    carry = source.index('feedback(handle, "CARRY"', lift)

    assert precheck < descend < grasp < attach < lift < carry
    assert "validateTrajectoryWithAttachedObject" in source
    assert "hypothetical loaded CARRY state" in source
    assert "no collision-free loaded LIFT-to-CARRY route was found" in source
    assert "executePlan(escape_plan->carry_plan)" in source
    assert "PICK FAILED: category=%u failed_state=%s detail=%s" in source


def test_pick_state_transition_occurs_only_after_action_execution_starts():
    source = (SOURCE / "pick_object_server.cpp").read_text()

    goal_callback = source[
        source.index("server_ = rclcpp_action::create_server<Pick>("):
        source.index("debug_service_callback_group_", source.index(
            "server_ = rclcpp_action::create_server<Pick>("
        ))
    ]
    execute = source[source.index("void execute("):]

    assert "START_PICK" not in goal_callback
    assert execute.index("START_PICK") < execute.index(
        'feedback(handle, "ENSURE_STOWED"'
    )
    assert "abortBeforeTaskStart(handle, transition)" in execute
    assert "pick_object could not start:" in source


def test_task_state_manager_logs_each_fault_once_with_diagnostics():
    source = (SOURCE / "arm_task_state_manager.cpp").read_text()

    assert "snapshot.state == ArmTaskState::FAULTED && !fault_logged_" in source
    assert "***** ARM ENTERED FAULTED *****" in source
    assert "failure_code=%s detail=%s" in source
    assert "active_operation=%s active_phase=%s" in source
    assert "bool fault_logged_{false};" in source
