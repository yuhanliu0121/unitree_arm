from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from d1_moveit_config.planning_description import load_planning_description


def build_moveit_config(arm_serial=""):
    description = Path(
        get_package_share_directory("d1_constrained_description")
    ) / "urdf" / "d1_description.urdf"
    config = (
        MoveItConfigsBuilder(
            "d1_constrained_description",
            package_name="d1_moveit_config",
        )
        .robot_description(file_path=str(description))
        .robot_description_semantic(file_path="config/d1.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config.robot_description = {
        "robot_description": load_planning_description(description, arm_serial)
    }
    return config


def _launch_setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    if backend not in {"simulation", "real"}:
        raise RuntimeError(f"unsupported backend: {backend}")
    requested_arm_serial = LaunchConfiguration("arm_serial").perform(context).strip()
    arm_serial = requested_arm_serial if backend == "real" else ""
    gravity_frame = LaunchConfiguration("gravity_frame").perform(context).strip()
    if not gravity_frame:
        raise RuntimeError("gravity_frame must not be empty")
    commissioning_text = LaunchConfiguration("enable_commissioning_api").perform(
        context
    ).strip().lower()
    rviz_text = LaunchConfiguration("launch_rviz").perform(context).strip().lower()
    if commissioning_text not in {"true", "false"} or rviz_text not in {"true", "false"}:
        raise RuntimeError("launch_rviz and enable_commissioning_api must be true or false")
    enable_commissioning_api = commissioning_text == "true"
    debug_text = LaunchConfiguration("enable_debug_outputs").perform(context).strip().lower()
    if debug_text and debug_text not in {"true", "false"}:
        raise RuntimeError("enable_debug_outputs must be empty, true, or false")
    enable_debug_outputs = (
        debug_text == "true" if debug_text else enable_commissioning_api or rviz_text == "true"
    )
    moveit_config = build_moveit_config(arm_serial)
    move_group_launch = Path(
        get_package_share_directory("d1_moveit_config")
    ) / "launch" / "move_group.launch.py"
    observe_config = Path(
        get_package_share_directory("d1_manipulation")
    ) / "config" / "observe_target.yaml"
    fixed_poses_config = Path(
        get_package_share_directory("d1_manipulation")
    ) / "config" / "fixed_poses.yaml"
    perception_config = Path(
        get_package_share_directory("d1_manipulation")
    ) / "config" / "perception_adapter.yaml"
    gripper_config = Path(
        get_package_share_directory("d1_manipulation")
    ) / "config" / f"gripper_{backend}.yaml"
    workspace_root = Path(get_package_prefix("d1_manipulation")).parents[1]
    runtime_root = workspace_root / "runtime" / "perception_runtime_v1"
    requested_model = LaunchConfiguration("perception_model_path").perform(context).strip()
    if requested_model:
        perception_model = Path(requested_model).expanduser()
        if not perception_model.is_absolute():
            perception_model = runtime_root / perception_model
    elif backend == "real":
        raise RuntimeError(
            "real backend requires an explicit perception_model_path from the "
            "deployment configuration"
        )
    else:
        perception_model = runtime_root / "weights" / "best.pt"
    if not perception_model.is_file():
        raise RuntimeError(f"perception model is unavailable: {perception_model}")
    return [
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(move_group_launch)),
                launch_arguments={
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "backend": backend,
                    "arm_serial": arm_serial,
                    "dds_domain_id": LaunchConfiguration("dds_domain_id"),
                    "interface": LaunchConfiguration("interface"),
                    "command_port": LaunchConfiguration("command_port"),
                    "feedback_port": LaunchConfiguration("feedback_port"),
                    "command_topic": LaunchConfiguration("command_topic"),
                    "native_segment_topic": LaunchConfiguration("native_segment_topic"),
                    "native_joint_speed_deg_s": LaunchConfiguration(
                        "native_joint_speed_deg_s"
                    ),
                    "real_command_rate_hz": LaunchConfiguration("real_command_rate_hz"),
                    "real_command_duration_ms": LaunchConfiguration(
                        "real_command_duration_ms"
                    ),
                    "feedback_topic": LaunchConfiguration("feedback_topic"),
                    "status_topic": LaunchConfiguration("status_topic"),
                    "gripper_closed_angle_deg": LaunchConfiguration("gripper_closed_angle_deg"),
                    "gripper_open_angle_deg": LaunchConfiguration("gripper_open_angle_deg"),
                    "gripper_travel_m": LaunchConfiguration("gripper_travel_m"),
                    "joint6_bypass": LaunchConfiguration("joint6_bypass"),
                }.items(),
        ),
        Node(
                package="d1_manipulation",
                executable="arm_task_state_manager",
                output="screen",
                parameters=[
                    str(fixed_poses_config),
                ],
        ),
        Node(
                package="d1_manipulation",
                executable="observe_target_server",
                output="screen",
                parameters=[
                    moveit_config.to_dict(), str(fixed_poses_config),
                    str(observe_config), str(gripper_config),
                    {
                        "backend": backend,
                        "gravity_frame": gravity_frame,
                        "enable_commissioning_api": enable_commissioning_api,
                    },
                ],
        ),
        Node(
                package="d1_manipulation",
                executable="pick_object_server",
                output="screen",
                parameters=[
                    moveit_config.to_dict(), str(fixed_poses_config),
                    str(observe_config), str(gripper_config),
                    {
                        "backend": backend,
                        "gravity_frame": gravity_frame,
                    },
                ],
        ),
        Node(
                package="d1_manipulation",
                executable="drop_object_server",
                output="screen",
                parameters=[
                    moveit_config.to_dict(), str(fixed_poses_config),
                    str(observe_config), str(gripper_config),
                    {
                        "backend": backend,
                        "gravity_frame": gravity_frame,
                    },
                ],
        ),
        Node(
                package="d1_manipulation",
                executable="detect_target_server",
                output="screen",
                parameters=[
                    str(perception_config),
                    str(gripper_config),
                    {
                        "perception_runtime_root": str(runtime_root),
                        "model_path": str(perception_model),
                        "gravity_frame": gravity_frame,
                        "enable_debug_outputs": enable_debug_outputs,
                    },
                ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "backend",
                choices=["simulation", "real"],
                description="Required immutable execution backend",
            ),
            DeclareLaunchArgument(
                "arm_serial",
                default_value="",
                description="Explicit physical D1 serial for per-unit URDF adjustments",
            ),
            DeclareLaunchArgument("gravity_frame", default_value="world"),
            DeclareLaunchArgument(
                "enable_commissioning_api",
                default_value="false",
                description=(
                    "Enable partial pick stages and manual continuation services; "
                    "must remain false in production"
                ),
            ),
            DeclareLaunchArgument(
                "enable_debug_outputs",
                default_value="",
                description=(
                    "Enable overlay publishers and debug image files. Empty enables "
                    "them only for RViz or commissioning launches."
                ),
            ),
            DeclareLaunchArgument(
                "perception_model_path",
                default_value="",
                description=(
                    "Perception weight path. Required for real deployment; simulation "
                    "uses weights/best.pt when empty."
                ),
            ),
            DeclareLaunchArgument("dds_domain_id", default_value="42"),
            DeclareLaunchArgument("interface", default_value=""),
            DeclareLaunchArgument("command_port", default_value="15000"),
            DeclareLaunchArgument("feedback_port", default_value="15001"),
            DeclareLaunchArgument("command_topic", default_value="rt/arm_Command"),
            DeclareLaunchArgument(
                "native_segment_topic", default_value="d1_native_joint_segment"
            ),
            DeclareLaunchArgument("native_joint_speed_deg_s", default_value="15.0"),
            DeclareLaunchArgument("real_command_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("real_command_duration_ms", default_value="0"),
            DeclareLaunchArgument("feedback_topic", default_value="current_servo_angle"),
            DeclareLaunchArgument("status_topic", default_value="rt/arm_Feedback"),
            DeclareLaunchArgument("gripper_closed_angle_deg", default_value="-30.0"),
            DeclareLaunchArgument("gripper_open_angle_deg", default_value="60.0"),
            DeclareLaunchArgument("gripper_travel_m", default_value="0.03"),
            DeclareLaunchArgument("joint6_bypass", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
