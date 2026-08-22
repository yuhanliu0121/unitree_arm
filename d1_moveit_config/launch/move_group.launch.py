from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
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
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    config.robot_description = {
        "robot_description": load_planning_description(description, arm_serial)
    }
    return config


def _launch_setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    requested_arm_serial = LaunchConfiguration("arm_serial").perform(context).strip()
    arm_serial = requested_arm_serial if backend == "real" else ""
    moveit_config = build_moveit_config(arm_serial)
    move_group_parameters = moveit_config.to_dict()
    if backend == "real":
        # The D1 mode=1 endpoint controller may need several bounded retries
        # before the firmware begins physical motion.  Keep MoveIt's outer
        # execution watchdog wider than that feedback-verified retry window.
        move_group_parameters["trajectory_execution"][
            "allowed_goal_duration_margin"
        ] = 12.0
    control_launch = Path(
        get_package_share_directory("d1_ros2_control")
    ) / "launch" / "control.launch.py"
    rviz_profile = "real.rviz" if backend == "real" else "moveit.rviz"
    rviz_config = Path(
        get_package_share_directory("d1_moveit_config")
    ) / "config" / rviz_profile
    depth_visualizer_config = Path(
        get_package_share_directory("d1_camera_visualization")
    ) / "config" / "depth_debug.yaml"

    return [
            # Keep this name distinct from control.launch.py's own `rviz`
            # argument. The included control stack is always headless here;
            # this launch owns the single, MoveIt-aware RViz instance.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(control_launch)),
                launch_arguments={
                    "rviz": "false",
                    "backend": backend,
                    "arm_serial": arm_serial,
                    "dds_domain_id": LaunchConfiguration("dds_domain_id"),
                    "interface": LaunchConfiguration("interface"),
                    "command_port": LaunchConfiguration("command_port"),
                    "feedback_port": LaunchConfiguration("feedback_port"),
                    "command_topic": LaunchConfiguration("command_topic"),
                    "feedback_topic": LaunchConfiguration("feedback_topic"),
                    "status_topic": LaunchConfiguration("status_topic"),
                    "gripper_closed_angle_deg": LaunchConfiguration("gripper_closed_angle_deg"),
                    "gripper_open_angle_deg": LaunchConfiguration("gripper_open_angle_deg"),
                    "gripper_travel_m": LaunchConfiguration("gripper_travel_m"),
                }.items(),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[move_group_parameters],
            ),
            Node(
                package="d1_camera_visualization",
                executable="depth_debug_visualizer",
                output="screen",
                parameters=[str(depth_visualizer_config)],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
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
            DeclareLaunchArgument("dds_domain_id", default_value="42"),
            DeclareLaunchArgument("interface", default_value=""),
            DeclareLaunchArgument("command_port", default_value="15000"),
            DeclareLaunchArgument("feedback_port", default_value="15001"),
            DeclareLaunchArgument("command_topic", default_value="rt/arm_Command"),
            DeclareLaunchArgument("feedback_topic", default_value="current_servo_angle"),
            DeclareLaunchArgument("status_topic", default_value="rt/arm_Feedback"),
            DeclareLaunchArgument("gripper_closed_angle_deg", default_value="-30.0"),
            DeclareLaunchArgument("gripper_open_angle_deg", default_value="60.0"),
            DeclareLaunchArgument("gripper_travel_m", default_value="0.03"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
