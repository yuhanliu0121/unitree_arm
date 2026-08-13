from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from d1_moveit_config.planning_description import load_planning_description


def build_moveit_config():
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
        "robot_description": load_planning_description(description)
    }
    return config


def generate_launch_description():
    moveit_config = build_moveit_config()
    move_group_launch = Path(
        get_package_share_directory("d1_moveit_config")
    ) / "launch" / "move_group.launch.py"
    observe_config = Path(
        get_package_share_directory("d1_manipulation")
    ) / "config" / "observe_target.yaml"
    perception_config = Path(
        get_package_share_directory("d1_manipulation")
    ) / "config" / "perception_adapter.yaml"
    workspace_root = Path(get_package_prefix("d1_manipulation")).parents[1]

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(move_group_launch)),
                launch_arguments={
                    "launch_rviz": LaunchConfiguration("launch_rviz")
                }.items(),
            ),
            Node(
                package="d1_manipulation",
                executable="observe_target_server",
                output="screen",
                parameters=[moveit_config.to_dict(), str(observe_config)],
            ),
            Node(
                package="d1_manipulation",
                executable="pick_object_server",
                output="screen",
                parameters=[moveit_config.to_dict(), str(observe_config)],
            ),
            Node(
                package="d1_manipulation",
                executable="drop_object_server",
                output="screen",
                parameters=[moveit_config.to_dict(), str(observe_config)],
            ),
            Node(
                package="d1_manipulation",
                executable="detect_target_server",
                output="screen",
                parameters=[
                    str(perception_config),
                    {
                        "perception_runtime_root": str(
                            workspace_root / "perception_runtime_v1"
                        )
                    },
                ],
            ),
        ]
    )
