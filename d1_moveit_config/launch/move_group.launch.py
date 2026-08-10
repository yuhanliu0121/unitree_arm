from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    config.robot_description = {
        "robot_description": load_planning_description(description)
    }
    return config


def generate_launch_description():
    moveit_config = build_moveit_config()
    control_launch = Path(
        get_package_share_directory("d1_ros2_control")
    ) / "launch" / "control.launch.py"
    rviz_config = Path(
        get_package_share_directory("d1_moveit_config")
    ) / "config" / "moveit.rviz"

    return LaunchDescription(
        [
            # Keep this name distinct from control.launch.py's own `rviz`
            # argument. The included control stack is always headless here;
            # this launch owns the single, MoveIt-aware RViz instance.
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(control_launch)),
                launch_arguments={"rviz": "false"}.items(),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict()],
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
    )
