from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config.robot_description = {
        "robot_description": load_planning_description(description)
    }
    return config


def generate_launch_description():
    moveit_config = build_moveit_config()
    output_csv = LaunchConfiguration("output_csv")
    object_type = LaunchConfiguration("object_type")
    x_min = LaunchConfiguration("x_min_m")
    x_max = LaunchConfiguration("x_max_m")
    y_min = LaunchConfiguration("y_min_m")
    y_max = LaunchConfiguration("y_max_m")
    xy_step = LaunchConfiguration("xy_step_m")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_csv",
                default_value="/tmp/d1_grasp_workspace/grasp_workspace.csv",
            ),
            DeclareLaunchArgument("object_type", default_value="yellow_cube"),
            DeclareLaunchArgument("x_min_m", default_value="-0.70"),
            DeclareLaunchArgument("x_max_m", default_value="0.70"),
            DeclareLaunchArgument("y_min_m", default_value="-0.70"),
            DeclareLaunchArgument("y_max_m", default_value="0.70"),
            DeclareLaunchArgument("xy_step_m", default_value="0.05"),
            Node(
                package="d1_workspace_analysis",
                executable="grasp_workspace_scan",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "output_csv": output_csv,
                        "object_type": object_type,
                        "x_min_m": ParameterValue(x_min, value_type=float),
                        "x_max_m": ParameterValue(x_max, value_type=float),
                        "y_min_m": ParameterValue(y_min, value_type=float),
                        "y_max_m": ParameterValue(y_max, value_type=float),
                        "xy_step_m": ParameterValue(xy_step, value_type=float),
                        "yaw_step_deg": 10.0,
                        "ground_surface_z_m": -0.225248769402,
                        "ik_timeout_s": 0.002,
                        "scan_planning_time_s": 0.20,
                    },
                ],
            ),
        ]
    )
