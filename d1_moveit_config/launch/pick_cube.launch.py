from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
    return LaunchDescription(
        [
            Node(
                package="d1_grasp_demo",
                executable="pick_cube",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "scene_state_port": 15002,
                        "pregrasp_clearance_m": 0.12,
                        "lift_distance_m": 0.10,
                        "cartesian_step_m": 0.005,
                        "minimum_cartesian_fraction": 0.95,
                        "gripper_open_m": 0.03,
                        "gripper_closed_m": 0.0,
                        "grasp_settle_s": 0.5,
                        "hold_s": 12.0,
                        # Keep DDS-following error below the controller's
                        # 0.15 rad path tolerance even with both GUIs active.
                        "velocity_scaling": 0.10,
                        "acceleration_scaling": 0.10,
                    },
                ],
            )
        ]
    )
