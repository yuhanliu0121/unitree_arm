from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("d1_constrained_description")
    )
    robot_description = (
        package_share / "urdf" / "d1_description.urdf"
    ).read_text()
    rviz_config = package_share / "config" / "display.rviz"
    interface = LaunchConfiguration("interface")
    feedback_topic = LaunchConfiguration("feedback_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "interface",
                default_value="eno1",
                description="Network interface connected to the D1 arm",
            ),
            DeclareLaunchArgument(
                "feedback_topic",
                default_value="current_servo_angle",
                description="Read-only Unitree DDS joint feedback topic",
            ),
            Node(
                package="d1_constrained_description",
                executable="d1_joint_state_bridge",
                parameters=[
                    {
                        "interface": interface,
                        "feedback_topic": feedback_topic,
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
            ),
        ]
    )
