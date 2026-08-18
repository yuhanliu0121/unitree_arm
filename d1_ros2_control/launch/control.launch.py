import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from d1_constrained_description import (
    apply_joint_limit_overrides,
    joint_limits_from_urdf,
    load_joint_limit_overrides,
)


def make_control_block(command_port: str, feedback_port: str, joint_limits) -> str:
    joints = []
    for index, (lower, upper) in enumerate(joint_limits):
        joints.append(
            f"""
    <joint name="Joint{index}">
      <command_interface name="position">
        <param name="min">{lower}</param>
        <param name="max">{upper}</param>
      </command_interface>
      <state_interface name="position"/>{'<state_interface name="velocity"/>' if index == 6 else ''}
    </joint>"""
        )
    return f"""
  <ros2_control name="D1System" type="system">
    <hardware>
      <plugin>d1_ros2_control/D1SystemHardware</plugin>
      <param name="gateway_host">127.0.0.1</param>
      <param name="command_port">{command_port}</param>
      <param name="feedback_port">{feedback_port}</param>
      <param name="command_rate_hz">10.0</param>
      <param name="feedback_timeout_s">0.5</param>
      <param name="initial_feedback_timeout_s">10.0</param>
      <param name="smoothing_mode">0</param>
      <param name="gripper_closed_angle_deg">-30.0</param>
      <param name="gripper_open_angle_deg">60.0</param>
      <param name="gripper_travel_m">0.03</param>
    </hardware>
{''.join(joints)}
  </ros2_control>
"""


def launch_setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    if backend not in {"simulation", "real"}:
        raise RuntimeError(f"unsupported backend: {backend}")
    control_share = Path(get_package_share_directory("d1_ros2_control"))
    control_prefix = Path(get_package_prefix("d1_ros2_control"))
    description_share = Path(
        get_package_share_directory("d1_constrained_description")
    )
    robot_description = (
        description_share / "urdf" / "d1_description.urdf"
    ).read_text(encoding="utf-8")
    requested_arm_serial = LaunchConfiguration("arm_serial").perform(context).strip()
    arm_serial = requested_arm_serial if backend == "real" else ""
    if requested_arm_serial and backend != "real":
        print("D1 hardware profile: ignored arm_serial for simulation backend")
    profile_path = description_share / "config" / "hardware_profiles.yaml"
    overrides = load_joint_limit_overrides(profile_path, arm_serial)
    if arm_serial and not overrides:
        print(
            f"D1 hardware profile: serial {arm_serial!r} has no profile; "
            "using the base URDF unchanged"
        )
    elif overrides:
        print(f"D1 hardware profile: applying explicit serial {arm_serial}")
    robot_description = apply_joint_limit_overrides(robot_description, overrides)
    joint_limits = joint_limits_from_urdf(
        robot_description, [f"Joint{index}" for index in range(7)]
    )
    control_block = make_control_block(
        LaunchConfiguration("command_port").perform(context),
        LaunchConfiguration("feedback_port").perform(context),
        joint_limits,
    )
    control_description = robot_description.replace(
        "</robot>", control_block + "</robot>"
    )
    controllers = control_share / "config" / "controllers.yaml"
    rviz_config = description_share / "config" / "display.rviz"
    gateway = control_prefix / "lib" / "d1_ros2_control" / "d1_dds_gateway"
    gateway_cmd = [
        str(gateway),
        "--domain",
        LaunchConfiguration("dds_domain_id").perform(context),
        "--command-topic",
        "rt/arm_Command",
        "--feedback-topic",
        "current_servo_angle",
        "--command-port",
        LaunchConfiguration("command_port").perform(context),
        "--feedback-port",
        LaunchConfiguration("feedback_port").perform(context),
    ]
    interface = LaunchConfiguration("interface").perform(context)
    if interface:
        gateway_cmd[3:3] = ["--interface", interface]

    return [
        ExecuteProcess(
            cmd=gateway_cmd,
            additional_env={
                "LD_LIBRARY_PATH": "/usr/local/lib:"
                + os.environ.get("LD_LIBRARY_PATH", "")
            },
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": control_description},
                str(controllers),
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
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                "dds_domain_id",
                default_value="42",
                description="D1 native DDS domain; 42 is the safe simulation default",
            ),
            DeclareLaunchArgument(
                "interface",
                default_value="",
                description="Optional network interface for native D1 DDS",
            ),
            DeclareLaunchArgument(
                "command_port",
                default_value="15000",
                description="Loopback UDP port consumed by the isolated D1 gateway",
            ),
            DeclareLaunchArgument(
                "feedback_port",
                default_value="15001",
                description="Loopback UDP port consumed by the hardware plugin",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the D1 description",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
