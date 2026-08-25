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


def make_control_block(
    command_port: str,
    feedback_port: str,
    joint_limits,
    command_rate_hz: float,
    command_duration_ms: int,
    prepare_hardware: bool,
    send_repeated_commands: bool,
    command_output_enabled: bool,
    smoothing_mode: int,
    gripper_closed_angle_deg: str,
    gripper_open_angle_deg: str,
    gripper_travel_m: str,
) -> str:
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
      <param name="command_rate_hz">{command_rate_hz}</param>
      <param name="command_duration_ms">{command_duration_ms}</param>
      <param name="feedback_timeout_s">1.5</param>
      <param name="command_limit_tolerance_rad">0.01</param>
      <param name="initial_feedback_timeout_s">10.0</param>
      <param name="hardware_prepare_timeout_s">5.0</param>
      <param name="prepare_hardware">{'true' if prepare_hardware else 'false'}</param>
      <param name="send_repeated_commands">{'true' if send_repeated_commands else 'false'}</param>
      <param name="command_output_enabled">{'true' if command_output_enabled else 'false'}</param>
      <param name="smoothing_mode">{smoothing_mode}</param>
      <param name="gripper_closed_angle_deg">{gripper_closed_angle_deg}</param>
      <param name="gripper_open_angle_deg">{gripper_open_angle_deg}</param>
      <param name="gripper_travel_m">{gripper_travel_m}</param>
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
    command_rate_hz = float(
        LaunchConfiguration("real_command_rate_hz").perform(context)
    ) if backend == "real" else 10.0
    requested_duration_ms = int(
        LaunchConfiguration("real_command_duration_ms").perform(context)
    ) if backend == "real" else 0
    if requested_duration_ms < 0 or requested_duration_ms > 32767:
        raise RuntimeError("real_command_duration_ms must be in [0, 32767]")
    command_duration_ms = requested_duration_ms or int(round(1000.0 / command_rate_hz))
    control_block = make_control_block(
        LaunchConfiguration("command_port").perform(context),
        LaunchConfiguration("feedback_port").perform(context),
        joint_limits,
        command_rate_hz,
        command_duration_ms,
        False,
        backend != "real",
        backend != "real",
        0,
        LaunchConfiguration("gripper_closed_angle_deg").perform(context),
        LaunchConfiguration("gripper_open_angle_deg").perform(context),
        LaunchConfiguration("gripper_travel_m").perform(context),
    )
    control_description = robot_description.replace(
        "</robot>", control_block + "</robot>"
    )
    controllers = control_share / "config" / "controllers.yaml"
    rviz_config = description_share / "config" / "display.rviz"
    gateway = control_prefix / "lib" / "d1_ros2_control" / "d1_dds_gateway"
    gateway_common = [
        str(gateway),
        "--domain",
        LaunchConfiguration("dds_domain_id").perform(context),
    ]
    interface = LaunchConfiguration("interface").perform(context)
    if interface:
        gateway_common.extend(["--interface", interface])
    command_gateway_cmd = gateway_common + [
        "--direction", "command",
        "--command-transport", "servo_angle" if backend == "real" else "arm_command",
        "--command-topic", LaunchConfiguration("command_topic").perform(context),
        "--servo-command-topic", LaunchConfiguration("servo_command_topic").perform(context),
        "--native-segment-topic", LaunchConfiguration("native_segment_topic").perform(context),
        "--command-port", LaunchConfiguration("command_port").perform(context),
        "--feedback-port", LaunchConfiguration("feedback_port").perform(context),
    ]
    feedback_gateway_cmd = gateway_common + [
        "--direction", "feedback",
        "--feedback-topic", LaunchConfiguration("feedback_topic").perform(context),
        "--command-port", LaunchConfiguration("command_port").perform(context),
        "--feedback-port", LaunchConfiguration("feedback_port").perform(context),
    ]
    status_gateway_cmd = gateway_common + [
        "--direction", "status",
        "--status-topic", LaunchConfiguration("status_topic").perform(context),
        "--command-port", LaunchConfiguration("command_port").perform(context),
        "--feedback-port", LaunchConfiguration("feedback_port").perform(context),
    ]

    gateway_actions = [
        ExecuteProcess(
            cmd=command_gateway_cmd,
            additional_env={
                "LD_LIBRARY_PATH": "/usr/local/lib:"
                + os.environ.get("LD_LIBRARY_PATH", "")
            },
            output="screen",
        ),
        ExecuteProcess(
            cmd=feedback_gateway_cmd,
            additional_env={
                "LD_LIBRARY_PATH": "/usr/local/lib:"
                + os.environ.get("LD_LIBRARY_PATH", "")
            },
            output="screen",
        ),
    ]
    # The enhanced onboard executor publishes measured joints directly and
    # owns serial preparation. The vendor rt/arm_Feedback status publisher is
    # intentionally absent in this real backend.

    controller_actions = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        )
    ]
    if backend == "simulation":
        controller_actions.extend([
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
        ])
    else:
        controller_actions.append(Node(
            package="d1_ros2_control",
            executable="d1_native_segment_controller",
            name="d1_native_segment_controller",
            parameters=[{
                "gateway_host": "127.0.0.1",
                "command_port": int(LaunchConfiguration("command_port").perform(context)),
                "native_joint_speed_deg_s": float(
                    LaunchConfiguration("native_joint_speed_deg_s").perform(context)
                ),
                "gripper_closed_angle_deg": float(
                    LaunchConfiguration("gripper_closed_angle_deg").perform(context)
                ),
                "gripper_open_angle_deg": float(
                    LaunchConfiguration("gripper_open_angle_deg").perform(context)
                ),
                "gripper_travel_m": float(
                    LaunchConfiguration("gripper_travel_m").perform(context)
                ),
                "lower_limits": [float(value[0]) for value in joint_limits[:6]] + [0.0],
                "upper_limits": [float(value[1]) for value in joint_limits[:6]] + [
                    float(LaunchConfiguration("gripper_travel_m").perform(context))
                ],
            }],
            output="screen",
        ))

    return gateway_actions + [
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
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ] + controller_actions


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
                "status_topic",
                default_value="rt/arm_Feedback",
                description="D1 native hardware status/ACK topic",
            ),
            DeclareLaunchArgument("command_topic", default_value="rt/arm_Command"),
            DeclareLaunchArgument("servo_command_topic", default_value="set_servo_angle"),
            DeclareLaunchArgument(
                "native_segment_topic", default_value="d1_native_joint_segment"
            ),
            DeclareLaunchArgument(
                "native_joint_speed_deg_s",
                default_value="15.0",
                description="Nominal physical D1 arm-joint speed in degrees/second",
            ),
            DeclareLaunchArgument("feedback_topic", default_value="current_servo_angle"),
            DeclareLaunchArgument(
                "real_command_rate_hz",
                default_value="20.0",
                description="Physical D1 streamed setpoint rate; validate before increasing",
            ),
            DeclareLaunchArgument(
                "real_command_duration_ms",
                default_value="0",
                description=(
                    "Physical servo interpolation duration in milliseconds; "
                    "0 derives it from real_command_rate_hz"
                ),
            ),
            DeclareLaunchArgument("gripper_closed_angle_deg", default_value="-30.0"),
            DeclareLaunchArgument("gripper_open_angle_deg", default_value="60.0"),
            DeclareLaunchArgument("gripper_travel_m", default_value="0.03"),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the D1 description",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
