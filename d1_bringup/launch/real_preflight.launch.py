import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from d1_constrained_description import (
    apply_joint_limit_overrides,
    joint_limits_from_urdf,
    load_joint_limit_overrides,
)
from d1_bringup.deployment_config import (
    resolve_camera_driver_location,
    resolve_camera_stream_profiles,
)


def _static_tf(parent, child, transform):
    xyz = [str(value) for value in transform["translation_xyz_m"]]
    quat = [str(value) for value in transform["quaternion_xyzw"]]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", xyz[0], "--y", xyz[1], "--z", xyz[2],
            "--qx", quat[0], "--qy", quat[1], "--qz", quat[2], "--qw", quat[3],
            "--frame-id", parent, "--child-frame-id", child,
        ],
        output="screen",
    )


def _launch_setup(context):
    default_config = Path(get_package_share_directory("d1_bringup")) / "config" / "real_machine.yaml"
    requested_config = LaunchConfiguration("config").perform(context).strip()
    config_path = Path(requested_config or os.environ.get("D1_REAL_MACHINE_CONFIG", default_config))
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if config["deployment"]["backend"] != "real":
        raise RuntimeError("real_preflight requires deployment.backend: real")
    expected_ros_domain = int(config["deployment"]["ros_domain_id"])
    actual_ros_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if actual_ros_domain != expected_ros_domain:
        raise RuntimeError(
            f"ROS_DOMAIN_ID={actual_ros_domain}, but config requires {expected_ros_domain}; "
            f"run with ROS_DOMAIN_ID={expected_ros_domain}"
        )

    arm = config["arm"]
    camera = config["wrist_camera"]
    camera_driver_location = resolve_camera_driver_location(
        camera, LaunchConfiguration("camera_driver_location").perform(context).strip()
    )
    color_profile, depth_profile = resolve_camera_stream_profiles(camera)
    site = config["site"]
    preflight_config = config["preflight"]

    description_share = Path(get_package_share_directory("d1_constrained_description"))
    robot_description = (description_share / "urdf" / "d1_description.urdf").read_text(
        encoding="utf-8"
    )
    arm_serial = LaunchConfiguration("arm_serial").perform(context).strip() or str(
        arm.get("serial_no", "")
    ).strip()
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
    gateway = (
        Path(get_package_prefix("d1_ros2_control"))
        / "lib"
        / "d1_ros2_control"
        / "d1_dds_gateway"
    )
    # This is intentionally ExecuteProcess: the isolated gateway is not a ROS
    # node and must not receive the implicit `--ros-args` suffix from Node().
    feedback_gateway = ExecuteProcess(
        cmd=[str(gateway),
            "--domain", str(arm["d1_native_domain_id"]),
            "--interface", arm["network_interface"],
            "--direction", "feedback",
            "--feedback-topic", arm["feedback_topic"],
            "--command-port", str(arm["loopback_command_port"]),
            "--feedback-port", str(arm["loopback_feedback_port"]),
        ],
        additional_env={
            "LD_LIBRARY_PATH": "/usr/local/lib:" + os.environ.get("LD_LIBRARY_PATH", "")
        },
        output="screen",
    )
    onboard_executor = bool(arm.get("onboard_streaming_deployed", False))
    status_gateway = None if onboard_executor else ExecuteProcess(
        cmd=[str(gateway),
            "--domain", str(arm["d1_native_domain_id"]),
            "--interface", arm["network_interface"],
            "--direction", "status",
            "--status-topic", arm["status_topic"],
            "--command-port", str(arm["loopback_command_port"]),
            "--feedback-port", str(arm["loopback_feedback_port"]),
        ],
        additional_env={
            "LD_LIBRARY_PATH": "/usr/local/lib:" + os.environ.get("LD_LIBRARY_PATH", "")
        },
        output="screen",
    )

    realsense_actions = []
    if camera_driver_location == "local":
        rs_launch = (
            Path(get_package_share_directory("realsense2_camera"))
            / "launch"
            / "rs_launch.py"
        )
        realsense_actions.append(GroupAction(
            scoped=True,
            forwarding=False,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rs_launch)),
                launch_arguments={
                    "camera_namespace": "/",
                    "camera_name": camera["name"],
                    "serial_no": "_" + camera["serial_no"],
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "enable_accel": "true",
                    "enable_gyro": "true",
                    "unite_imu_method": "2",
                    "rgb_camera.color_profile": color_profile,
                    "depth_module.depth_profile": depth_profile,
                    "publish_tf": "true",
                }.items(),
            )],
        ))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    mount_tf = _static_tf(
        "go2_base", "base_link", site["go2_base_to_arm_base"]
    )
    camera_tf = _static_tf(
        "Link6", "wrist_camera_link", camera["link6_to_camera_link"]
    )

    limits = []
    for lower, upper in joint_limits_from_urdf(
        robot_description, [f"Joint{index}" for index in range(7)]
    ):
        limits.extend([lower, upper])

    gravity = site["gravity"]
    preflight = Node(
        package="d1_bringup",
        executable="real_preflight",
        name="d1_real_preflight",
        parameters=[{
            "network_interface": arm["network_interface"],
            "expected_ipv4_subnet": arm["expected_ipv4_subnet"],
            "feedback_port": arm["loopback_feedback_port"],
            "gripper_closed_angle_deg": arm["gripper_closed_angle_deg"],
            "gripper_open_angle_deg": arm["gripper_open_angle_deg"],
            "gripper_travel_m": arm["gripper_travel_m"],
            "color_camera_info_topic": camera["color_camera_info_topic"],
            "aligned_depth_camera_info_topic": camera["aligned_depth_camera_info_topic"],
            "imu_topic": camera["imu_topic"],
            "gravity_source": gravity["source"],
            "gravity_parent_frame": gravity["parent_frame"],
            "gravity_frame": gravity["frame"],
            "expected_acceleration_mps2": gravity["expected_acceleration_mps2"],
            "acceleration_norm_tolerance_mps2": gravity["norm_tolerance_mps2"],
            "acceleration_max_component_standard_error_mps2": gravity[
                "max_component_standard_error_mps2"
            ],
            "angular_velocity_max_rms_rad_s": gravity[
                "max_angular_velocity_rms_rad_s"
            ],
            "gravity_required_samples": gravity["required_samples"],
            "gravity_output_path": LaunchConfiguration("gravity_output_path"),
            "timeout_s": preflight_config["timeout_s"],
            "minimum_arm_feedback_samples": preflight_config["minimum_arm_feedback_samples"],
            "feedback_max_age_s": preflight_config["feedback_max_age_s"],
            "camera_info_max_age_s": preflight_config["camera_info_max_age_s"],
            "joint_limit_tolerance_rad": preflight_config["joint_limit_tolerance_rad"],
            "require_arm_hardware_status": not onboard_executor,
            "joint_limits": limits,
        }],
        output="screen",
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=preflight,
            on_exit=[EmitEvent(event=Shutdown(reason="real preflight finished"))],
        )
    )
    return [
        feedback_gateway,
        *([] if status_gateway is None else [status_gateway]),
        *realsense_actions,
        robot_state_publisher,
        mount_tf,
        camera_tf,
        preflight,
        shutdown,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "arm_serial",
            default_value="",
            description="Explicit physical D1 serial for per-unit URDF adjustments",
        ),
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="Physical deployment YAML; defaults to D1_REAL_MACHINE_CONFIG or package config",
        ),
        DeclareLaunchArgument(
            "camera_driver_location",
            default_value="",
            description="Override wrist camera driver location: local or remote",
        ),
        DeclareLaunchArgument("gravity_output_path", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
