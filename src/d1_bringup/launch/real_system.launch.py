import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from d1_bringup.deployment_config import (
    resolve_camera_driver_location,
    resolve_camera_stream_profiles,
    resolve_perception_model_path,
)


def _static_tf(parent, child, translation, quaternion):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", str(translation[0]), "--y", str(translation[1]), "--z", str(translation[2]),
            "--qx", str(quaternion[0]), "--qy", str(quaternion[1]),
            "--qz", str(quaternion[2]), "--qw", str(quaternion[3]),
            "--frame-id", parent, "--child-frame-id", child,
        ],
        output="screen",
    )


def _configured_static_tf(parent, child, transform):
    return _static_tf(
        parent,
        child,
        transform["translation_xyz_m"],
        transform["quaternion_xyzw"],
    )


def _load_yaml(path, description):
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"cannot load {description} {path}: {error}") from error
    if not isinstance(document, dict):
        raise RuntimeError(f"{description} {path} is not a YAML mapping")
    return document


def _launch_setup(context):
    default_config = Path(get_package_share_directory("d1_bringup")) / "config" / "real_machine.yaml"
    requested_config = LaunchConfiguration("config").perform(context).strip()
    config_path = Path(requested_config or os.environ.get("D1_REAL_MACHINE_CONFIG", default_config))
    config = _load_yaml(config_path, "real-machine config")
    if config.get("deployment", {}).get("backend") != "real":
        raise RuntimeError("real_system requires deployment.backend: real")
    perception_model = resolve_perception_model_path(config, config_path)

    expected_ros_domain = int(config["deployment"]["ros_domain_id"])
    actual_ros_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if actual_ros_domain != expected_ros_domain:
        raise RuntimeError(
            f"ROS_DOMAIN_ID={actual_ros_domain}, but config requires {expected_ros_domain}"
        )

    arm = config["arm"]
    requested_joint_speed = LaunchConfiguration(
        "native_joint_speed_deg_s"
    ).perform(context).strip()
    native_joint_speed_deg_s = float(
        requested_joint_speed or arm.get("native_joint_speed_deg_s", 15.0)
    )
    if not math.isfinite(native_joint_speed_deg_s) or native_joint_speed_deg_s <= 0.0:
        raise RuntimeError("native_joint_speed_deg_s must be a finite positive number")
    if not bool(arm.get("onboard_control_deployed", False)):
        raise RuntimeError(
            "real_system requires the D1 onboard control node; deploy and verify "
            "it before setting arm.onboard_control_deployed: true"
        )
    camera = config["wrist_camera"]
    camera_driver_location = resolve_camera_driver_location(
        camera, LaunchConfiguration("camera_driver_location").perform(context).strip()
    )
    color_profile, depth_profile = resolve_camera_stream_profiles(camera)
    site = config["site"]
    gravity_config = site["gravity"]
    arm_serial = LaunchConfiguration("arm_serial").perform(context).strip() or str(
        arm.get("serial_no", "")
    ).strip()
    requested_command_rate = LaunchConfiguration("real_command_rate_hz").perform(context).strip()
    command_rate_hz = float(requested_command_rate or arm["command_rate_hz"])
    if not math.isfinite(command_rate_hz) or command_rate_hz <= 0.0:
        raise RuntimeError("real_command_rate_hz must be a finite positive number")
    requested_duration = LaunchConfiguration("real_command_duration_ms").perform(context).strip()
    command_duration_ms = int(
        requested_duration or arm.get("command_duration_ms", 0)
    )
    if command_duration_ms < 0 or command_duration_ms > 32767:
        raise RuntimeError("real_command_duration_ms must be in [0, 32767]")
    effective_duration_ms = command_duration_ms or int(round(1000.0 / command_rate_hz))

    gravity_path_text = LaunchConfiguration("gravity_calibration").perform(context).strip()
    if not gravity_path_text:
        raise RuntimeError("real_system requires a gravity calibration produced by preflight")
    gravity_path = Path(gravity_path_text)
    gravity = _load_yaml(gravity_path, "gravity calibration")
    expected_parent = gravity_config["parent_frame"]
    expected_child = gravity_config["frame"]
    if gravity.get("parent_frame") != expected_parent or gravity.get("child_frame") != expected_child:
        raise RuntimeError(
            "gravity calibration frame mismatch: "
            f"got {gravity.get('parent_frame')} -> {gravity.get('child_frame')}, "
            f"expected {expected_parent} -> {expected_child}"
        )
    quaternion = gravity.get("quaternion_xyzw", [])
    if len(quaternion) != 4 or not all(math.isfinite(float(value)) for value in quaternion):
        raise RuntimeError("gravity calibration has an invalid quaternion")
    quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if abs(quaternion_norm - 1.0) > 1e-3:
        raise RuntimeError(f"gravity calibration quaternion norm is {quaternion_norm:.6f}")

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

    manipulation_launch = (
        Path(get_package_share_directory("d1_manipulation"))
        / "launch"
        / "observe_target.launch.py"
    )
    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(manipulation_launch)),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "backend": "real",
            "arm_serial": arm_serial,
            "gravity_frame": expected_child,
            "dds_domain_id": str(arm["d1_native_domain_id"]),
            "interface": arm["network_interface"],
            "command_port": str(arm["loopback_command_port"]),
            "feedback_port": str(arm["loopback_feedback_port"]),
            "command_topic": arm["command_topic"],
            "native_segment_topic": arm["native_segment_topic"],
            "native_joint_speed_deg_s": str(native_joint_speed_deg_s),
            "real_command_rate_hz": str(command_rate_hz),
            "real_command_duration_ms": str(command_duration_ms),
            "feedback_topic": arm["feedback_topic"],
            "status_topic": arm["status_topic"],
            "gripper_closed_angle_deg": str(arm["gripper_closed_angle_deg"]),
            "gripper_open_angle_deg": str(arm["gripper_open_angle_deg"]),
            "gripper_travel_m": str(arm["gripper_travel_m"]),
            "joint6_bypass": LaunchConfiguration("joint6_bypass"),
            "perception_model_path": str(perception_model),
        }.items(),
    )

    readiness = Node(
        package="d1_bringup",
        executable="stack_readiness",
        name="d1_stack_readiness",
        parameters=[{
            "joint6_bypass": LaunchConfiguration("joint6_bypass"),
            "color_camera_info_topic": camera["color_camera_info_topic"],
            "aligned_depth_camera_info_topic": camera[
                "aligned_depth_camera_info_topic"
            ],
        }],
        output="screen",
    )

    return [
        LogInfo(
            msg=(
                f"Starting REAL D1 system: serial={arm_serial} config={config_path} "
                f"camera_driver={camera_driver_location} "
                f"perception_model={perception_model} "
                f"command_rate={command_rate_hz:g}Hz "
                f"servo_duration={effective_duration_ms}ms"
            )
        ),
        *realsense_actions,
        _configured_static_tf("go2_base", "base_link", site["go2_base_to_arm_base"]),
        _configured_static_tf("Link6", "wrist_camera_link", camera["link6_to_camera_link"]),
        _static_tf(
            gravity["parent_frame"],
            gravity["child_frame"],
            gravity.get("translation_xyz_m", [0.0, 0.0, 0.0]),
            quaternion,
        ),
        manipulation,
        readiness,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("arm_serial", default_value=""),
        DeclareLaunchArgument(
            "camera_driver_location",
            default_value="",
            description="Override wrist camera driver location: local or remote",
        ),
        DeclareLaunchArgument("gravity_calibration"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("native_joint_speed_deg_s", default_value=""),
        DeclareLaunchArgument("real_command_rate_hz", default_value=""),
        DeclareLaunchArgument("real_command_duration_ms", default_value=""),
        DeclareLaunchArgument("joint6_bypass", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
