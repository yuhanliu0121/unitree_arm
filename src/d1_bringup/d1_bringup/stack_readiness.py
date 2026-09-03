#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool


GREEN = "\033[1;92m"
RED = "\033[1;91m"
RESET = "\033[0m"


class StackReadiness(Node):
    def __init__(self):
        super().__init__("d1_stack_readiness")
        self.declare_parameter("timeout_s", 60.0)
        self.declare_parameter("freshness_s", 2.0)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "color_camera_info_topic", "/wrist_camera/color/camera_info"
        )
        self.declare_parameter(
            "aligned_depth_camera_info_topic",
            "/wrist_camera/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter(
            "required_actions",
            [
                "/move_action",
                "/arm_controller/follow_joint_trajectory",
                "/gripper_controller/gripper_cmd",
                "/arm/internal/observe_target",
                "/arm/tasks/pick_object",
                "/arm/tasks/drop_object",
            ],
        )
        self.declare_parameter(
            "required_services",
            [
                "/arm/perception/detect_target",
                "/arm/perception/estimate_cube",
                "/arm/perception/estimate_zucchini",
                "/arm/perception/estimate_bowl",
                "/arm/perception/verify_held_object",
            ],
        )

        self.started = time.monotonic()
        self.last_joint_state = None
        self.joint_names = set()
        self.last_color_info = None
        self.last_depth_info = None
        self.base_planning_scene_ready = False
        self.timeout_reported = False
        self.ready = False

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/arm/planning_scene_ready",
            self._on_planning_scene_ready,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("color_camera_info_topic").value),
            self._on_color_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("aligned_depth_camera_info_topic").value),
            self._on_depth_info,
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._evaluate)

    def _on_joint_state(self, message):
        self.last_joint_state = time.monotonic()
        self.joint_names = set(message.name)

    def _on_color_info(self, _message):
        self.last_color_info = time.monotonic()

    def _on_depth_info(self, _message):
        self.last_depth_info = time.monotonic()

    def _on_planning_scene_ready(self, message):
        self.base_planning_scene_ready = bool(message.data)

    def _missing(self):
        now = time.monotonic()
        freshness = float(self.get_parameter("freshness_s").value)
        missing = []

        required_joints = {f"Joint{index}" for index in range(7)}
        if (
            self.last_joint_state is None
            or now - self.last_joint_state > freshness
            or not required_joints.issubset(self.joint_names)
        ):
            missing.append("fresh D1 Joint0..6 feedback")
        if self.last_color_info is None or now - self.last_color_info > freshness:
            missing.append("wrist RGB CameraInfo")
        if self.last_depth_info is None or now - self.last_depth_info > freshness:
            missing.append("wrist aligned-depth CameraInfo")
        if not self.base_planning_scene_ready:
            missing.append("base MoveIt planning scene (ground and Go2 platform)")

        available_services = {
            name for name, _types in self.get_service_names_and_types()
        }
        for action in self.get_parameter("required_actions").value:
            action_services = {
                f"{action}/_action/send_goal",
                f"{action}/_action/get_result",
            }
            if not action_services.issubset(available_services):
                missing.append(f"action {action}")
        for service in self.get_parameter("required_services").value:
            if service not in available_services:
                missing.append(f"service {service}")
        return missing

    def _evaluate(self):
        if self.ready:
            return
        missing = self._missing()
        if not missing:
            banner = (
                "\n************************************************************\n"
                "* D1 REAL CONTROL STACK READY                              *\n"
                "* pick_object and drop_object are ready to accept commands *\n"
                "************************************************************"
            )
            print(f"{GREEN}{banner}{RESET}", flush=True)
            self.ready = True
            return

        timeout = float(self.get_parameter("timeout_s").value)
        if not self.timeout_reported and time.monotonic() - self.started >= timeout:
            detail = "\n  - ".join(missing)
            print(
                f"{RED}\n***** D1 STACK NOT READY AFTER {timeout:.0f} s *****\n"
                f"Missing:\n  - {detail}\n"
                "The monitor will keep waiting; do not send a task yet."
                f"{RESET}",
                flush=True,
            )
            self.timeout_reported = True


def main(args=None):
    rclpy.init(args=args)
    node = StackReadiness()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
