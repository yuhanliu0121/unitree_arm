#!/usr/bin/env python3
"""Send one safe D1 funcode=2/mode=1 gripper test goal.

The six arm targets are copied from a fresh /joint_states sample.  This tool is
for isolated real-hardware validation only; it deliberately bypasses the ROS 2
gripper controller.
"""

import argparse
import ctypes
import math
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


MAGIC = 0x44314350
VERSION = 2
COMMAND_KIND = 1


class JointPacket(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint16),
        ("kind", ctypes.c_uint16),
        ("sequence", ctypes.c_uint64),
        ("smoothing_mode", ctypes.c_uint32),
        ("duration_ms", ctypes.c_uint32),
        ("acceleration_ms", ctypes.c_uint32),
        ("deceleration_ms", ctypes.c_uint32),
        ("angle_deg", ctypes.c_double * 7),
    ]


assert ctypes.sizeof(JointPacket) == 88


class FeedbackReader(Node):
    def __init__(self) -> None:
        super().__init__("d1_mode1_gripper_goal")
        self.positions = None
        self.received_at = 0.0
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data
        )

    def _on_joint_state(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        names = [f"Joint{index}" for index in range(7)]
        if not all(name in values and math.isfinite(values[name]) for name in names):
            return
        self.positions = [values[name] for name in names]
        self.received_at = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target_deg", type=float, help="Joint6 target in degrees")
    parser.add_argument("--command-port", type=int, default=15100)
    parser.add_argument("--feedback-timeout", type=float, default=2.0)
    args = parser.parse_args()
    if not -30.0 <= args.target_deg <= 60.0:
        parser.error("target_deg must be within the D1095 configured range [-30, 60]")

    rclpy.init()
    node = FeedbackReader()
    deadline = time.monotonic() + args.feedback_timeout
    try:
        while node.positions is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.positions is None or time.monotonic() - node.received_at > 0.5:
            print("ERROR: no fresh /joint_states sample; command not sent", file=sys.stderr)
            return 1

        packet = JointPacket()
        packet.magic = MAGIC
        packet.version = VERSION
        packet.kind = COMMAND_KIND
        packet.sequence = time.monotonic_ns()
        packet.smoothing_mode = 1
        for index in range(6):
            packet.angle_deg[index] = math.degrees(node.positions[index])
        packet.angle_deg[6] = args.target_deg

        arm_text = ", ".join(f"{packet.angle_deg[i]:.2f}" for i in range(6))
        print(
            f"Sending funcode=2 mode=1: arm=[{arm_text}] "
            f"Joint6={args.target_deg:.2f} deg"
        )
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sent = sender.sendto(bytes(packet), ("127.0.0.1", args.command_port))
        if sent != ctypes.sizeof(packet):
            print("ERROR: incomplete UDP command", file=sys.stderr)
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
