#!/usr/bin/env python3
"""Send the seed-scene cube hint to the staged visual PickObject Action."""

from __future__ import annotations

import argparse
import socket
import time
import rclpy
from rclpy.action import ActionClient

from d1_manipulation.action import PickObject

def _rotate(qw, qx, qy, qz, vector):
    x, y, z = vector
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def receive_cube(port: int, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", port))
    try:
        while time.monotonic() < deadline:
            sock.settimeout(max(0.01, deadline - time.monotonic()))
            payload = sock.recv(4096).decode("ascii").splitlines()
            for line in payload[1:]:
                fields = line.split()
                if len(fields) == 8 and fields[0] == "yellow_cube":
                    values = [float(value) for value in fields[1:]]
                    offset = _rotate(*values[3:], (-0.000787, -0.000889, 0.025))
                    return tuple(a + b for a, b in zip(values[:3], offset))
    finally:
        sock.close()
    raise TimeoutError("no yellow_cube scene truth received")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=15002)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--stage", choices=("compute", "pregrasp", "descend", "lift"), default="compute")
    args = parser.parse_args()
    stages = {"compute": 0, "pregrasp": 1, "descend": 2, "lift": 3}
    target = receive_cube(args.port, args.timeout)
    initial_z = target[2]
    rclpy.init()
    node = rclpy.create_node("d1_pick_seed_cube")
    client = ActionClient(node, PickObject, "/arm/tasks/pick_object")
    try:
        if not client.wait_for_server(timeout_sec=args.timeout):
            print("PICK FAILED: PickObject action unavailable")
            return 1
        goal = PickObject.Goal()
        goal.target.header.frame_id = "base_link"
        goal.target.header.stamp = node.get_clock().now().to_msg()
        goal.target.point.x, goal.target.point.y, goal.target.point.z = target
        goal.stop_after = stages[args.stage]

        def feedback(message):
            value = message.feedback
            print(f"[{value.current_state}] {100.0 * value.progress:.0f}% {value.detail}")

        sent = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            print("PICK FAILED: goal rejected")
            return 1
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        if wrapped is None or not wrapped.result.success:
            detail = wrapped.result.detail if wrapped is not None else "no result"
            print(f"PICK FAILED: {detail}")
            return 1
        result = wrapped.result
        point = result.estimated_center.point
        grasp = result.grasp_pose.pose
        pregrasp = result.pregrasp_pose.pose
        print(
            "GRASP POSE: "
            f"p=({grasp.position.x:.4f},{grasp.position.y:.4f},{grasp.position.z:.4f}) "
            f"q=({grasp.orientation.x:.5f},{grasp.orientation.y:.5f},"
            f"{grasp.orientation.z:.5f},{grasp.orientation.w:.5f})"
        )
        print(
            "PREGRASP POSE: "
            f"p=({pregrasp.position.x:.4f},{pregrasp.position.y:.4f},{pregrasp.position.z:.4f})"
        )
        print(
            "SELECTED SEARCH PARAMETERS: "
            f"pregrasp={1000.0 * result.pregrasp_distance_m:+.0f} mm "
            f"grasp={1000.0 * result.grasp_distance_m:+.0f} mm "
            f"yaw={result.grasp_yaw_degrees:.1f} deg "
            f"tilt={result.approach_tilt_degrees:.1f} deg"
        )
        if args.stage == "lift":
            final = receive_cube(args.port, args.timeout)
            rise = final[2] - initial_z
            if rise < 0.05:
                print(f"PICK FAILED: cube rose only {rise:.3f} m")
                return 1
            print(f"PHYSICAL LIFT VERIFIED: cube rise={rise:.3f} m")
        print(
            f"PICK STAGE SUCCEEDED: {args.stage} class={result.class_name} "
            f"center=({point.x:.3f},{point.y:.3f},{point.z:.3f})"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
