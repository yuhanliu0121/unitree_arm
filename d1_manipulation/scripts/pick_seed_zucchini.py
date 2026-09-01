#!/usr/bin/env python3
"""Send the fixed-scene zucchini hint to the staged PickObject Action."""

from __future__ import annotations

import argparse
import socket
import time

import rclpy
from rclpy.action import ActionClient

from d1_interfaces.action import PickObject


def wait_for_result_or_cancel(node, handle):
    future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        print("\nCtrl+C: canceling PickObject and waiting for the arm to hold...")
        canceled = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, canceled, timeout_sec=3.0)
        return None
    return future.result()


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


def receive_zucchini(port: int, timeout_s: float):
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
                if len(fields) == 8 and fields[0] == "zucchini":
                    values = [float(value) for value in fields[1:]]
                    offset = _rotate(*values[3:], (0.00047, -0.00478, 0.01656))
                    return tuple(a + b for a, b in zip(values[:3], offset))
    finally:
        sock.close()
    raise TimeoutError("no zucchini scene truth received")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=15002)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--stage", choices=("compute", "pregrasp", "descend", "lift", "carry"),
        default="compute",
    )
    args = parser.parse_args()
    stages = {"compute": 0, "pregrasp": 1, "descend": 2, "lift": 3, "carry": 4}
    target = receive_zucchini(args.port, args.timeout)
    initial_z = target[2]
    rclpy.init()
    node = rclpy.create_node("d1_pick_seed_zucchini")
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
        wrapped = wait_for_result_or_cancel(node, handle)
        if wrapped is None:
            return 130
        if wrapped is None or not wrapped.result.success:
            detail = wrapped.result.detail if wrapped is not None else "no result"
            print(f"PICK FAILED: {detail}")
            return 1
        result = wrapped.result
        print(
            "SELECTED SEARCH PARAMETERS: "
            f"pregrasp={1000.0 * result.pregrasp_distance_m:+.0f} mm "
            f"tcp_ground_clearance={1000.0 * result.grasp_distance_m:.0f} mm "
            f"yaw={result.grasp_yaw_degrees:.1f} deg "
            f"tilt={result.approach_tilt_degrees:.1f} deg"
        )
        if args.stage in ("lift", "carry"):
            final = receive_zucchini(args.port, args.timeout)
            rise = final[2] - initial_z
            if rise < 0.05:
                print(f"PICK FAILED: zucchini rose only {rise:.3f} m")
                return 1
            print(f"PHYSICAL LIFT VERIFIED: zucchini rise={rise:.3f} m")
        point = result.estimated_center.point
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
