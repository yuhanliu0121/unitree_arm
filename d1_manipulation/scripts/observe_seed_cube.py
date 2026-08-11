#!/usr/bin/env python3

"""Send the current seed-scene yellow cube centre to ObserveTarget."""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time

import rclpy
from rclpy.action import ActionClient

from d1_manipulation.action import ObserveTarget
from d1_manipulation.srv import DetectTarget


def rotate(qw: float, qx: float, qy: float, qz: float, vector):
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
            if not payload or not payload[0].startswith("D1SCENE 1 "):
                continue
            for line in payload[1:]:
                fields = line.split()
                if len(fields) != 8 or fields[0] != "yellow_cube":
                    continue
                values = [float(value) for value in fields[1:]]
                position = values[:3]
                qw, qx, qy, qz = values[3:]
                offset = rotate(
                    qw, qx, qy, qz, (-0.000787, -0.000889, 0.025)
                )
                return tuple(a + b for a, b in zip(position, offset))
    finally:
        sock.close()
    raise TimeoutError("no yellow_cube scene truth received")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=15002)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--action", default="/arm/debug/observe_target"
    )
    parser.add_argument(
        "--detect-service", default="/arm/perception/detect_target"
    )
    args = parser.parse_args()

    try:
        target = receive_cube(args.port, args.timeout)
    except (OSError, TimeoutError) as exception:
        print(f"Failed to obtain yellow cube truth: {exception}", file=sys.stderr)
        return 1
    if not all(math.isfinite(value) for value in target):
        print("Received non-finite cube coordinates", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("d1_observe_seed_cube")
    client = ActionClient(node, ObserveTarget, args.action)
    try:
        if not client.wait_for_server(timeout_sec=args.timeout):
            print(f"ObserveTarget server unavailable: {args.action}", file=sys.stderr)
            return 1
        goal = ObserveTarget.Goal()
        goal.target.header.frame_id = "base_link"
        goal.target.header.stamp = node.get_clock().now().to_msg()
        goal.target.point.x, goal.target.point.y, goal.target.point.z = target
        print(
            "Sending yellow cube centre: "
            f"x={target[0]:.3f} y={target[1]:.3f} z={target[2]:.3f}"
        )

        def feedback(message):
            value = message.feedback
            if value.candidate_index:
                print(
                    f"[{value.current_state}] candidate "
                    f"{value.candidate_index}/{value.candidate_count}: "
                    f"beta={value.beta_deg:.1f} alpha={value.alpha_deg:.1f} "
                    f"d={value.distance_m:.2f}"
                )
            else:
                print(f"[{value.current_state}] {value.detail}")

        send_future = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("ObserveTarget goal was rejected", file=sys.stderr)
            return 1
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        if wrapped is None or not wrapped.result.success:
            detail = wrapped.result.detail if wrapped is not None else "no result"
            print(f"OBSERVE FAILED: {detail}", file=sys.stderr)
            return 1
        result = wrapped.result
        print(
            "OBSERVE SUCCEEDED: "
            f"beta={result.selected_beta_deg:.1f} "
            f"alpha={result.selected_alpha_deg:.1f} "
            f"distance={result.selected_distance_m:.2f}"
        )
        detect_client = node.create_client(DetectTarget, args.detect_service)
        if not detect_client.wait_for_service(timeout_sec=args.timeout):
            print(
                f"DetectTarget service unavailable: {args.detect_service}",
                file=sys.stderr,
            )
            return 1
        request = DetectTarget.Request()
        request.target_hint.header.frame_id = "base_link"
        request.target_hint.header.stamp = node.get_clock().now().to_msg()
        (
            request.target_hint.point.x,
            request.target_hint.point.y,
            request.target_hint.point.z,
        ) = target
        detect_future = detect_client.call_async(request)
        rclpy.spin_until_future_complete(
            node, detect_future, timeout_sec=args.timeout + 5.0
        )
        detection = detect_future.result()
        if detection is None or not detection.success:
            detail = detection.detail if detection is not None else "no response"
            print(f"DETECTION FAILED: {detail}", file=sys.stderr)
            return 1
        print(
            "DETECTION SUCCEEDED: "
            f"class={detection.class_name} confidence={detection.confidence:.3f} "
            f"pixel=({detection.center_u},{detection.center_v}) "
            f"base=({detection.position_base.point.x:.3f},"
            f"{detection.position_base.point.y:.3f},"
            f"{detection.position_base.point.z:.3f})"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
