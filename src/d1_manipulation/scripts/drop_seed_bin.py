#!/usr/bin/env python3
"""Send MuJoCo trash-bin truth to DropObject and verify physical placement."""

from __future__ import annotations

import argparse
import math
import socket
import time

import rclpy
from rclpy.action import ActionClient

from d1_interfaces.action import DropObject


def wait_for_result_or_cancel(node, handle):
    future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        print("\nCtrl+C: canceling DropObject and waiting for the arm to hold...")
        canceled = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, canceled, timeout_sec=3.0)
        return None
    return future.result()


def receive_records(sock: socket.socket, timeout_s: float, object_name: str):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        sock.settimeout(max(0.01, deadline - time.monotonic()))
        lines = sock.recv(8192).decode("ascii").splitlines()
        records = {}
        for line in lines[1:]:
            fields = line.split()
            if len(fields) == 8:
                records[fields[0]] = tuple(float(value) for value in fields[1:4])
        if "trash_bin" in records and object_name in records:
            return records
    raise TimeoutError(f"trash-bin/{object_name} scene truth unavailable")


def discard_queued_packets(sock: socket.socket) -> int:
    """Discard scene-truth datagrams accumulated while the Action was running."""
    discarded = 0
    sock.setblocking(False)
    try:
        while True:
            try:
                sock.recv(8192)
                discarded += 1
            except BlockingIOError:
                return discarded
    finally:
        sock.setblocking(True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=15002)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--bin-inner-radius", type=float, default=0.14)
    parser.add_argument("--bin-height", type=float, default=0.10)
    parser.add_argument(
        "--object", choices=("yellow_cube", "zucchini", "bowl"), default="yellow_cube"
    )
    args = parser.parse_args()
    footprint_radius = {
        "yellow_cube": 0.036, "zucchini": 0.078, "bowl": 0.060,
    }[args.object]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", args.port))
    try:
        initial = receive_records(sock, args.timeout, args.object)
        target = initial["trash_bin"]
        print(
            "TRASH BIN TARGET: "
            f"base=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})"
        )
        rclpy.init()
        node = rclpy.create_node("d1_drop_seed_bin")
        client = ActionClient(node, DropObject, "/arm/tasks/drop_object")
        try:
            if not client.wait_for_server(timeout_sec=args.timeout):
                print("DROP FAILED: DropObject action unavailable")
                return 1
            goal = DropObject.Goal()
            goal.target.header.frame_id = "base_link"
            goal.target.header.stamp = node.get_clock().now().to_msg()
            goal.target.point.x, goal.target.point.y, goal.target.point.z = target

            def feedback(message):
                value = message.feedback
                print(
                    f"[{value.current_state}] {100.0 * value.progress:.0f}% "
                    f"{value.detail}"
                )

            sent = client.send_goal_async(goal, feedback_callback=feedback)
            rclpy.spin_until_future_complete(node, sent)
            handle = sent.result()
            if handle is None or not handle.accepted:
                print("DROP FAILED: goal rejected")
                return 1
            wrapped = wait_for_result_or_cancel(node, handle)
            if wrapped is None:
                return 130
            if wrapped is None or not wrapped.result.success:
                detail = wrapped.result.detail if wrapped is not None else "no result"
                print(f"DROP FAILED: {detail}")
                return 1
            result = wrapped.result
            print(
                "SELECTED RELEASE: "
                f"height_offset={1000.0 * result.height_offset_m:+.0f} mm "
                f"yaw={result.release_yaw_degrees:+.1f} deg"
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()

        discarded = discard_queued_packets(sock)
        print(f"DISCARDED STALE SCENE FRAMES: {discarded}")
        samples = []
        for _ in range(3):
            records = receive_records(sock, args.timeout, args.object)
            samples.append(records[args.object])
            time.sleep(0.15)
        for object_position in samples:
            radial = math.hypot(
                object_position[0] - target[0], object_position[1] - target[1]
            )
            relative_height = object_position[2] - target[2]
            if radial > args.bin_inner_radius - footprint_radius:
                print(
                    f"DROP FAILED: {args.object} radial position "
                    f"{radial:.3f} m is outside bin"
                )
                return 1
            if not 0.0 <= relative_height < args.bin_height:
                print(
                    f"DROP FAILED: {args.object} centre height relative to bin bottom is "
                    f"{relative_height:.3f} m"
                )
                return 1
        displacement = math.dist(samples[0], samples[-1])
        print(
            f"FINAL {args.object.upper()} SAMPLES: "
            + " ".join(
                f"({sample[0]:.3f},{sample[1]:.3f},{sample[2]:.3f})"
                for sample in samples
            )
        )
        if displacement > 0.005:
            print(f"DROP FAILED: cube still moving by {displacement:.3f} m")
            return 1
        print(
            f"DROP SUCCEEDED: {args.object} is stable inside bin "
            f"at ({samples[-1][0]:.3f},{samples[-1][1]:.3f},{samples[-1][2]:.3f})"
        )
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
