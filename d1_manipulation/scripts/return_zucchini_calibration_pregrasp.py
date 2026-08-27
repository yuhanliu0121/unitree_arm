#!/usr/bin/env python3
"""Return an open gripper from calibration DESCEND to a recorded PREGRASP."""

import argparse
import math
import sys
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from d1_ros2_control.action import ExecuteJointSegment
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CalibrationPoseReturn(Node):
    def __init__(self, config: dict) -> None:
        super().__init__("return_zucchini_calibration_pregrasp")
        self.config = config
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.client = ActionClient(
            self, ExecuteJointSegment, str(config["action_name"])
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.latest_joint_state = message

    def wait_for_feedback(self, timeout_s: float) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.latest_joint_state is None:
            if self.get_clock().now().nanoseconds >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_joint_state is not None

    def validate_start(self) -> tuple[bool, str]:
        state = self.latest_joint_state
        if state is None:
            return False, "joint feedback is unavailable"
        measured = dict(zip(state.name, state.position))
        names = list(self.config["joint_names"])
        targets = list(self.config["positions_rad"])
        missing = [name for name in names + ["Joint6"] if name not in measured]
        if missing:
            return False, "joint feedback is missing: " + ", ".join(missing)
        errors = [abs(measured[name] - target) for name, target in zip(names, targets)]
        worst_index = max(range(len(errors)), key=errors.__getitem__)
        limit = float(self.config["maximum_start_delta_rad"])
        if errors[worst_index] > limit:
            return False, (
                f"{names[worst_index]} is {math.degrees(errors[worst_index]):.1f} deg "
                f"from the recorded PREGRASP; limit is {math.degrees(limit):.1f} deg"
            )
        minimum_open = float(self.config["minimum_gripper_opening_m"])
        if measured["Joint6"] < minimum_open:
            return False, (
                f"gripper is not safely open: {measured['Joint6'] * 1000.0:.1f} mm "
                f"< {minimum_open * 1000.0:.1f} mm"
            )
        return True, (
            f"start accepted; maximum arm delta={math.degrees(max(errors)):.1f} deg, "
            f"gripper={measured['Joint6'] * 1000.0:.1f} mm"
        )

    def execute(self, timeout_s: float) -> tuple[bool, str]:
        if not self.client.wait_for_server(timeout_sec=5.0):
            return False, f"action server unavailable: {self.config['action_name']}"
        goal = ExecuteJointSegment.Goal()
        goal.joint_names = list(self.config["joint_names"])
        goal.positions = [float(value) for value in self.config["positions_rad"]]
        goal.motion_profile = ExecuteJointSegment.Goal.COMMON_ARRIVAL
        goal.speed_deg_s = float(self.config["speed_deg_s"])
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=5.0)
        if not sent.done() or sent.result() is None or not sent.result().accepted:
            return False, "calibration PREGRASP command was rejected"
        handle = sent.result()
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout_s)
        if not result.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=2.0)
            return False, "calibration PREGRASP return timed out and was canceled"
        wrapped = result.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = "unavailable" if wrapped is None else str(wrapped.status)
            return False, f"calibration PREGRASP return failed (status={status})"
        if not wrapped.result.success:
            return False, wrapped.result.detail
        return True, wrapped.result.detail


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Return from an open-gripper zucchini calibration DESCEND to the recorded PREGRASP."
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--motion-timeout", type=float, default=20.0)
    options, ros_args = parser.parse_known_args(argv)
    if options.confirm != "ZUCCHINI_CALIBRATION_MOVE":
        parser.error("physical motion requires --confirm ZUCCHINI_CALIBRATION_MOVE")
    return options, ros_args


def main(argv=None) -> int:
    options, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    config_path = Path(options.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    rclpy.init(args=ros_args)
    node = CalibrationPoseReturn(config)
    try:
        if not node.wait_for_feedback(options.feedback_timeout):
            print("ERROR: joint feedback is unavailable", file=sys.stderr)
            return 2
        accepted, detail = node.validate_start()
        if not accepted:
            print(f"ERROR: {detail}", file=sys.stderr)
            return 2
        print(detail)
        print("Returning to the recorded zucchini calibration PREGRASP...")
        succeeded, detail = node.execute(options.motion_timeout)
        if not succeeded:
            print(f"ERROR: {detail}", file=sys.stderr)
            return 1
        print(f"ZUCCHINI CALIBRATION PREGRASP REACHED: {detail}")
        return 0
    except KeyboardInterrupt:
        print("\nCanceled; no follow-up command was sent.", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
