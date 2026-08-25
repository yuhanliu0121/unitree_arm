#!/usr/bin/env python3

import argparse
import math
import sys
from typing import Optional

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
STOWED = [0.0, -1.54, 1.55, 0.0, 0.0, 0.0]
OPEN_GRIPPER_M = 0.03


class StowedRecovery(Node):
    def __init__(self) -> None:
        super().__init__("d1_recover_stowed")
        self.arm = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.gripper = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.active_goal = None

    def wait_for_servers(self, timeout_s: float) -> bool:
        print("Waiting for the streaming arm controller...")
        if not self.arm.wait_for_server(timeout_sec=timeout_s):
            print(f"ERROR: action server unavailable: {ARM_ACTION}", file=sys.stderr)
            return False
        if not self.gripper.wait_for_server(timeout_sec=timeout_s):
            print(f"ERROR: action server unavailable: {GRIPPER_ACTION}", file=sys.stderr)
            return False
        return True

    def _wait(self, future, timeout_s: float) -> bool:
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return future.done()

    def cancel_active_goal(self) -> None:
        if self.active_goal is None:
            return
        cancel = self.active_goal.cancel_goal_async()
        self._wait(cancel, 2.0)

    def recover_arm(self, timeout_s: float, nominal_duration_s: float) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f"Joint{index}" for index in range(6)]
        point = JointTrajectoryPoint()
        point.positions = STOWED
        nanoseconds = int(round(nominal_duration_s * 1_000_000_000.0))
        point.time_from_start.sec = nanoseconds // 1_000_000_000
        point.time_from_start.nanosec = nanoseconds % 1_000_000_000
        goal.trajectory.points = [point]

        print("[1/2] Moving Joint0..5 to canonical STOWED through the streaming controller...")
        sent = self.arm.send_goal_async(goal)
        if not self._wait(sent, 5.0) or sent.result() is None:
            print("ERROR: timed out while sending the STOWED arm goal", file=sys.stderr)
            return False
        self.active_goal = sent.result()
        if not self.active_goal.accepted:
            print("ERROR: the streaming arm controller rejected the STOWED goal", file=sys.stderr)
            self.active_goal = None
            return False

        result = self.active_goal.get_result_async()
        if not self._wait(result, timeout_s):
            self.cancel_active_goal()
            self.active_goal = None
            print("ERROR: STOWED arm motion timed out and was canceled", file=sys.stderr)
            return False
        self.active_goal = None
        wrapped = result.result()
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = "unavailable" if wrapped is None else str(wrapped.status)
            print(f"ERROR: STOWED arm motion did not succeed (status={status})", file=sys.stderr)
            return False
        response = wrapped.result
        if response.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            print(
                "ERROR: STOWED arm motion failed: "
                f"code={response.error_code} detail={response.error_string}",
                file=sys.stderr,
            )
            return False
        return True

    def open_gripper(self, timeout_s: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = OPEN_GRIPPER_M
        goal.command.max_effort = 0.0
        print("[2/2] Opening Joint6 fully through the streaming controller...")
        sent = self.gripper.send_goal_async(goal)
        if not self._wait(sent, 5.0) or sent.result() is None:
            print("ERROR: timed out while sending the gripper goal", file=sys.stderr)
            return False
        self.active_goal = sent.result()
        if not self.active_goal.accepted:
            print("ERROR: the streaming controller rejected the gripper goal", file=sys.stderr)
            self.active_goal = None
            return False

        result = self.active_goal.get_result_async()
        if not self._wait(result, timeout_s):
            self.cancel_active_goal()
            self.active_goal = None
            print("ERROR: gripper opening timed out and was canceled", file=sys.stderr)
            return False
        self.active_goal = None
        wrapped = result.result()
        if (
            wrapped is None
            or wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped.result.reached_goal
        ):
            status = "unavailable" if wrapped is None else str(wrapped.status)
            print(f"ERROR: gripper did not reach fully open (status={status})", file=sys.stderr)
            return False
        return True


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Recover a physical D1 to STOWED through the active streaming controller."
    )
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--server-timeout", type=float, default=5.0)
    parser.add_argument("--arm-timeout", type=float, default=45.0)
    parser.add_argument("--gripper-timeout", type=float, default=15.0)
    parser.add_argument("--nominal-duration", type=float, default=8.0)
    args, ros_args = parser.parse_known_args(argv)
    if args.confirm != "STOWED_MOVE":
        parser.error("physical motion requires --confirm STOWED_MOVE")
    for name in (
        "server_timeout", "arm_timeout", "gripper_timeout", "nominal_duration"
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    return args, ros_args


def main(args: Optional[list] = None) -> int:
    options, ros_args = parse_arguments(sys.argv[1:] if args is None else args)
    rclpy.init(args=ros_args)
    node = StowedRecovery()
    try:
        print("WARNING: this commands the physical D1 through the active streaming stack.")
        print("Support the arm and clear its workspace before continuing.")
        if not node.wait_for_servers(options.server_timeout):
            return 2
        if not node.recover_arm(options.arm_timeout, options.nominal_duration):
            return 1
        if not node.open_gripper(options.gripper_timeout):
            return 1
        print("STOWED reached and Joint6 is fully open.")
        return 0
    except KeyboardInterrupt:
        print("\nCanceled by user; requesting the controller to hold measured position.")
        node.cancel_active_goal()
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
