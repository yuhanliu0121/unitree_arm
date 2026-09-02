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
        super().__init__("return_calibration_pregrasp")
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
    parser.add_argument("--config")
    parser.add_argument(
        "--session-root",
        help="calibration root containing .active_session and recorded DESCEND",
    )
    parser.add_argument(
        "--session-target", choices=("pregrasp", "descend"), default="pregrasp"
    )
    parser.add_argument(
        "--record-current-as-descend", action="store_true",
        help="save current Joint0..5 as this session's validated DESCEND target",
    )
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--motion-timeout", type=float, default=20.0)
    options, ros_args = parser.parse_known_args(argv)
    if not options.config and not options.session_root:
        parser.error("one of --config or --session-root is required")
    if options.session_target == "descend" and not options.session_root:
        parser.error("DESCEND target requires --session-root")
    if options.session_root and options.session_target == "descend":
        expected = "CUBE_CALIBRATION_DESCEND"
    elif options.session_root:
        expected = "CUBE_CALIBRATION_MOVE"
    else:
        expected = "ZUCCHINI_CALIBRATION_MOVE"
    if options.confirm != expected:
        parser.error(f"physical motion requires --confirm {expected}")
    return options, ros_args


def active_session(root: Path) -> Path:
    active_path = root / ".active_session"
    if not active_path.is_file():
        raise RuntimeError("no active repeated-closing calibration session")
    return root / active_path.read_text(encoding="utf-8").strip()


def load_session_config(root: Path, target: str) -> dict:
    session = active_session(root)
    if target == "descend":
        descent_path = session / "validated_descend.yaml"
        if not descent_path.is_file():
            raise RuntimeError(
                "validated DESCEND is not recorded; run the first return with "
                "--record-current-as-descend after a successful physical descent"
            )
        return yaml.safe_load(descent_path.read_text(encoding="utf-8"))
    manifest = yaml.safe_load((session / "session.json").read_text(encoding="utf-8"))
    samples = manifest.get("samples", {})
    if not samples:
        raise RuntimeError("active calibration has no captured PREGRASP sample")
    first_relative = next(iter(samples.values()))
    sample = yaml.safe_load((session / first_relative).read_text(encoding="utf-8"))
    return {
        "joint_names": [f"Joint{index}" for index in range(6)],
        "positions_rad": sample["arm_joint_positions_rad"],
        "maximum_start_delta_rad": math.radians(30.0),
        "minimum_gripper_opening_m": 0.025,
        "speed_deg_s": 15.0,
        "action_name": "/arm_controller/execute_joint_segment",
    }


def record_descend(root: Path, message: JointState) -> Path:
    measured = dict(zip(message.name, message.position))
    missing = [f"Joint{index}" for index in range(6) if f"Joint{index}" not in measured]
    if missing:
        raise RuntimeError("cannot record DESCEND; missing " + ", ".join(missing))
    config = {
        "joint_names": [f"Joint{index}" for index in range(6)],
        "positions_rad": [float(measured[f"Joint{index}"]) for index in range(6)],
        "maximum_start_delta_rad": math.radians(30.0),
        "minimum_gripper_opening_m": 0.025,
        "speed_deg_s": 15.0,
        "action_name": "/arm_controller/execute_joint_segment",
    }
    path = active_session(root) / "validated_descend.yaml"
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    return path


def main(argv=None) -> int:
    options, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    if options.session_target == "pregrasp" and options.config:
        config_path = Path(options.config)
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    elif options.session_root:
        try:
            config = load_session_config(
                Path(options.session_root).expanduser(), options.session_target
            )
        except (KeyError, OSError, RuntimeError, TypeError) as exception:
            print(f"ERROR: {exception}", file=sys.stderr)
            return 2
    else:
        print("ERROR: calibration target configuration is unavailable", file=sys.stderr)
        return 2
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
        if options.record_current_as_descend:
            if not options.session_root or options.session_target != "pregrasp":
                print(
                    "ERROR: --record-current-as-descend requires a cube session "
                    "PREGRASP return",
                    file=sys.stderr,
                )
                return 2
            try:
                path = record_descend(
                    Path(options.session_root).expanduser(), node.latest_joint_state
                )
            except (OSError, RuntimeError) as exception:
                print(f"ERROR: {exception}", file=sys.stderr)
                return 2
            print(f"Recorded physically validated DESCEND: {path}")
        target_label = (
            "DESCEND" if options.session_target == "descend" else "PREGRASP"
        )
        print(f"Moving to the recorded calibration {target_label}...")
        succeeded, detail = node.execute(options.motion_timeout)
        if not succeeded:
            print(f"ERROR: {detail}", file=sys.stderr)
            return 1
        print(f"CALIBRATION {target_label} REACHED: {detail}")
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
