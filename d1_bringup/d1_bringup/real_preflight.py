import math
import socket
import statistics
import struct
import sys
import time
from collections import deque
from pathlib import Path

import yaml

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu, JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


PACKET = struct.Struct("<IHHQII7d")
PACKET_MAGIC = 0x44314350
PACKET_VERSION = 1
PACKET_FEEDBACK = 2
PACKET_STATUS = 3
DEG_TO_RAD = math.pi / 180.0


def _normalize(vector):
    norm = math.sqrt(sum(value * value for value in vector))
    if norm < 1e-9:
        raise ValueError("zero-length vector")
    return tuple(value / norm for value in vector)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _matrix_to_quaternion(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (matrix[2][1] - matrix[1][2]) / s,
            (matrix[0][2] - matrix[2][0]) / s,
            (matrix[1][0] - matrix[0][1]) / s,
            0.25 * s,
        )
    diagonal = [matrix[0][0], matrix[1][1], matrix[2][2]]
    index = diagonal.index(max(diagonal))
    if index == 0:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        return (0.25 * s, (matrix[0][1] + matrix[1][0]) / s,
                (matrix[0][2] + matrix[2][0]) / s, (matrix[2][1] - matrix[1][2]) / s)
    if index == 1:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        return ((matrix[0][1] + matrix[1][0]) / s, 0.25 * s,
                (matrix[1][2] + matrix[2][1]) / s, (matrix[0][2] - matrix[2][0]) / s)
    s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
    return ((matrix[0][2] + matrix[2][0]) / s,
            (matrix[1][2] + matrix[2][1]) / s, 0.25 * s,
            (matrix[1][0] - matrix[0][1]) / s)


def _rotate_by_quaternion(vector, quaternion):
    x, y, z, w = quaternion
    qvec = (x, y, z)
    uv = _cross(qvec, vector)
    uuv = _cross(qvec, uv)
    return tuple(vector[i] + 2.0 * (w * uv[i] + uuv[i]) for i in range(3))


class RealPreflight(Node):
    def __init__(self):
        super().__init__("d1_real_preflight")
        defaults = {
            "network_interface": "enp3s0",
            "expected_ipv4_subnet": "192.168.123.0/24",
            "feedback_port": 15101,
            "gripper_closed_angle_deg": -30.0,
            "gripper_open_angle_deg": 60.0,
            "gripper_travel_m": 0.03,
            "color_camera_info_topic": "/wrist_camera/color/camera_info",
            "aligned_depth_camera_info_topic": "/wrist_camera/aligned_depth_to_color/camera_info",
            "imu_topic": "/wrist_camera/imu",
            "gravity_source": "realsense_accel",
            "gravity_parent_frame": "base_link",
            "gravity_frame": "gravity_frame",
            "expected_acceleration_mps2": 9.80665,
            "acceleration_norm_tolerance_mps2": 1.5,
            "acceleration_max_component_stddev_mps2": 0.35,
            "gravity_required_samples": 30,
            "gravity_output_path": "",
            "timeout_s": 20.0,
            "minimum_arm_feedback_samples": 5,
            "feedback_max_age_s": 0.5,
            "camera_info_max_age_s": 2.0,
            "joint_limit_tolerance_rad": 0.001,
            "joint_limits": [
                -2.785545486183, 2.809980095711,
                -1.541125729511, 1.619665545851,
                -1.619665545851, 1.546361717267,
                -2.623229865747, 2.654645792283,
                -1.610938899591, 1.848303677862,
                -2.764601535159, 2.778564169175,
                0.0, 0.03,
            ],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.started = time.monotonic()
        self.last_feedback = None
        self.feedback_count = 0
        self.latest_positions = None
        self.latest_arm_status = None
        self.last_arm_status = None
        self.color_info = None
        self.color_info_time = None
        self.depth_info = None
        self.depth_info_time = None
        self.imu_frame = None
        self.acceleration_samples = deque(
            maxlen=int(self.get_parameter("gravity_required_samples").value)
        )
        self.gravity_published = False
        self.finished = False
        self.exit_code = 2

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind(("127.0.0.1", int(self.get_parameter("feedback_port").value)))

        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("color_camera_info_topic").value),
            self._on_color_info,
            10,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("aligned_depth_camera_info_topic").value),
            self._on_depth_info,
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(0.01, self._receive_feedback)
        self.create_timer(0.1, self._evaluate)
        self.get_logger().warning(
            "MOTIONLESS PREFLIGHT ONLY: ros2_control is not started and no arm command is sent"
        )

    def _on_color_info(self, message):
        self.color_info = message
        self.color_info_time = time.monotonic()

    def _on_depth_info(self, message):
        self.depth_info = message
        self.depth_info_time = time.monotonic()

    def _on_imu(self, message):
        vector = message.linear_acceleration
        sample = (vector.x, vector.y, vector.z)
        if all(math.isfinite(value) for value in sample):
            self.acceleration_samples.append(sample)
            self.imu_frame = message.header.frame_id

    def _receive_feedback(self):
        while True:
            try:
                data, _ = self.socket.recvfrom(PACKET.size)
            except BlockingIOError:
                return
            if len(data) != PACKET.size:
                continue
            unpacked = PACKET.unpack(data)
            if unpacked[0:2] != (PACKET_MAGIC, PACKET_VERSION):
                continue
            if unpacked[2] == PACKET_STATUS:
                self.latest_arm_status = tuple(int(value) for value in unpacked[6:9])
                self.last_arm_status = time.monotonic()
                continue
            if unpacked[2] != PACKET_FEEDBACK:
                continue
            angles_deg = unpacked[6:13]
            if not all(math.isfinite(value) for value in angles_deg):
                continue
            positions = [value * DEG_TO_RAD for value in angles_deg[:6]]
            closed = float(self.get_parameter("gripper_closed_angle_deg").value)
            opened = float(self.get_parameter("gripper_open_angle_deg").value)
            travel = float(self.get_parameter("gripper_travel_m").value)
            ratio = max(0.0, min(1.0, (angles_deg[6] - closed) / (opened - closed)))
            positions.append(travel * ratio)
            self.latest_positions = positions
            self.last_feedback = time.monotonic()
            self.feedback_count += 1
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [f"Joint{index}" for index in range(7)]
            message.position = positions
            self.joint_publisher.publish(message)

    def _gravity_result(self):
        if self.get_parameter("gravity_source").value != "realsense_accel":
            return False, "unsupported gravity source"
        required = int(self.get_parameter("gravity_required_samples").value)
        if len(self.acceleration_samples) < required:
            return False, f"IMU samples {len(self.acceleration_samples)}/{required}"
        means = tuple(statistics.fmean(sample[i] for sample in self.acceleration_samples) for i in range(3))
        stddev = tuple(statistics.pstdev(sample[i] for sample in self.acceleration_samples) for i in range(3))
        norm = math.sqrt(sum(value * value for value in means))
        expected = float(self.get_parameter("expected_acceleration_mps2").value)
        tolerance = float(self.get_parameter("acceleration_norm_tolerance_mps2").value)
        max_stddev = float(self.get_parameter("acceleration_max_component_stddev_mps2").value)
        if abs(norm - expected) > tolerance:
            return False, f"acceleration norm {norm:.3f} m/s^2 outside {expected:.3f}+/-{tolerance:.3f}"
        if max(stddev) > max_stddev:
            return False, f"arm/camera moving: acceleration stddev {max(stddev):.3f} m/s^2"
        parent = str(self.get_parameter("gravity_parent_frame").value)
        if not self.imu_frame:
            return False, "IMU frame_id is empty"
        try:
            transform = self.tf_buffer.lookup_transform(parent, self.imu_frame, rclpy.time.Time())
        except Exception as error:
            return False, f"missing TF {parent} <- {self.imu_frame}: {error}"
        rotation = transform.transform.rotation
        up = _normalize(_rotate_by_quaternion(means, (rotation.x, rotation.y, rotation.z, rotation.w)))
        base_x = (1.0, 0.0, 0.0)
        projection = tuple(base_x[i] - sum(base_x[j] * up[j] for j in range(3)) * up[i] for i in range(3))
        if math.sqrt(sum(value * value for value in projection)) < 1e-3:
            projection = _cross((0.0, 1.0, 0.0), up)
        x_axis = _normalize(projection)
        y_axis = _normalize(_cross(up, x_axis))
        matrix = (
            (x_axis[0], y_axis[0], up[0]),
            (x_axis[1], y_axis[1], up[1]),
            (x_axis[2], y_axis[2], up[2]),
        )
        quaternion = _matrix_to_quaternion(matrix)
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = parent
        message.child_frame_id = str(self.get_parameter("gravity_frame").value)
        message.transform.rotation.x = quaternion[0]
        message.transform.rotation.y = quaternion[1]
        message.transform.rotation.z = quaternion[2]
        message.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(message)
        output_path = str(self.get_parameter("gravity_output_path").value)
        if output_path and not self.gravity_published:
            destination = Path(output_path)
            temporary = destination.with_suffix(destination.suffix + ".tmp")
            try:
                temporary.write_text(
                    yaml.safe_dump({
                        "parent_frame": parent,
                        "child_frame": message.child_frame_id,
                        "translation_xyz_m": [0.0, 0.0, 0.0],
                        "quaternion_xyzw": list(quaternion),
                    }, sort_keys=False),
                    encoding="utf-8",
                )
                temporary.replace(destination)
            except OSError as error:
                return False, f"cannot persist gravity calibration: {error}"
        self.gravity_published = True
        return True, f"stationary acceleration norm={norm:.3f} m/s^2, up(base)={up}"

    def _network_result(self):
        interface = str(self.get_parameter("network_interface").value)
        try:
            state = open(f"/sys/class/net/{interface}/operstate", encoding="utf-8").read().strip()
        except OSError as error:
            return False, f"interface {interface} unavailable: {error}"
        return (state == "up", f"{interface} operstate={state}")

    def _joint_result(self):
        minimum = int(self.get_parameter("minimum_arm_feedback_samples").value)
        if self.feedback_count < minimum or self.latest_positions is None:
            return False, f"D1 feedback samples {self.feedback_count}/{minimum}"
        age = time.monotonic() - self.last_feedback
        if age > float(self.get_parameter("feedback_max_age_s").value):
            return False, f"D1 feedback stale ({age:.3f} s)"
        flat_limits = list(self.get_parameter("joint_limits").value)
        tolerance = float(self.get_parameter("joint_limit_tolerance_rad").value)
        violations = []
        for index, position in enumerate(self.latest_positions):
            lower, upper = flat_limits[index * 2:index * 2 + 2]
            if position < lower - tolerance or position > upper + tolerance:
                violations.append(
                    f"Joint{index}={position:.6f} outside [{lower:.6f}, {upper:.6f}] rad"
                )
        if violations:
            return False, "; ".join(violations)
        return True, "feedback fresh; all joints within configured limits"

    def _arm_status_result(self):
        if self.latest_arm_status is None:
            return False, "waiting for D1 hardware status"
        age = time.monotonic() - self.last_arm_status
        if age > float(self.get_parameter("feedback_max_age_s").value):
            return False, f"D1 hardware status stale ({age:.3f} s)"
        enable, power, error = self.latest_arm_status
        if error != 0:
            return False, f"D1 error_status={error}"
        # This phase is deliberately motionless and runs before ros2_control.
        # A disabled arm is therefore valid here: hardware activation owns the
        # idempotent power/enable requests and verifies enable_status afterward.
        if enable not in (0, 1):
            return False, f"unexpected D1 enable_status={enable}"
        # D1 power_status has proven unreliable on physical arms: a live,
        # responsive arm can continue to report zero.  Keep it in diagnostics,
        # but never use it as a readiness gate.
        return True, (
            f"enable={enable} (activation verifies enabled); error={error}; "
            f"power_status={power} (diagnostic only)"
        )

    def _camera_result(self):
        if self.color_info is None or self.depth_info is None:
            return False, "waiting for color/aligned-depth CameraInfo"
        max_age = float(self.get_parameter("camera_info_max_age_s").value)
        age = max(time.monotonic() - self.color_info_time, time.monotonic() - self.depth_info_time)
        if age > max_age:
            return False, f"CameraInfo stale ({age:.3f} s)"
        if (self.color_info.width, self.color_info.height) != (1280, 720):
            return False, f"unexpected color size {self.color_info.width}x{self.color_info.height}"
        if (self.depth_info.width, self.depth_info.height) != (1280, 720):
            return False, f"unexpected aligned-depth size {self.depth_info.width}x{self.depth_info.height}"
        return True, "color and aligned depth are live at 1280x720"

    def _evaluate(self):
        if self.finished:
            return
        results = {
            "network": self._network_result(),
            "arm_feedback_and_limits": self._joint_result(),
            "arm_hardware_status": self._arm_status_result(),
            "camera": self._camera_result(),
            "gravity": self._gravity_result(),
        }
        ready_to_finish = all(result[0] for result in results.values())
        timed_out = time.monotonic() - self.started >= float(self.get_parameter("timeout_s").value)
        definitive_joint_failure = (
            self.latest_positions is not None
            and not results["arm_feedback_and_limits"][0]
            and "outside" in results["arm_feedback_and_limits"][1]
        )
        # Once every sensor/TF check is complete, a joint-limit violation can
        # finish immediately. Otherwise wait for the deadline so one run gives
        # a comprehensive report rather than hiding later checks.
        complete_with_joint_failure = definitive_joint_failure and all(
            results[name][0]
            for name in ("network", "arm_hardware_status", "camera", "gravity")
        )
        if not (ready_to_finish or timed_out or complete_with_joint_failure):
            return
        print("\nD1 REAL-MACHINE MOTIONLESS PREFLIGHT")
        for name, (passed, detail) in results.items():
            print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")
        if ready_to_finish:
            print("PREFLIGHT PASSED: hardware is observable; controllers remain inactive.")
            self.exit_code = 0
        else:
            print("PREFLIGHT FAILED: NOT_READY; controllers were never started.")
            self.exit_code = 2
        self.finished = True

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealPreflight()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning("preflight canceled; no command was sent")
        node.exit_code = 130
    exit_code = node.exit_code
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
