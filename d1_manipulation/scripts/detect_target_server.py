#!/usr/bin/env python3
"""Adapt synchronized ROS wrist RGB-D frames to perception_runtime_v1."""

from __future__ import annotations

import array
from collections import deque
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from d1_manipulation.srv import DetectTarget
from d1_perception_adapter import (
    image_to_bgr,
    match_target_detection,
    project_plumb_bob,
    ros_depth_to_meters,
    transform_point,
)


class DetectTargetServer(Node):
    def __init__(self) -> None:
        super().__init__("d1_detect_target")
        self._callback_group = ReentrantCallbackGroup()
        self._service_name = self.declare_parameter(
            "service_name", "/arm/perception/detect_target"
        ).value
        self._color_topic = self.declare_parameter(
            "color_topic", "/wrist_camera/color/image_raw"
        ).value
        self._depth_topic = self.declare_parameter(
            "aligned_depth_topic",
            "/wrist_camera/aligned_depth_to_color/image_raw",
        ).value
        self._camera_info_topic = self.declare_parameter(
            "aligned_camera_info_topic",
            "/wrist_camera/aligned_depth_to_color/camera_info",
        ).value
        self._planning_frame = self.declare_parameter(
            "planning_frame", "base_link"
        ).value
        self._runtime_root = Path(
            self.declare_parameter("perception_runtime_root", "").value
        ).expanduser()
        self._profile = self.declare_parameter("profile", "arm").value
        self._frame_timeout_s = float(
            self.declare_parameter("fresh_frame_timeout_s", 3.0).value
        )
        self._tf_timeout_s = float(
            self.declare_parameter("tf_timeout_s", 1.0).value
        )
        self._sync_tolerance_s = float(
            self.declare_parameter("sync_tolerance_s", 0.02).value
        )
        self._max_mask_distance_px = float(
            self.declare_parameter("max_hint_mask_distance_px", 120.0).value
        )
        self._depth_scale = float(
            self.declare_parameter("depth_scale_m_per_unit", 0.001).value
        )
        self._debug_topic = self.declare_parameter(
            "debug_overlay_topic", "/arm/perception/debug/overlay"
        ).value
        if not self._runtime_root.is_dir():
            raise RuntimeError(
                f"perception_runtime_root is unavailable: {self._runtime_root}"
            )
        sys.path.insert(0, str(self._runtime_root))
        os.environ.setdefault(
            "YOLO_CONFIG_DIR", "/tmp/d1_perception_ultralytics"
        )
        os.environ.setdefault("MPLCONFIGDIR", "/tmp/d1_perception_matplotlib")
        from perception_runtime import CameraIntrinsics, PerceptionRuntime

        self._CameraIntrinsics = CameraIntrinsics
        self._runtime = PerceptionRuntime(
            self._runtime_root / "configs" / "runtime.yaml",
            profile=str(self._profile),
        )

        self._condition = threading.Condition()
        self._color_frames: deque[tuple[float, Image]] = deque(maxlen=8)
        self._depth_frames: deque[tuple[float, Image]] = deque(maxlen=8)
        self._camera_info: CameraInfo | None = None
        self._request_lock = threading.Lock()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Image,
            self._color_topic,
            self._on_color,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Image,
            self._depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        debug_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._debug_publisher = self.create_publisher(
            Image, self._debug_topic, debug_qos
        )
        self._service = self.create_service(
            DetectTarget,
            self._service_name,
            self._detect,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "DetectTarget ready: service=%s device=%s RGB=%s aligned_depth=%s"
            % (
                self._service_name,
                self._runtime.segmenter.device,
                self._color_topic,
                self._depth_topic,
            )
        )

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        return float(message.header.stamp.sec) + 1e-9 * message.header.stamp.nanosec

    def _on_color(self, message: Image) -> None:
        with self._condition:
            self._color_frames.append((time.monotonic(), message))
            self._condition.notify_all()

    def _on_depth(self, message: Image) -> None:
        with self._condition:
            self._depth_frames.append((time.monotonic(), message))
            self._condition.notify_all()

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._condition:
            self._camera_info = message
            self._condition.notify_all()

    def _wait_for_fresh_pair(self) -> tuple[Image, Image, CameraInfo]:
        started = time.monotonic()
        deadline = started + self._frame_timeout_s
        with self._condition:
            self._color_frames.clear()
            self._depth_frames.clear()
            while time.monotonic() < deadline:
                if self._camera_info is not None:
                    best: tuple[float, Image, Image] | None = None
                    for color_arrival, color in self._color_frames:
                        if color_arrival < started:
                            continue
                        for depth_arrival, depth in self._depth_frames:
                            if depth_arrival < started:
                                continue
                            delta = abs(
                                self._stamp_seconds(color)
                                - self._stamp_seconds(depth)
                            )
                            if delta <= self._sync_tolerance_s and (
                                best is None or delta < best[0]
                            ):
                                best = (delta, color, depth)
                    if best is not None:
                        return best[1], best[2], self._camera_info
                remaining = max(0.0, deadline - time.monotonic())
                self._condition.wait(timeout=min(0.05, remaining))
        raise TimeoutError("fresh synchronized wrist RGB-D frame is unavailable")

    def _lookup_transform(self, target_frame: str, source_frame: str):
        return self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=self._tf_timeout_s),
        )

    @staticmethod
    def _apply_transform(point: np.ndarray, transform) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return transform_point(
            point,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _point_in_frame(self, point: PointStamped, target_frame: str) -> np.ndarray:
        value = np.array([point.point.x, point.point.y, point.point.z], dtype=np.float64)
        if not np.isfinite(value).all():
            raise ValueError("target hint contains non-finite coordinates")
        if not point.header.frame_id:
            raise ValueError("target hint frame_id is empty")
        if point.header.frame_id == target_frame:
            return value
        transform = self._lookup_transform(target_frame, point.header.frame_id)
        return self._apply_transform(value, transform)

    @staticmethod
    def _set_point(message: PointStamped, frame: str, stamp, values: np.ndarray) -> None:
        message.header.frame_id = frame
        message.header.stamp = stamp
        message.point.x = float(values[0])
        message.point.y = float(values[1])
        message.point.z = float(values[2])

    def _publish_overlay(self, color_bgr: np.ndarray, detections, matched, header) -> None:
        overlay = color_bgr.copy()
        if matched is not None:
            selected = matched.mask.astype(bool)
            magenta_bgr = np.array([255, 0, 255], dtype=np.float32)
            overlay[selected] = np.rint(
                0.60 * overlay[selected].astype(np.float32)
                + 0.40 * magenta_bgr
            ).astype(np.uint8)
        for detection in detections:
            contours, _ = cv2.findContours(
                detection.mask.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            color = (255, 0, 255) if detection is matched else (0, 180, 255)
            cv2.drawContours(overlay, contours, -1, color, 2)
            x1, y1, _, _ = detection.bbox_xyxy
            cv2.putText(
                overlay,
                f"{detection.class_name} {detection.confidence:.2f} {detection.status}",
                (x1, max(18, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        output = Image()
        output.header = header
        output.height, output.width = overlay.shape[:2]
        output.encoding = "rgb8"
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = array.array("B", overlay[..., ::-1].tobytes())
        self._debug_publisher.publish(output)

    def _fail(self, response, reason: int, detail: str):
        response.success = False
        response.failure_reason = reason
        response.detail = detail
        self.get_logger().warning(detail)
        return response

    def _detect(self, request, response):
        if not self._request_lock.acquire(blocking=False):
            return self._fail(
                response,
                DetectTarget.Response.FAILURE_INTERNAL,
                "another perception request is already running",
            )
        try:
            color_message, depth_message, camera_info = self._wait_for_fresh_pair()
            color_bgr = image_to_bgr(
                bytes(color_message.data),
                color_message.height,
                color_message.width,
                color_message.step,
                color_message.encoding,
            )
            depth_m = ros_depth_to_meters(
                bytes(depth_message.data),
                depth_message.height,
                depth_message.width,
                depth_message.step,
                depth_message.encoding,
                bool(depth_message.is_bigendian),
                self._depth_scale,
            )
            if color_bgr.shape[:2] != depth_m.shape:
                raise ValueError(
                    f"RGB and aligned depth shapes differ: {color_bgr.shape[:2]} vs {depth_m.shape}"
                )
            optical_frame = camera_info.header.frame_id
            if not optical_frame:
                raise ValueError("aligned CameraInfo frame_id is empty")
            hint_camera = self._point_in_frame(request.target_hint, optical_frame)
            hint_pixel = project_plumb_bob(hint_camera, camera_info.k, camera_info.d)
            intrinsics = self._CameraIntrinsics(
                fx=float(camera_info.k[0]),
                fy=float(camera_info.k[4]),
                cx=float(camera_info.k[2]),
                cy=float(camera_info.k[5]),
                width=int(camera_info.width),
                height=int(camera_info.height),
            )
            detections = self._runtime.process(color_bgr, depth_m, intrinsics)
            match = match_target_detection(
                detections,
                hint_pixel,
                hint_camera,
                self._max_mask_distance_px,
            )
            if match is None:
                self._publish_overlay(color_bgr, detections, None, color_message.header)
                return self._fail(
                    response,
                    DetectTarget.Response.FAILURE_NO_MATCHING_DETECTION,
                    "no depth-verified detection matches the projected target hint",
                )
            detection = match.detection
            position_camera = np.asarray(detection.position_camera_m, dtype=np.float64)
            base_transform = self._lookup_transform(
                self._planning_frame, optical_frame
            )
            position_base = self._apply_transform(position_camera, base_transform)
            response.success = True
            response.failure_reason = DetectTarget.Response.FAILURE_NONE
            response.detail = "fresh RGB-D target observation matched"
            response.class_name = detection.class_name
            response.confidence = float(detection.confidence)
            response.perception_status = detection.status
            mask_y, mask_x = np.nonzero(detection.mask)
            response.center_u = int(np.median(mask_x))
            response.center_v = int(np.median(mask_y))
            response.hint_mask_distance_px = float(match.mask_distance_px)
            response.hint_position_distance_m = float(match.position_distance_m)
            self._set_point(
                response.position_camera,
                optical_frame,
                color_message.header.stamp,
                position_camera,
            )
            self._set_point(
                response.position_base,
                self._planning_frame,
                color_message.header.stamp,
                position_base,
            )
            self._publish_overlay(
                color_bgr, detections, detection, color_message.header
            )
            self.get_logger().info(
                "Matched %s conf=%.3f pixel=(%d,%d) mask_distance=%.1fpx position_distance=%.3fm"
                % (
                    detection.class_name,
                    detection.confidence,
                    response.center_u,
                    response.center_v,
                    match.mask_distance_px,
                    match.position_distance_m,
                )
            )
            return response
        except (TimeoutError, ValueError, TransformException) as exception:
            return self._fail(
                response,
                DetectTarget.Response.FAILURE_INCOMPLETE_INFORMATION,
                str(exception),
            )
        except Exception as exception:  # Keep the ROS service alive after inference errors.
            self.get_logger().error(f"DetectTarget internal error: {exception}")
            return self._fail(
                response,
                DetectTarget.Response.FAILURE_INTERNAL,
                str(exception),
            )
        finally:
            self._request_lock.release()


def main() -> None:
    rclpy.init()
    node = DetectTargetServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
