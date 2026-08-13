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
import json

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PointStamped
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
from d1_manipulation.srv import EstimateCube
from d1_manipulation.srv import VerifyHeldObject
from d1_perception_adapter import (
    image_to_bgr,
    fit_ground_plane_ransac,
    fit_square_on_plane,
    intersect_rays_with_plane,
    match_target_detection,
    project_plumb_bob,
    ros_depth_to_meters,
    transform_point,
    transform_rotation,
    undistorted_rays,
)


class DetectTargetServer(Node):
    def __init__(self) -> None:
        super().__init__("d1_detect_target")
        self._callback_group = ReentrantCallbackGroup()
        self._service_name = self.declare_parameter(
            "service_name", "/arm/perception/detect_target"
        ).value
        self._cube_service_name = self.declare_parameter(
            "cube_estimation_service_name", "/arm/perception/estimate_cube"
        ).value
        self._held_verify_service_name = self.declare_parameter(
            "held_object_verification_service_name",
            "/arm/perception/verify_held_object",
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
        self._gravity_frame = self.declare_parameter(
            "gravity_frame", "world"
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
        self._cube_debug_topic = self.declare_parameter(
            "cube_debug_overlay_topic", "/arm/perception/debug/cube_geometry"
        ).value
        self._ground_distance = float(
            self.declare_parameter("ground_ransac_distance_m", 0.008).value
        )
        self._ground_normal_tolerance = float(
            self.declare_parameter("ground_normal_tolerance_deg", 15.0).value
        )
        self._ground_iterations = int(
            self.declare_parameter("ground_ransac_iterations", 160).value
        )
        self._ground_max_points = int(
            self.declare_parameter("ground_max_points", 12000).value
        )
        self._cube_half_size = float(
            self.declare_parameter("cube_half_size_m", 0.025).value
        )
        self._cube_debug_directory = Path(
            self.declare_parameter("cube_debug_directory", "/tmp/d1_cube_debug_latest").value
        )
        self._held_verify_frames = int(
            self.declare_parameter("held_verify_frames", 3).value
        )
        self._held_verify_roi_radius = int(
            self.declare_parameter("held_verify_roi_radius_px", 140).value
        )
        self._held_verify_min_area = int(
            self.declare_parameter("held_verify_min_area_px", 500).value
        )
        self._yellow_hsv_lower = np.asarray(
            self.declare_parameter("yellow_hsv_lower", [18, 70, 60]).value,
            dtype=np.uint8,
        )
        self._yellow_hsv_upper = np.asarray(
            self.declare_parameter("yellow_hsv_upper", [40, 255, 255]).value,
            dtype=np.uint8,
        )
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
        self._cube_debug_publisher = self.create_publisher(
            Image, self._cube_debug_topic, debug_qos
        )
        self._service = self.create_service(
            DetectTarget,
            self._service_name,
            self._detect,
            callback_group=self._callback_group,
        )
        self._cube_service = self.create_service(
            EstimateCube,
            self._cube_service_name,
            self._estimate_cube,
            callback_group=self._callback_group,
        )
        self._held_verify_service = self.create_service(
            VerifyHeldObject,
            self._held_verify_service_name,
            self._verify_held_object,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "DetectTarget ready: service=%s cube_service=%s device=%s RGB=%s aligned_depth=%s"
            % (
                self._service_name,
                self._cube_service_name,
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

    def _wait_for_fresh_color(self) -> tuple[Image, CameraInfo]:
        started = time.monotonic()
        deadline = started + self._frame_timeout_s
        with self._condition:
            self._color_frames.clear()
            while time.monotonic() < deadline:
                if self._camera_info is not None:
                    for arrival, color in reversed(self._color_frames):
                        if arrival >= started:
                            return color, self._camera_info
                remaining = max(0.0, deadline - time.monotonic())
                self._condition.wait(timeout=min(0.05, remaining))
        raise TimeoutError("fresh wrist RGB frame is unavailable")

    def _verify_held_object(self, request, response):
        if not self._request_lock.acquire(blocking=False):
            response.detail = "another perception request is already running"
            return response
        try:
            if request.class_name != "yellow_cube":
                raise ValueError(
                    f"color verification is not configured for {request.class_name}"
                )
            areas: list[float] = []
            for _ in range(self._held_verify_frames):
                color_message, camera_info = self._wait_for_fresh_color()
                color_bgr = image_to_bgr(
                    bytes(color_message.data), color_message.height,
                    color_message.width, color_message.step,
                    color_message.encoding,
                )
                optical_frame = camera_info.header.frame_id
                expected_camera = self._point_in_frame(
                    request.expected_center, optical_frame
                )
                expected_pixel = project_plumb_bob(
                    expected_camera, camera_info.k, camera_info.d
                )
                u, v = np.rint(expected_pixel).astype(int)
                height, width = color_bgr.shape[:2]
                if u < 0 or u >= width or v < 0 or v >= height:
                    raise ValueError(
                        "expected held-object centre projects outside RGB image"
                    )
                yy, xx = np.ogrid[:height, :width]
                roi = (
                    (xx - u) ** 2 + (yy - v) ** 2
                    <= self._held_verify_roi_radius ** 2
                )
                hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(
                    hsv, self._yellow_hsv_lower, self._yellow_hsv_upper
                )
                mask[~roi] = 0
                kernel = np.ones((5, 5), dtype=np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
                candidates = [float(stats[label, cv2.CC_STAT_AREA])
                              for label in range(1, count)]
                area = max(candidates, default=0.0)
                areas.append(area)
                debug = color_bgr.copy()
                cv2.circle(debug, (u, v), self._held_verify_roi_radius,
                           (255, 0, 255), 3)
                cv2.drawMarker(debug, (u, v), (255, 0, 255),
                               cv2.MARKER_CROSS, 24, 3)
                cv2.putText(debug, f"yellow area={int(area)} px", (20, 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2,
                            cv2.LINE_AA)
                self._cube_debug_directory.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(
                    str(self._cube_debug_directory /
                        f"held_verify_{len(areas)}.png"), debug
                )
            response.success = True
            response.mean_area_px = float(np.mean(areas))
            response.held = all(
                area >= self._held_verify_min_area for area in areas
            )
            response.detail = (
                "yellow ROI areas="
                + ",".join(str(int(area)) for area in areas)
                + " px"
            )
            self.get_logger().info(
                f"Held-object color verification: held={response.held} "
                f"{response.detail}"
            )
            return response
        except (TimeoutError, ValueError, TransformException) as exception:
            response.detail = str(exception)
            return response
        except Exception as exception:
            self.get_logger().error(
                f"Held-object verification error: {exception}"
            )
            response.detail = str(exception)
            return response
        finally:
            self._request_lock.release()

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

    @staticmethod
    def _to_ros_image(image_bgr: np.ndarray, header) -> Image:
        output = Image()
        output.header = header
        output.height, output.width = image_bgr.shape[:2]
        output.encoding = "rgb8"
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = array.array("B", image_bgr[..., ::-1].tobytes())
        return output

    @staticmethod
    def _transform_arrays(transform) -> tuple[np.ndarray, np.ndarray]:
        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        rotation = transform_rotation((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return rotation, np.array([translation.x, translation.y, translation.z], dtype=np.float64)

    def _runtime_frame(self):
        color_message, depth_message, camera_info = self._wait_for_fresh_pair()
        color_bgr = image_to_bgr(
            bytes(color_message.data), color_message.height, color_message.width,
            color_message.step, color_message.encoding,
        )
        depth_m = ros_depth_to_meters(
            bytes(depth_message.data), depth_message.height, depth_message.width,
            depth_message.step, depth_message.encoding,
            bool(depth_message.is_bigendian), self._depth_scale,
        )
        if color_bgr.shape[:2] != depth_m.shape:
            raise ValueError(
                f"RGB and aligned depth shapes differ: {color_bgr.shape[:2]} vs {depth_m.shape}"
            )
        optical_frame = camera_info.header.frame_id
        if not optical_frame:
            raise ValueError("aligned CameraInfo frame_id is empty")
        intrinsics = self._CameraIntrinsics(
            fx=float(camera_info.k[0]), fy=float(camera_info.k[4]),
            cx=float(camera_info.k[2]), cy=float(camera_info.k[5]),
            width=int(camera_info.width), height=int(camera_info.height),
        )
        detections = self._runtime.process(color_bgr, depth_m, intrinsics)
        return color_message, color_bgr, depth_m, camera_info, optical_frame, detections

    def _match(self, hint, optical_frame, camera_info, detections):
        hint_camera = self._point_in_frame(hint, optical_frame)
        hint_pixel = project_plumb_bob(hint_camera, camera_info.k, camera_info.d)
        match = match_target_detection(
            detections, hint_pixel, hint_camera, self._max_mask_distance_px
        )
        if match is None:
            raise LookupError("no depth-verified detection matches the projected target hint")
        return match

    def _estimate_cube(self, request, response):
        if not self._request_lock.acquire(blocking=False):
            return self._cube_fail(response, EstimateCube.Response.FAILURE_INTERNAL,
                                   "another perception request is already running")
        try:
            (color_message, color_bgr, depth_m, camera_info,
             optical_frame, detections) = self._runtime_frame()
            match = self._match(request.target_hint, optical_frame, camera_info, detections)
            detection = match.detection
            if detection.class_name != "yellow_cube":
                raise LookupError(f"matched object is {detection.class_name}, not yellow_cube")
            planning_transform = self._lookup_transform(self._planning_frame, optical_frame)
            rotation, camera_origin = self._transform_arrays(planning_transform)

            if request.stage == EstimateCube.Request.COARSE:
                world_transform = self._lookup_transform(
                    self._planning_frame, self._gravity_frame
                )
                world_rotation, _ = self._transform_arrays(world_transform)
                gravity_up = world_rotation[:, 2]
                rows, columns = np.nonzero(np.isfinite(depth_m) & (depth_m > 0.0))
                if len(columns) > self._ground_max_points:
                    rng = np.random.default_rng(0)
                    indices = rng.choice(len(columns), self._ground_max_points, replace=False)
                    rows, columns = rows[indices], columns[indices]
                pixels = np.column_stack((columns, rows))
                rays_camera = undistorted_rays(pixels, camera_info.k, camera_info.d)
                points_camera = rays_camera * (depth_m[rows, columns] / rays_camera[:, 2])[:, None]
                points_planning = points_camera @ rotation.T + camera_origin
                normal, offset, _ = fit_ground_plane_ransac(
                    points_planning, gravity_up, self._ground_distance,
                    self._ground_normal_tolerance, self._ground_iterations, 0,
                )
                mask_y, mask_x = np.nonzero(detection.mask)
                center_pixel = np.array([[np.median(mask_x), np.median(mask_y)]])
                center_ray = undistorted_rays(center_pixel, camera_info.k, camera_info.d) @ rotation.T
                center = intersect_rays_with_plane(
                    camera_origin, center_ray, normal, offset - self._cube_half_size
                )[0]
                edge = np.zeros(3)
                corners = np.empty((0, 3))
                detail = "ground fitted and mask-centre ray intersected with cube mid-plane"
            elif request.stage == EstimateCube.Request.FINE:
                normal = np.array([
                    request.ground_normal.x, request.ground_normal.y,
                    request.ground_normal.z,
                ], dtype=np.float64)
                if not np.isfinite(normal).all() or np.linalg.norm(normal) < 0.9:
                    raise ValueError("fine estimate requires a valid ground normal")
                normal /= np.linalg.norm(normal)
                offset = float(request.ground_offset)
                contours, _ = cv2.findContours(
                    detection.mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_NONE,
                )
                if not contours:
                    raise ValueError("yellow_cube mask has no contour")
                contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
                if len(contour) < 12:
                    raise ValueError("yellow_cube contour is too small")
                rays_planning = undistorted_rays(contour, camera_info.k, camera_info.d) @ rotation.T
                top_points = intersect_rays_with_plane(
                    camera_origin, rays_planning, normal,
                    offset - 2.0 * self._cube_half_size,
                )
                top_center, edge, corners = fit_square_on_plane(top_points, normal)
                center = top_center - self._cube_half_size * normal
                detail = "mask contour projected to known top plane and fitted with minAreaRect"
                debug = color_bgr.copy()
                cv2.drawContours(debug, [contour.astype(np.int32)], -1, (255, 0, 255), 2)
                cv2.putText(debug, "yellow_cube fine geometry", (20, 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2, cv2.LINE_AA)
                self._cube_debug_publisher.publish(self._to_ros_image(debug, color_message.header))
            else:
                raise ValueError(f"unsupported cube estimation stage {request.stage}")

            response.success = True
            response.failure_reason = EstimateCube.Response.FAILURE_NONE
            response.detail = detail
            response.class_name = detection.class_name
            response.confidence = float(detection.confidence)
            self._set_point(response.center, self._planning_frame,
                            color_message.header.stamp, center)
            response.ground_normal.x, response.ground_normal.y, response.ground_normal.z = map(float, normal)
            response.ground_offset = float(offset)
            response.edge_direction.x, response.edge_direction.y, response.edge_direction.z = map(float, edge)
            response.top_polygon.header = response.center.header
            for corner in corners:
                point = Point32()
                point.x, point.y, point.z = map(float, corner)
                response.top_polygon.polygon.points.append(point)
            self._publish_overlay(color_bgr, detections, detection, color_message.header)
            self._cube_debug_directory.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(
                str(self._cube_debug_directory / ("coarse.png" if request.stage == 0 else "fine.png")),
                color_bgr if request.stage == 0 else debug,
            )
            geometry = {
                "stage": "coarse" if request.stage == 0 else "fine",
                "class_name": detection.class_name,
                "confidence": float(detection.confidence),
                "center_base_m": center.tolist(),
                "ground_normal_base": normal.tolist(),
                "ground_offset_m": float(offset),
                "edge_direction_base": edge.tolist(),
                "top_corners_base_m": corners.tolist(),
            }
            (self._cube_debug_directory / ("coarse.json" if request.stage == 0 else "fine.json")).write_text(
                json.dumps(geometry, indent=2), encoding="utf-8"
            )
            self.get_logger().info(
                "Cube %s estimate: center=(%.3f, %.3f, %.3f) normal=(%.3f, %.3f, %.3f)"
                % ("coarse" if request.stage == 0 else "fine", *center, *normal)
            )
            return response
        except LookupError as exception:
            return self._cube_fail(response, EstimateCube.Response.FAILURE_NO_MATCHING_DETECTION, str(exception))
        except (TimeoutError, ValueError, TransformException) as exception:
            return self._cube_fail(response, EstimateCube.Response.FAILURE_INCOMPLETE_INFORMATION, str(exception))
        except Exception as exception:
            self.get_logger().error(f"EstimateCube internal error: {exception}")
            return self._cube_fail(response, EstimateCube.Response.FAILURE_INTERNAL, str(exception))
        finally:
            self._request_lock.release()

    def _cube_fail(self, response, reason: int, detail: str):
        response.success = False
        response.failure_reason = reason
        response.detail = detail
        self.get_logger().warning(detail)
        return response

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
