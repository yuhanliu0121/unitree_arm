#!/usr/bin/env python3
"""Capture one boundary sample for the yellow-cube safe-descend region.

This tool is deliberately read-only with respect to the arm.  It calls the
existing fine cube estimator, snapshots the corresponding wrist RGB-D and
robot state, and expresses the fitted cube top centre in the RGB optical
frame.  Four invocations produce the rectangular safe-descend cross-section.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from tf2_ros import Buffer, TransformException, TransformListener

from d1_manipulation.srv import EstimateCube
from d1_perception_adapter import (
    BOUNDARY_NAMES,
    build_safe_region,
    fit_ground_plane_ransac,
    fit_square_on_plane,
    intersect_rays_with_plane,
    undistorted_rays,
)


BOUNDARY_LABELS = {
    1: BOUNDARY_NAMES[0],
    2: BOUNDARY_NAMES[1],
    3: BOUNDARY_NAMES[2],
    4: BOUNDARY_NAMES[3],
}


def _stamp_ns(stamp: TimeMessage) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _stamp_dict(stamp: TimeMessage) -> dict:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def _vector(value) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def _quaternion(value) -> list[float]:
    return [float(value.x), float(value.y), float(value.z), float(value.w)]


def _rotation_xyzw(quaternion) -> np.ndarray:
    x, y, z, w = map(float, quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("TF quaternion has zero length")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _transform_arrays(transform) -> tuple[np.ndarray, np.ndarray]:
    value = transform.transform
    rotation = _rotation_xyzw(_quaternion(value.rotation))
    translation = np.asarray(_vector(value.translation), dtype=np.float64)
    return rotation, translation


def _nearest(messages, target_ns: int):
    if not messages:
        return None, None
    message = min(messages, key=lambda item: abs(_stamp_ns(item.header.stamp) - target_ns))
    return message, abs(_stamp_ns(message.header.stamp) - target_ns) / 1e9


def _image_array(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    if encoding in ("rgb8", "bgr8"):
        channels, dtype = 3, np.uint8
    elif encoding in ("mono8", "8uc1"):
        channels, dtype = 1, np.uint8
    elif encoding in ("16uc1", "mono16"):
        channels, dtype = 1, np.dtype(">u2" if message.is_bigendian else "<u2")
    elif encoding == "32fc1":
        channels, dtype = 1, np.dtype(">f4" if message.is_bigendian else "<f4")
    else:
        raise ValueError(f"unsupported image encoding {message.encoding!r}")
    item_size = np.dtype(dtype).itemsize
    row_items = int(message.step) // item_size
    data = np.frombuffer(message.data, dtype=dtype).reshape(message.height, row_items)
    data = data[:, : message.width * channels]
    if channels > 1:
        data = data.reshape(message.height, message.width, channels)
    return np.array(data, copy=True)


def _save_image(path: Path, message: Image) -> None:
    array = _image_array(message)
    if message.encoding.lower() == "rgb8":
        array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), array):
        raise OSError(f"failed to write {path}")


def _save_depth_preview(path: Path, message: Image) -> None:
    depth = _image_array(message).astype(np.float32)
    if message.encoding.lower() == "16uc1":
        depth *= 0.001
    valid = np.isfinite(depth) & (depth >= 0.20) & (depth <= 2.0)
    normalized = np.zeros(depth.shape, dtype=np.uint8)
    normalized[valid] = np.clip((depth[valid] - 0.20) / 1.80 * 255.0, 0, 255).astype(np.uint8)
    plasma = cv2.applyColorMap(normalized, cv2.COLORMAP_PLASMA)
    plasma[~valid] = 0
    if not cv2.imwrite(str(path), plasma):
        raise OSError(f"failed to write {path}")


def _camera_info_dict(message: CameraInfo) -> dict:
    return {
        "stamp": _stamp_dict(message.header.stamp),
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "d": list(map(float, message.d)),
        "k": list(map(float, message.k)),
        "r": list(map(float, message.r)),
        "p": list(map(float, message.p)),
    }


class CaptureNode(Node):
    def __init__(self, args) -> None:
        super().__init__("capture_cube_safe_boundary")
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.messages = {
            "color": deque(maxlen=50),
            "aligned_depth": deque(maxlen=80),
            "raw_depth": deque(maxlen=80),
            "overlay": deque(maxlen=20),
            "color_info": deque(maxlen=10),
            "aligned_info": deque(maxlen=10),
            "imu": deque(maxlen=300),
            "joints": deque(maxlen=300),
        }
        self._subscribe(Image, args.color_topic, "color")
        self._subscribe(Image, args.aligned_depth_topic, "aligned_depth")
        self._subscribe(Image, args.raw_depth_topic, "raw_depth")
        self._subscribe(Image, args.overlay_topic, "overlay")
        self._subscribe(CameraInfo, args.color_info_topic, "color_info")
        self._subscribe(CameraInfo, args.aligned_info_topic, "aligned_info")
        self._subscribe(Imu, args.imu_topic, "imu")
        self._subscribe(JointState, args.joint_topic, "joints")
        self.client = self.create_client(EstimateCube, args.estimate_service)
        self.fallback_color_mask = None
        self.fallback_mask = None
        self.fallback_overlay = None

    def _subscribe(self, message_type, topic: str, key: str) -> None:
        self.create_subscription(
            message_type,
            topic,
            lambda message, name=key: self.messages[name].append(message),
            qos_profile_sensor_data,
        )

    def wait_for_inputs(self, timeout_s: float) -> None:
        required = ("color", "aligned_depth", "color_info", "imu", "joints")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(self.messages[name] for name in required):
                return
        missing = [name for name in required if not self.messages[name]]
        raise TimeoutError(f"required calibration inputs unavailable: {', '.join(missing)}")

    def collect_stationary_window(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_tf_frames(self, timeout_s: float) -> None:
        camera_frame = self.messages["color"][-1].header.frame_id
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            planning_ready = self.tf_buffer.can_transform(
                self.args.planning_frame, camera_frame, Time()
            )
            tcp_ready = self.tf_buffer.can_transform(
                camera_frame, self.args.tcp_frame, Time()
            )
            if planning_ready and tcp_ready:
                return
        raise TimeoutError(
            "TF frames did not become available: "
            f"{self.args.planning_frame} <-> {camera_frame} <-> {self.args.tcp_frame}"
        )

    def estimate(self, timeout_s: float):
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"cube estimator unavailable: {self.args.estimate_service}")
        request = EstimateCube.Request()
        request.stage = EstimateCube.Request.FINE
        request.target_hint.header.frame_id = self.args.planning_frame
        request.target_hint.header.stamp = self.get_clock().now().to_msg()
        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError("fine cube estimation timed out")
        response = future.result()
        if response is None or not response.success:
            detail = response.detail if response is not None else "no service response"
            raise RuntimeError(f"fine cube estimation failed: {detail}")
        # Let the service's debug overlay reach this process before snapshotting.
        end = time.monotonic() + 0.35
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return response

    def estimate_from_calibration_color(self):
        """Estimate geometry with a calibration-only yellow-mask fallback."""
        color_message = self.messages["color"][-1]
        stamp = color_message.header.stamp
        target_ns = _stamp_ns(stamp)
        depth_message, depth_delta = _nearest(
            self.messages["aligned_depth"], target_ns
        )
        camera_info, info_delta = _nearest(
            self.messages["color_info"], target_ns
        )
        if depth_message is None or camera_info is None:
            raise RuntimeError("calibration fallback has no matched RGB-D inputs")
        if depth_delta > self.args.sync_tolerance or info_delta > 0.5:
            raise RuntimeError(
                "calibration fallback RGB-D is not synchronized: "
                f"depth={depth_delta:.3f}s info={info_delta:.3f}s"
            )

        color = _image_array(color_message)
        color_bgr = (
            cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            if color_message.encoding.lower() == "rgb8" else color
        )
        hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.asarray(self.args.yellow_hsv_lower, dtype=np.uint8),
            np.asarray(self.args.yellow_hsv_upper, dtype=np.uint8),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        principal = np.asarray([camera_info.k[2], camera_info.k[5]])
        candidates = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.args.fallback_min_area_px:
                continue
            rows, columns = np.nonzero(labels == label)
            distance = float(np.min(np.hypot(
                columns - principal[0], rows - principal[1]
            )))
            candidates.append((distance, -area, label))
        if not candidates:
            raise RuntimeError("calibration yellow fallback found no usable component")
        _, negative_area, selected_label = min(candidates)
        selected = np.asarray(labels == selected_label, dtype=np.uint8)

        depth = _image_array(depth_message).astype(np.float64)
        if depth_message.encoding.lower() in ("16uc1", "mono16"):
            depth *= 0.001
        valid_depth = (
            np.isfinite(depth)
            & (depth >= self.args.depth_min_m)
            & (depth <= self.args.depth_max_m)
        )
        selected_pixels = selected.astype(bool)
        valid_ratio = float(np.mean(valid_depth[selected_pixels]))
        if valid_ratio < self.args.fallback_min_valid_depth_ratio:
            raise RuntimeError(
                "calibration yellow component lacks valid depth: "
                f"ratio={valid_ratio:.3f}"
            )

        camera_frame = color_message.header.frame_id
        capture_time = Time.from_msg(stamp)
        planning_from_camera = self.tf_buffer.lookup_transform(
            self.args.planning_frame, camera_frame, capture_time,
            timeout=Duration(seconds=self.args.tf_timeout),
        )
        planning_from_gravity = self.tf_buffer.lookup_transform(
            self.args.planning_frame, self.args.gravity_frame, Time(),
            timeout=Duration(seconds=self.args.tf_timeout),
        )
        rotation_pc, camera_origin = _transform_arrays(planning_from_camera)
        rotation_pg, _ = _transform_arrays(planning_from_gravity)
        gravity_up = rotation_pg[:, 2]

        rows, columns = np.nonzero(valid_depth)
        if len(columns) > self.args.ground_max_points:
            indices = np.random.default_rng(0).choice(
                len(columns), self.args.ground_max_points, replace=False
            )
            rows, columns = rows[indices], columns[indices]
        pixels = np.column_stack((columns, rows))
        rays_camera = undistorted_rays(pixels, camera_info.k, camera_info.d)
        points_camera = rays_camera * (
            depth[rows, columns] / rays_camera[:, 2]
        )[:, None]
        points_planning = points_camera @ rotation_pc.T + camera_origin
        normal, offset, _ = fit_ground_plane_ransac(
            points_planning,
            gravity_up,
            self.args.ground_distance_m,
            self.args.ground_normal_tolerance_deg,
            self.args.ground_iterations,
            0,
        )

        mask_rows, mask_columns = np.nonzero(selected_pixels & valid_depth)
        mask_pixels = np.column_stack((mask_columns, mask_rows))
        mask_rays_camera = undistorted_rays(
            mask_pixels, camera_info.k, camera_info.d
        )
        mask_points_camera = mask_rays_camera * (
            depth[mask_rows, mask_columns] / mask_rays_camera[:, 2]
        )[:, None]
        mask_points_planning = mask_points_camera @ rotation_pc.T + camera_origin
        height_above_ground = mask_points_planning @ normal + offset
        top_inliers = np.abs(
            height_above_ground - self.args.cube_size_m
        ) <= self.args.fallback_top_height_tolerance_m
        top_mask = np.zeros_like(selected)
        top_mask[mask_rows[top_inliers], mask_columns[top_inliers]] = 1
        top_mask = cv2.morphologyEx(top_mask, cv2.MORPH_CLOSE, kernel)
        top_count, top_labels, top_stats, _ = cv2.connectedComponentsWithStats(top_mask)
        if top_count < 2:
            raise RuntimeError("calibration depth filter found no cube top face")
        top_label = 1 + int(np.argmax(top_stats[1:, cv2.CC_STAT_AREA]))
        top_area = int(top_stats[top_label, cv2.CC_STAT_AREA])
        if top_area < self.args.fallback_min_top_area_px:
            raise RuntimeError(
                f"calibration cube top face is too small: area={top_area}px"
            )
        top_mask = np.asarray(top_labels == top_label, dtype=np.uint8)

        contours, _ = cv2.findContours(
            top_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
        if len(contour) < 12:
            raise RuntimeError("calibration yellow component contour is too small")
        rays_planning = (
            undistorted_rays(contour, camera_info.k, camera_info.d)
            @ rotation_pc.T
        )
        top_points = intersect_rays_with_plane(
            camera_origin,
            rays_planning,
            normal,
            offset - self.args.cube_size_m,
        )
        top_center, edge, corners = fit_square_on_plane(top_points, normal)
        center = top_center - 0.5 * self.args.cube_size_m * normal

        overlay = color_bgr.copy()
        cv2.drawContours(overlay, [contour.astype(np.int32)], -1, (255, 0, 255), 3)
        cv2.circle(overlay, tuple(np.rint(principal).astype(int)), 5, (255, 255, 255), 2)
        cv2.putText(
            overlay, "CALIBRATION HSV FALLBACK", (20, 32),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2, cv2.LINE_AA,
        )
        self.fallback_color_mask = selected * 255
        self.fallback_mask = top_mask * 255
        self.fallback_overlay = overlay

        def point(value):
            return SimpleNamespace(
                x=float(value[0]), y=float(value[1]), z=float(value[2])
            )

        header = SimpleNamespace(stamp=stamp, frame_id=self.args.planning_frame)
        return SimpleNamespace(
            success=True,
            class_name="yellow_cube",
            confidence=0.0,
            detail=(
                "calibration-only HSV fallback; ground refitted and yellow "
                f"top-face contour projected to cube top plane; "
                f"color_area={-negative_area}px top_area={top_area}px "
                f"valid_depth_ratio={valid_ratio:.3f}"
            ),
            center=SimpleNamespace(header=header, point=point(center)),
            ground_normal=point(normal),
            ground_offset=float(offset),
            edge_direction=point(edge),
            top_polygon=SimpleNamespace(
                header=header,
                polygon=SimpleNamespace(points=[point(corner) for corner in corners]),
            ),
        )


def _resolve_session(output_root: Path, boundary: int, force_new: bool):
    output_root.mkdir(parents=True, exist_ok=True)
    active_path = output_root / ".active_session"
    if boundary == 1:
        if active_path.exists() and not force_new:
            active = active_path.read_text(encoding="utf-8").strip()
            raise RuntimeError(
                f"an unfinished calibration session is active: {active}; "
                "use --new only if you intend to start another session"
            )
        name = "cube_safe_region_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        session = output_root / name
        return session, active_path, True
    if not active_path.exists():
        raise RuntimeError("no active session; capture boundary 1 first")
    session = output_root / active_path.read_text(encoding="utf-8").strip()
    if not session.is_dir():
        raise RuntimeError(f"active session directory is missing: {session}")
    return session, active_path, False


def _joint_snapshot(message: JointState) -> dict:
    positions = {name: float(value) for name, value in zip(message.name, message.position)}
    arm = [positions.get(f"Joint{index}") for index in range(6)]
    if any(value is None for value in arm):
        raise RuntimeError("/joint_states does not contain every Joint0..Joint5")
    return {
        "stamp": _stamp_dict(message.header.stamp),
        "name": list(message.name),
        "position": list(map(float, message.position)),
        "velocity": list(map(float, message.velocity)),
        "effort": list(map(float, message.effort)),
        "arm_joint_positions_rad": arm,
    }


def _stationary_spread(messages, target_ns: int, window_s: float = 1.0) -> float:
    selected = [
        message for message in messages
        if 0 <= target_ns - _stamp_ns(message.header.stamp) <= int(window_s * 1e9)
    ]
    rows = []
    for message in selected:
        values = {name: value for name, value in zip(message.name, message.position)}
        if all(f"Joint{i}" in values for i in range(6)):
            rows.append([values[f"Joint{i}"] for i in range(6)])
    if len(rows) < 2:
        return math.nan
    return float(np.degrees(np.max(np.ptp(np.asarray(rows), axis=0))))


def _capture(args) -> tuple[Path, dict]:
    output_root = Path(args.output_root).expanduser().resolve()
    session, active_path, new_session = _resolve_session(
        output_root, args.boundary, args.new
    )
    boundary_name = BOUNDARY_LABELS[args.boundary]
    sample_directory = session / f"{args.boundary}_{boundary_name}"
    if sample_directory.exists():
        raise RuntimeError(f"boundary already captured: {sample_directory}")

    node = CaptureNode(args)
    try:
        node.wait_for_inputs(args.timeout)
        node.collect_stationary_window(args.stationary_window)
        node.wait_for_tf_frames(args.tf_timeout)
        try:
            response = node.estimate(args.timeout)
        except RuntimeError as estimate_error:
            if "found no yellow_cube mask" not in str(estimate_error):
                raise
            node.get_logger().warning(
                "YOLO missed the close cube; using calibration-only yellow "
                f"fallback: {estimate_error}"
            )
            response = node.estimate_from_calibration_color()
        except TimeoutError:
            # A failed mask/geometry estimate is exactly when the raw frame is
            # most useful.  Keep it outside the session so it cannot be
            # mistaken for an accepted boundary sample.
            failure_directory = output_root / "failed_latest"
            failure_directory.mkdir(parents=True, exist_ok=True)
            if node.messages["color"]:
                _save_image(failure_directory / "rgb.png", node.messages["color"][-1])
            if node.messages["aligned_depth"]:
                depth = node.messages["aligned_depth"][-1]
                _save_image(failure_directory / "aligned_depth.png", depth)
                _save_depth_preview(failure_directory / "aligned_depth_plasma.png", depth)
            raise
        stamp = response.top_polygon.header.stamp
        target_ns = _stamp_ns(stamp)
        camera_frame = node.messages["color"][-1].header.frame_id
        planning_frame = response.top_polygon.header.frame_id or args.planning_frame
        if len(response.top_polygon.polygon.points) != 4:
            raise RuntimeError(
                f"fine estimator returned {len(response.top_polygon.polygon.points)} top corners, expected 4"
            )
        corners_planning = np.asarray(
            [_vector(point) for point in response.top_polygon.polygon.points],
            dtype=np.float64,
        )
        top_center_planning = np.mean(corners_planning, axis=0)

        capture_time = Time.from_msg(stamp)
        camera_from_planning = node.tf_buffer.lookup_transform(
            camera_frame, planning_frame, capture_time,
            timeout=Duration(seconds=args.tf_timeout),
        )
        camera_from_tcp = node.tf_buffer.lookup_transform(
            camera_frame, args.tcp_frame, capture_time,
            timeout=Duration(seconds=args.tf_timeout),
        )
        rotation_cp, translation_cp = _transform_arrays(camera_from_planning)
        rotation_ct, _ = _transform_arrays(camera_from_tcp)
        top_center_camera = rotation_cp @ top_center_planning + translation_cp
        gravity_up_camera = rotation_cp @ np.asarray(_vector(response.ground_normal))

        nearest = {}
        deltas = {}
        for key in node.messages:
            nearest[key], deltas[key] = _nearest(node.messages[key], target_ns)
        mandatory = ("color", "aligned_depth", "color_info", "imu", "joints")
        missing = [key for key in mandatory if nearest[key] is None]
        if missing:
            raise RuntimeError(f"could not match captured messages: {', '.join(missing)}")
        if deltas["color"] > args.sync_tolerance or deltas["aligned_depth"] > args.sync_tolerance:
            raise RuntimeError(
                "captured RGB-D does not match estimator stamp: "
                f"rgb={deltas['color']:.3f}s depth={deltas['aligned_depth']:.3f}s"
            )

        joint = _joint_snapshot(nearest["joints"])
        stationary_spread_deg = _stationary_spread(
            node.messages["joints"], target_ns, args.stationary_window
        )
        if math.isfinite(stationary_spread_deg) and stationary_spread_deg > args.max_stationary_deg:
            raise RuntimeError(
                f"arm moved {stationary_spread_deg:.3f} deg during the capture window; sample rejected"
            )

        imu = nearest["imu"]
        sample = {
            "schema_version": 1,
            "boundary_index": args.boundary,
            "boundary_name": boundary_name,
            "captured_at_local": datetime.now().astimezone().isoformat(),
            "camera_frame": camera_frame,
            "planning_frame": planning_frame,
            "tcp_frame": args.tcp_frame,
            "estimate_stamp": _stamp_dict(stamp),
            "class_name": response.class_name,
            "confidence": float(response.confidence),
            "estimate_detail": response.detail,
            "cube_center_planning_m": _vector(response.center.point),
            "top_center_planning_m": top_center_planning.tolist(),
            "top_corners_planning_m": corners_planning.tolist(),
            "top_center_camera_m": top_center_camera.tolist(),
            "gravity_up_planning": _vector(response.ground_normal),
            "gravity_up_camera": gravity_up_camera.tolist(),
            # tcp_link +Y joins the fingertips; +X runs along finger length.
            "closing_axis_camera": rotation_ct[:, 1].tolist(),
            "finger_axis_camera": rotation_ct[:, 0].tolist(),
            "approach_axis_camera": rotation_ct[:, 2].tolist(),
            "ground_offset_planning_m": float(response.ground_offset),
            "cube_edge_direction_planning": _vector(response.edge_direction),
            "arm_joint_positions_rad": joint["arm_joint_positions_rad"],
            "joint_state": joint,
            "imu": {
                "stamp": _stamp_dict(imu.header.stamp),
                "frame_id": imu.header.frame_id,
                "orientation_xyzw": _quaternion(imu.orientation),
                "angular_velocity": _vector(imu.angular_velocity),
                "linear_acceleration": _vector(imu.linear_acceleration),
            },
            "camera_info": _camera_info_dict(nearest["color_info"]),
            "capture_diagnostics": {
                "message_stamp_delta_s": {
                    key: (None if value is None else float(value))
                    for key, value in deltas.items()
                },
                "arm_motion_range_last_window_deg": stationary_spread_deg,
            },
        }

        manifest_path = session / "session.json"
        manifest = {
            "schema_version": 1,
            "session": session.name,
            "boundary_order": list(BOUNDARY_NAMES),
            "samples": {},
            "status": "in_progress",
        }
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest["samples"]:
            reference_relative = next(iter(manifest["samples"].values()))
            reference = json.loads(
                (session / reference_relative).read_text(encoding="utf-8")
            )
            pose_delta_deg = float(np.degrees(np.max(np.abs(
                np.asarray(sample["arm_joint_positions_rad"])
                - np.asarray(reference["arm_joint_positions_rad"])
            ))))
            sample["capture_diagnostics"]["arm_pose_delta_from_first_deg"] = pose_delta_deg
            sample["capture_diagnostics"]["arm_pose_delta_warning"] = (
                pose_delta_deg > args.max_pose_delta_deg
            )

        if new_session:
            session.mkdir()
            active_path.write_text(session.name + "\n", encoding="utf-8")
        sample_directory.mkdir()
        _save_image(sample_directory / "rgb.png", nearest["color"])
        _save_image(sample_directory / "aligned_depth.png", nearest["aligned_depth"])
        _save_depth_preview(sample_directory / "aligned_depth_plasma.png", nearest["aligned_depth"])
        if nearest["raw_depth"] is not None:
            _save_image(sample_directory / "raw_depth.png", nearest["raw_depth"])
        if nearest["overlay"] is not None:
            _save_image(sample_directory / "cube_geometry_overlay.png", nearest["overlay"])
        if node.fallback_mask is not None:
            cv2.imwrite(
                str(sample_directory / "calibration_fallback_color_mask.png"),
                node.fallback_color_mask,
            )
            cv2.imwrite(str(sample_directory / "calibration_fallback_mask.png"), node.fallback_mask)
        if node.fallback_overlay is not None:
            cv2.imwrite(
                str(sample_directory / "calibration_fallback_overlay.png"),
                node.fallback_overlay,
            )
        (sample_directory / "sample.json").write_text(
            json.dumps(sample, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        debug_directory = Path(args.estimator_debug_directory)
        for name in ("fine.png", "fine.json"):
            source = debug_directory / name
            if source.is_file():
                shutil.copy2(source, sample_directory / f"estimator_{name}")

        manifest["samples"][boundary_name] = str(sample_directory.relative_to(session) / "sample.json")

        if args.boundary == 4:
            loaded = {}
            for name in BOUNDARY_NAMES:
                relative = manifest["samples"].get(name)
                if not relative:
                    raise RuntimeError(f"session is missing {name}")
                loaded[name] = json.loads((session / relative).read_text(encoding="utf-8"))
            region = build_safe_region(
                loaded,
                closing_margin_m=args.closing_margin_mm / 1000.0,
                finger_margin_m=args.finger_margin_mm / 1000.0,
            )
            region["source_session"] = session.name
            region["status"] = "provisional_pending_descend_validation"
            (session / "safe_region.json").write_text(
                json.dumps(region, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
            )
            manifest["status"] = "complete"
            manifest["safe_region"] = "safe_region.json"
            if active_path.exists():
                active_path.unlink()
        manifest_path.write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        return session, sample
    finally:
        node.destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture one of four yellow-cube safe-descend boundaries"
    )
    parser.add_argument("boundary", type=int, choices=(1, 2, 3, 4))
    parser.add_argument("--new", action="store_true", help="start a new session at boundary 1")
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--estimate-service", default="/arm/perception/estimate_cube")
    parser.add_argument("--planning-frame", default="base_link")
    parser.add_argument("--gravity-frame", default="gravity_frame")
    parser.add_argument("--tcp-frame", default="tcp_link")
    parser.add_argument("--color-topic", default="/wrist_camera/color/image_raw")
    parser.add_argument("--color-info-topic", default="/wrist_camera/color/camera_info")
    parser.add_argument("--aligned-depth-topic", default="/wrist_camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--aligned-info-topic", default="/wrist_camera/aligned_depth_to_color/camera_info")
    parser.add_argument("--raw-depth-topic", default="/wrist_camera/depth/image_rect_raw")
    parser.add_argument("--imu-topic", default="/wrist_camera/imu")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--overlay-topic", default="/arm/perception/debug/cube_geometry")
    parser.add_argument("--estimator-debug-directory", default="/tmp/d1_cube_debug_latest")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--sync-tolerance", type=float, default=0.04)
    parser.add_argument("--stationary-window", type=float, default=1.0)
    parser.add_argument("--max-stationary-deg", type=float, default=0.5)
    parser.add_argument("--max-pose-delta-deg", type=float, default=1.0)
    # The four captured points are already physically validated boundary
    # cases.  Extra inset is optional rather than silently applied twice.
    parser.add_argument("--closing-margin-mm", type=float, default=0.0)
    parser.add_argument("--finger-margin-mm", type=float, default=0.0)
    parser.add_argument("--yellow-hsv-lower", nargs=3, type=int, default=(18, 70, 60))
    parser.add_argument("--yellow-hsv-upper", nargs=3, type=int, default=(40, 255, 255))
    parser.add_argument("--fallback-min-area-px", type=int, default=500)
    parser.add_argument("--fallback-min-top-area-px", type=int, default=200)
    parser.add_argument("--fallback-min-valid-depth-ratio", type=float, default=0.30)
    parser.add_argument("--fallback-top-height-tolerance-m", type=float, default=0.012)
    parser.add_argument("--depth-min-m", type=float, default=0.20)
    parser.add_argument("--depth-max-m", type=float, default=2.0)
    parser.add_argument("--ground-max-points", type=int, default=12000)
    parser.add_argument("--ground-distance-m", type=float, default=0.008)
    parser.add_argument("--ground-normal-tolerance-deg", type=float, default=15.0)
    parser.add_argument("--ground-iterations", type=int, default=160)
    parser.add_argument("--cube-size-m", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    try:
        session, sample = _capture(args)
        print(
            f"CAPTURED boundary {args.boundary}/4 ({sample['boundary_name']}): "
            f"top_center_camera={np.round(sample['top_center_camera_m'], 6).tolist()} m"
        )
        print(f"Data: {session}")
        if args.boundary == 4:
            print(f"SAFE REGION GENERATED: {session / 'safe_region.json'}")
        else:
            print(f"Next boundary: {args.boundary + 1}/4")
        return 0
    except (OSError, RuntimeError, TimeoutError, TransformException, ValueError) as exception:
        print(f"CALIBRATION CAPTURE FAILED: {exception}", file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
