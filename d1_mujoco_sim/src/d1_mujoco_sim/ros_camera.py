from __future__ import annotations

import array
import logging
import sys
import threading

import mujoco
import numpy as np


LOGGER = logging.getLogger(__name__)


def _plasma_lut() -> np.ndarray:
    """Return a compact plasma-like RGB lookup table without matplotlib."""
    positions = np.asarray(
        [0.0, 0.13, 0.25, 0.38, 0.50, 0.63, 0.75, 0.88, 1.0],
        dtype=np.float32,
    )
    colors = np.asarray(
        [
            [13, 8, 135],
            [75, 3, 161],
            [125, 3, 168],
            [168, 34, 150],
            [203, 70, 121],
            [229, 107, 93],
            [248, 148, 65],
            [253, 195, 40],
            [240, 249, 33],
        ],
        dtype=np.float32,
    )
    samples = np.linspace(0.0, 1.0, 256, dtype=np.float32)
    channels = [np.interp(samples, positions, colors[:, i]) for i in range(3)]
    return np.rint(np.stack(channels, axis=1)).astype(np.uint8)


PLASMA_LUT = _plasma_lut()

# REP-103 camera body axes are +X forward, +Y left, +Z up. Optical axes are
# +X right, +Y down, +Z forward.
CAMERA_LINK_FROM_OPTICAL = np.asarray(
    [
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def depth_to_uint16(
    depth_m: np.ndarray,
    scale_m_per_unit: float,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    valid = (
        np.isfinite(depth_m)
        & (depth_m >= min_depth_m)
        & (depth_m <= max_depth_m)
    )
    result = np.zeros(depth_m.shape, dtype=np.uint16)
    scaled = np.rint(depth_m[valid] / scale_m_per_unit)
    result[valid] = np.clip(scaled, 1, np.iinfo(np.uint16).max).astype(
        np.uint16
    )
    return result


def colorize_depth(
    depth_m: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    valid = (
        np.isfinite(depth_m)
        & (depth_m >= min_depth_m)
        & (depth_m <= max_depth_m)
    )
    normalized = np.zeros(depth_m.shape, dtype=np.float32)
    normalized[valid] = np.clip(
        (depth_m[valid] - min_depth_m) / (max_depth_m - min_depth_m),
        0.0,
        1.0,
    )
    indices = np.rint(normalized * 255.0).astype(np.uint8)
    result = PLASMA_LUT[indices]
    result[~valid] = 0
    return result


def distortion_source_indices(config: dict) -> np.ndarray | None:
    """Map each distorted output pixel to its ideal pinhole source pixel."""
    coefficients = np.asarray(config.get("distortion", []), dtype=np.float64)
    if coefficients.size == 0 or np.allclose(coefficients, 0.0):
        return None
    if coefficients.size != 5:
        raise ValueError("plumb_bob distortion must contain five coefficients")

    width = int(config["width"])
    height = int(config["height"])
    fx = float(config["fx"])
    fy = float(config["fy"])
    cx = float(config["cx"])
    cy = float(config["cy"])
    k1, k2, p1, p2, k3 = coefficients
    pixel_y, pixel_x = np.indices((height, width), dtype=np.float64)
    distorted_x = (pixel_x - cx) / fx
    distorted_y = (pixel_y - cy) / fy
    ideal_x = distorted_x.copy()
    ideal_y = distorted_y.copy()
    for _ in range(8):
        radius2 = ideal_x * ideal_x + ideal_y * ideal_y
        radial = 1.0 + radius2 * (
            k1 + radius2 * (k2 + radius2 * k3)
        )
        delta_x = (
            2.0 * p1 * ideal_x * ideal_y
            + p2 * (radius2 + 2.0 * ideal_x * ideal_x)
        )
        delta_y = (
            p1 * (radius2 + 2.0 * ideal_y * ideal_y)
            + 2.0 * p2 * ideal_x * ideal_y
        )
        ideal_x = (distorted_x - delta_x) / radial
        ideal_y = (distorted_y - delta_y) / radial

    source_x = np.rint(fx * ideal_x + cx).astype(np.int32)
    source_y = np.rint(fy * ideal_y + cy).astype(np.int32)
    # A same-size ideal pinhole render has no samples just outside its border.
    # Replicate the nearest edge pixel there instead of creating an artificial
    # black rim which a physical RealSense image would not contain.
    source_x = np.clip(source_x, 0, width - 1)
    source_y = np.clip(source_y, 0, height - 1)
    return source_y * width + source_x


def apply_distortion(image: np.ndarray, indices: np.ndarray | None) -> np.ndarray:
    if indices is None:
        return image
    flattened = image.reshape(-1, image.shape[2])
    safe_indices = np.maximum(indices.reshape(-1), 0)
    result = flattened[safe_indices].reshape(image.shape)
    result[indices < 0] = 0
    return result


def _deproject_depth_pixels(
    pixel_x: np.ndarray,
    pixel_y: np.ndarray,
    depth_m: np.ndarray,
    config: dict,
) -> np.ndarray:
    """Deproject rectified depth pixels into the depth optical frame."""
    normalized_x = (pixel_x - float(config["cx"])) / float(config["fx"])
    normalized_y = (pixel_y - float(config["cy"])) / float(config["fy"])
    return np.stack(
        (
            normalized_x * depth_m,
            normalized_y * depth_m,
            depth_m,
        ),
        axis=0,
    )


def _project_color_pixels(points: np.ndarray, config: dict) -> np.ndarray:
    """Project color-frame points through the calibrated plumb-bob model."""
    normalized_x = points[0] / points[2]
    normalized_y = points[1] / points[2]
    coefficients = np.asarray(config.get("distortion", []), dtype=np.float64)
    if coefficients.size:
        if coefficients.size != 5:
            raise ValueError(
                "plumb_bob distortion must contain five coefficients"
            )
        k1, k2, p1, p2, k3 = coefficients
        radius2 = normalized_x * normalized_x + normalized_y * normalized_y
        radial = 1.0 + radius2 * (
            k1 + radius2 * (k2 + radius2 * k3)
        )
        distorted_x = (
            normalized_x * radial
            + 2.0 * p1 * normalized_x * normalized_y
            + p2 * (radius2 + 2.0 * normalized_x * normalized_x)
        )
        distorted_y = (
            normalized_y * radial
            + p1 * (radius2 + 2.0 * normalized_y * normalized_y)
            + 2.0 * p2 * normalized_x * normalized_y
        )
    else:
        distorted_x = normalized_x
        distorted_y = normalized_y
    return np.stack(
        (
            float(config["fx"]) * distorted_x + float(config["cx"]),
            float(config["fy"]) * distorted_y + float(config["cy"]),
        ),
        axis=0,
    )


def align_depth_to_color(
    depth_raw: np.ndarray,
    depth_config: dict,
    color_config: dict,
    color_from_depth: np.ndarray,
) -> np.ndarray:
    """Align a Z16 depth frame to the distorted RGB pixel grid.

    This follows librealsense's CPU align semantics: project both corners of
    each valid depth pixel, fill the covered RGB rectangle, and keep the
    nearest original Z16 value when projected regions overlap.
    """
    expected_shape = (
        int(depth_config["height"]),
        int(depth_config["width"]),
    )
    if depth_raw.shape != expected_shape:
        raise ValueError(
            f"Depth shape {depth_raw.shape} does not match {expected_shape}"
        )
    transform = np.asarray(color_from_depth, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("color_from_depth must be a 4x4 transform")

    source_y, source_x = np.nonzero(depth_raw)
    output_shape = (
        int(color_config["height"]),
        int(color_config["width"]),
    )
    if source_x.size == 0:
        return np.zeros(output_shape, dtype=np.uint16)

    source_z16 = depth_raw[source_y, source_x]
    depth_m = source_z16.astype(np.float64) * float(
        depth_config["depth_scale_m_per_unit"]
    )

    projected_corners = []
    color_z = None
    for offset in (-0.5, 0.5):
        depth_points = _deproject_depth_pixels(
            source_x.astype(np.float64) + offset,
            source_y.astype(np.float64) + offset,
            depth_m,
            depth_config,
        )
        color_points = (
            transform[:3, :3] @ depth_points
            + transform[:3, 3, np.newaxis]
        )
        if color_z is None:
            color_z = color_points[2]
        projected_corners.append(
            _project_color_pixels(color_points, color_config)
        )

    top_left, bottom_right = projected_corners
    x0 = np.floor(top_left[0] + 0.5).astype(np.int32)
    y0 = np.floor(top_left[1] + 0.5).astype(np.int32)
    x1 = np.floor(bottom_right[0] + 0.5).astype(np.int32)
    y1 = np.floor(bottom_right[1] + 0.5).astype(np.int32)
    target_height, target_width = output_shape
    valid = (
        (color_z > 0.0)
        & (x0 >= 0)
        & (y0 >= 0)
        & (x1 >= x0)
        & (y1 >= y0)
        & (x1 < target_width)
        & (y1 < target_height)
    )
    if not np.any(valid):
        return np.zeros(output_shape, dtype=np.uint16)

    x0 = x0[valid]
    y0 = y0[valid]
    x1 = x1[valid]
    y1 = y1[valid]
    source_z16 = source_z16[valid]
    output = np.full(
        target_height * target_width,
        np.iinfo(np.uint16).max,
        dtype=np.uint16,
    )
    max_width = int(np.max(x1 - x0))
    max_height = int(np.max(y1 - y0))
    for offset_y in range(max_height + 1):
        for offset_x in range(max_width + 1):
            covered = (
                (x0 + offset_x <= x1)
                & (y0 + offset_y <= y1)
            )
            target_index = (
                (y0[covered] + offset_y) * target_width
                + x0[covered]
                + offset_x
            )
            np.minimum.at(output, target_index, source_z16[covered])
    output[output == np.iinfo(np.uint16).max] = 0
    return output.reshape(output_shape)


def _quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    quaternion_wxyz = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion_wxyz, rotation.reshape(-1))
    return [
        float(quaternion_wxyz[1]),
        float(quaternion_wxyz[2]),
        float(quaternion_wxyz[3]),
        float(quaternion_wxyz[0]),
    ]


def camera_static_transforms(
    config: dict,
) -> list[tuple[str, str, np.ndarray]]:
    """Build the REP-103 wrist-camera TF edges from calibrated optical poses."""
    link_from_color = np.asarray(
        config["T_link6_color_optical"],
        dtype=np.float64,
    )
    color_from_depth = np.asarray(
        config["T_color_depth_optical"],
        dtype=np.float64,
    )
    camera_link_from_color = CAMERA_LINK_FROM_OPTICAL
    link_from_camera_link = (
        link_from_color @ np.linalg.inv(camera_link_from_color)
    )
    camera_link_from_depth = camera_link_from_color @ color_from_depth
    parent_link = str(config.get("parent_link", "Link6"))
    camera_link = str(config.get("link_frame", "wrist_camera_link"))
    return [
        (parent_link, camera_link, link_from_camera_link),
        (
            camera_link,
            str(config["color"]["frame"]),
            camera_link_from_color,
        ),
        (
            camera_link,
            str(config["depth"]["frame"]),
            camera_link_from_depth,
        ),
    ]


class RosCameraPublisher:
    """Render calibrated MuJoCo cameras and publish RealSense-like ROS topics."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: dict,
        *,
        publish_debug: bool = False,
    ) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import TransformStamped
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import CameraInfo, Image
            from tf2_ros.static_transform_broadcaster import (
                StaticTransformBroadcaster,
            )
        except ImportError as exc:
            raise RuntimeError(
                "ROS camera publishing requires sourcing "
                "/opt/ros/humble/setup.zsh before starting d1-mujoco-sim"
            ) from exc

        self._rclpy = rclpy
        self._Image = Image
        self._CameraInfo = CameraInfo
        self._TransformStamped = TransformStamped
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            # Keep the simulator's process-level SIGTERM behavior. Installing
            # rclpy's handler would invalidate the context in the middle of a
            # render/publish cycle and print a spurious traceback on cleanup.
            rclpy.init(
                args=[],
                signal_handler_options=SignalHandlerOptions.NO,
            )
        self._node = rclpy.create_node("d1_mujoco_camera")
        self._model = model
        self._data = data
        self._config = config
        self._publish_debug = publish_debug
        self.publish_rate_hz = float(config.get("publish_rate_hz", 10.0))
        self._topic_root = str(config.get("topic_root", "/wrist_camera"))
        self._topic_root = self._topic_root.rstrip("/")
        self._min_depth_m = float(config.get("min_depth_m", 0.2))
        self._max_depth_m = float(config.get("max_depth_m", 10.0))
        self._display_min_depth_m = float(
            config.get("display_min_depth_m", self._min_depth_m)
        )
        self._display_max_depth_m = float(
            config.get("display_max_depth_m", 2.0)
        )
        color = config["color"]
        depth = config["depth"]
        model.vis.global_.offwidth = max(
            int(model.vis.global_.offwidth),
            int(color["width"]),
            int(depth["width"]),
        )
        model.vis.global_.offheight = max(
            int(model.vis.global_.offheight),
            int(color["height"]),
            int(depth["height"]),
        )
        self._color_distortion_indices = distortion_source_indices(color)
        topics = self._publisher_topics(self._topic_root)

        self._color_image_publisher = self._node.create_publisher(
            Image,
            topics["color_image"],
            qos_profile_sensor_data,
        )
        self._color_info_publisher = self._node.create_publisher(
            CameraInfo,
            topics["color_info"],
            qos_profile_sensor_data,
        )
        self._depth_image_publisher = self._node.create_publisher(
            Image,
            topics["depth_image"],
            qos_profile_sensor_data,
        )
        self._depth_info_publisher = self._node.create_publisher(
            CameraInfo,
            topics["depth_info"],
            qos_profile_sensor_data,
        )
        self._aligned_depth_publisher = self._node.create_publisher(
            Image,
            topics["aligned_depth_image"],
            qos_profile_sensor_data,
        )
        self._aligned_depth_info_publisher = self._node.create_publisher(
            CameraInfo,
            topics["aligned_depth_info"],
            qos_profile_sensor_data,
        )
        self._raw_depth_debug_publisher = None
        self._aligned_depth_debug_publisher = None
        if publish_debug:
            self._raw_depth_debug_publisher = self._node.create_publisher(
                Image,
                topics["raw_depth_debug"],
                qos_profile_sensor_data,
            )
            self._aligned_depth_debug_publisher = self._node.create_publisher(
                Image,
                topics["aligned_depth_debug"],
                qos_profile_sensor_data,
            )

        self._static_tf = StaticTransformBroadcaster(self._node)
        self._publish_static_transforms()
        self._condition = threading.Condition()
        self._pending_qpos: np.ndarray | None = None
        self._worker_stop = False
        self._worker_error: BaseException | None = None
        self._worker_ready = threading.Event()
        self._worker = threading.Thread(
            target=self._worker_main,
            name="d1-mujoco-camera-renderer",
            daemon=True,
        )
        self._worker.start()
        if not self._worker_ready.wait(timeout=10.0):
            raise RuntimeError("Timed out initializing camera render thread")
        if self._worker_error is not None:
            raise RuntimeError("Could not initialize camera render thread") from (
                self._worker_error
            )
        LOGGER.info(
            "ROS camera streams ready under %s at %.2f Hz (debug=%s)",
            self._topic_root,
            self.publish_rate_hz,
            publish_debug,
        )

    @staticmethod
    def _publisher_topics(topic_root: str) -> dict[str, str]:
        """Return the stable stream and optional diagnostic topic names."""
        root = topic_root.rstrip("/")
        return {
            "color_image": f"{root}/color/image_raw",
            "color_info": f"{root}/color/camera_info",
            "depth_image": f"{root}/depth/image_rect_raw",
            "depth_info": f"{root}/depth/camera_info",
            "aligned_depth_image": (
                f"{root}/aligned_depth_to_color/image_raw"
            ),
            "aligned_depth_info": (
                f"{root}/aligned_depth_to_color/camera_info"
            ),
            "raw_depth_debug": f"{root}/debug/depth_plasma",
            "aligned_depth_debug": (
                f"{root}/debug/aligned_depth_plasma"
            ),
        }

    @staticmethod
    def _camera_info_values(config: dict) -> tuple[list[float], list[float]]:
        fx = float(config["fx"])
        fy = float(config["fy"])
        cx = float(config["cx"])
        cy = float(config["cy"])
        intrinsic = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        projection = [
            fx,
            0.0,
            cx,
            0.0,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return intrinsic, projection

    def _camera_info(self, config: dict, stamp) -> object:
        message = self._CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = str(config["frame"])
        message.width = int(config["width"])
        message.height = int(config["height"])
        message.distortion_model = str(config["distortion_model"])
        message.d = [float(value) for value in config["distortion"]]
        message.k, message.p = self._camera_info_values(config)
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return message

    def _image(
        self,
        image_array: np.ndarray,
        encoding: str,
        frame: str,
        stamp,
    ):
        message = self._Image()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.height = int(image_array.shape[0])
        message.width = int(image_array.shape[1])
        message.encoding = encoding
        message.is_bigendian = int(sys.byteorder == "big")
        message.step = int(image_array.strides[0])
        # Assigning bytes makes rosidl's debug setter validate every element in
        # Python (hundreds of milliseconds for a full RGB-D pair). Its native
        # array.array fast path performs the same copy in C.
        message.data = array.array(
            "B",
            np.ascontiguousarray(image_array).tobytes(),
        )
        return message

    def _publish_static_transforms(self) -> None:
        stamp = self._node.get_clock().now().to_msg()
        transforms = []
        for parent, child, transform in camera_static_transforms(
            self._config
        ):
            message = self._TransformStamped()
            message.header.stamp = stamp
            message.header.frame_id = parent
            message.child_frame_id = child
            translation = transform[:3, 3]
            message.transform.translation.x = float(translation[0])
            message.transform.translation.y = float(translation[1])
            message.transform.translation.z = float(translation[2])
            quaternion = _quaternion_xyzw(transform[:3, :3])
            message.transform.rotation.x = quaternion[0]
            message.transform.rotation.y = quaternion[1]
            message.transform.rotation.z = quaternion[2]
            message.transform.rotation.w = quaternion[3]
            transforms.append(message)
        self._static_tf.sendTransform(transforms)

    def _worker_main(self) -> None:
        color_config = self._config["color"]
        depth_config = self._config["depth"]
        color_renderer = None
        depth_renderer = None
        try:
            color_renderer = mujoco.Renderer(
                self._model,
                height=int(color_config["height"]),
                width=int(color_config["width"]),
            )
            depth_renderer = mujoco.Renderer(
                self._model,
                height=int(depth_config["height"]),
                width=int(depth_config["width"]),
            )
            depth_renderer.enable_depth_rendering()
            render_data = mujoco.MjData(self._model)
            self._worker_ready.set()
            while True:
                with self._condition:
                    self._condition.wait_for(
                        lambda: (
                            self._pending_qpos is not None
                            or self._worker_stop
                        )
                    )
                    if self._worker_stop:
                        break
                    qpos = self._pending_qpos
                    self._pending_qpos = None
                render_data.qpos[:] = qpos
                mujoco.mj_forward(self._model, render_data)
                self._publish_frame(
                    render_data,
                    color_renderer,
                    depth_renderer,
                )
        except BaseException as exc:
            self._worker_error = exc
            self._worker_ready.set()
            LOGGER.exception("Camera render thread failed")
        finally:
            if color_renderer is not None:
                color_renderer.close()
            if depth_renderer is not None:
                depth_renderer.close()

    def publish(self) -> None:
        """Queue the newest simulation state without blocking physics."""
        if self._worker_error is not None:
            raise RuntimeError("Camera render thread failed") from (
                self._worker_error
            )
        with self._condition:
            self._pending_qpos = self._data.qpos.copy()
            self._condition.notify()

    def _publish_frame(
        self,
        data: mujoco.MjData,
        color_renderer: mujoco.Renderer,
        depth_renderer: mujoco.Renderer,
    ) -> None:
        stamp = self._node.get_clock().now().to_msg()
        color_config = self._config["color"]
        depth_config = self._config["depth"]

        color_renderer.update_scene(
            data,
            camera=str(color_config["name"]),
        )
        color = color_renderer.render()
        color = apply_distortion(color, self._color_distortion_indices)

        depth_renderer.update_scene(
            data,
            camera=str(depth_config["name"]),
        )
        depth_m = depth_renderer.render()
        depth_raw = depth_to_uint16(
            depth_m,
            float(depth_config["depth_scale_m_per_unit"]),
            self._min_depth_m,
            self._max_depth_m,
        )
        aligned_depth_raw = align_depth_to_color(
            depth_raw,
            depth_config,
            color_config,
            np.asarray(
                self._config["T_color_depth_optical"],
                dtype=np.float64,
            ),
        )

        self._color_image_publisher.publish(
            self._image(
                color,
                "rgb8",
                str(color_config["frame"]),
                stamp,
            )
        )
        self._color_info_publisher.publish(
            self._camera_info(color_config, stamp)
        )
        self._depth_image_publisher.publish(
            self._image(
                depth_raw,
                "16UC1",
                str(depth_config["frame"]),
                stamp,
            )
        )
        self._depth_info_publisher.publish(
            self._camera_info(depth_config, stamp)
        )
        self._aligned_depth_publisher.publish(
            self._image(
                aligned_depth_raw,
                "16UC1",
                str(color_config["frame"]),
                stamp,
            )
        )
        self._aligned_depth_info_publisher.publish(
            self._camera_info(color_config, stamp)
        )
        if self._publish_debug:
            depth_colorized = colorize_depth(
                depth_m,
                self._display_min_depth_m,
                self._display_max_depth_m,
            )
            aligned_depth_m = aligned_depth_raw.astype(np.float32) * float(
                depth_config["depth_scale_m_per_unit"]
            )
            aligned_depth_colorized = colorize_depth(
                aligned_depth_m,
                self._display_min_depth_m,
                self._display_max_depth_m,
            )
            self._raw_depth_debug_publisher.publish(
                self._image(
                    depth_colorized,
                    "rgb8",
                    str(depth_config["frame"]),
                    stamp,
                )
            )
            self._aligned_depth_debug_publisher.publish(
                self._image(
                    aligned_depth_colorized,
                    "rgb8",
                    str(color_config["frame"]),
                    stamp,
                )
            )
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        with self._condition:
            self._worker_stop = True
            self._condition.notify()
        self._worker.join(timeout=10.0)
        if self._worker.is_alive():
            LOGGER.warning("Camera render thread did not stop within 10 seconds")
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()
