from __future__ import annotations

import sys
import time
from collections.abc import Callable

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def plasma_lut() -> np.ndarray:
    """Build the fixed RGB lookup table used by both real and simulated depth."""
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
    channels = [np.interp(samples, positions, colors[:, index]) for index in range(3)]
    return np.rint(np.stack(channels, axis=1)).astype(np.uint8)


PLASMA_LUT = plasma_lut()


def colorize_depth(
    depth_units: np.ndarray,
    scale_m_per_unit: float,
    sensor_min_m: float,
    sensor_max_m: float,
    display_min_m: float,
    display_max_m: float,
) -> np.ndarray:
    """Map valid depth units into a fixed-range Plasma RGB diagnostic image."""
    if display_max_m <= display_min_m:
        raise ValueError("display_max_m must be greater than display_min_m")
    depth_m = depth_units.astype(np.float32) * float(scale_m_per_unit)
    sensor_valid = (
        (depth_units != 0)
        & np.isfinite(depth_m)
        & (depth_m >= sensor_min_m)
        & (depth_m <= sensor_max_m)
    )
    display_valid = (
        sensor_valid
        & (depth_m >= display_min_m)
        & (depth_m <= display_max_m)
    )
    normalized = np.zeros(depth_m.shape, dtype=np.float32)
    normalized[display_valid] = np.clip(
        (depth_m[display_valid] - display_min_m)
        / (display_max_m - display_min_m),
        0.0,
        1.0,
    )
    indices = np.rint(normalized * 255.0).astype(np.uint8)
    result = PLASMA_LUT[indices]
    result[~display_valid] = 0
    return result


def image_depth_array(message: Image) -> np.ndarray:
    """Expose a potentially padded 16UC1 ROS image as a native uint16 array."""
    if message.encoding not in {"16UC1", "mono16"}:
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    row_bytes = int(message.width) * 2
    if message.step < row_bytes:
        raise ValueError(
            f"depth step {message.step} is smaller than row payload {row_bytes}"
        )
    required_bytes = int(message.step) * int(message.height)
    if len(message.data) < required_bytes:
        raise ValueError(
            f"depth payload has {len(message.data)} bytes; expected {required_bytes}"
        )
    wire_dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
    view = np.ndarray(
        shape=(int(message.height), int(message.width)),
        dtype=wire_dtype,
        buffer=message.data,
        strides=(int(message.step), 2),
    )
    native_big_endian = sys.byteorder == "big"
    if bool(message.is_bigendian) != native_big_endian:
        return view.byteswap().view(view.dtype.newbyteorder("=")).copy()
    return view.astype(np.uint16, copy=True)


class DepthDebugVisualizer(Node):
    """Publish identical depth diagnostics for simulation and physical cameras."""

    def __init__(self) -> None:
        super().__init__("d1_depth_debug_visualizer")
        defaults = {
            "raw_depth_topic": "/wrist_camera/depth/image_rect_raw",
            "aligned_depth_topic": "/wrist_camera/aligned_depth_to_color/image_raw",
            "raw_debug_topic": "/wrist_camera/debug/depth_plasma",
            "aligned_debug_topic": "/wrist_camera/debug/aligned_depth_plasma",
            "depth_scale_m_per_unit": 0.001,
            "sensor_min_m": 0.20,
            "sensor_max_m": 10.0,
            "display_min_m": 0.20,
            "display_max_m": 2.0,
            "publish_rate_limit_hz": 10.0,
            "enable_raw_depth_debug": False,
            "enable_aligned_depth_debug": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._scale = float(self.get_parameter("depth_scale_m_per_unit").value)
        self._sensor_min = float(self.get_parameter("sensor_min_m").value)
        self._sensor_max = float(self.get_parameter("sensor_max_m").value)
        self._display_min = float(self.get_parameter("display_min_m").value)
        self._display_max = float(self.get_parameter("display_max_m").value)
        rate_hz = float(self.get_parameter("publish_rate_limit_hz").value)
        enable_raw = bool(self.get_parameter("enable_raw_depth_debug").value)
        enable_aligned = bool(
            self.get_parameter("enable_aligned_depth_debug").value
        )
        if self._scale <= 0.0 or self._sensor_max <= self._sensor_min:
            raise ValueError("invalid physical depth range or scale")
        if self._display_min < self._sensor_min or self._display_max > self._sensor_max:
            raise ValueError("display depth range must lie inside the sensor-valid range")
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_limit_hz must be positive")
        if not enable_raw and not enable_aligned:
            raise ValueError("at least one depth debug stream must be enabled")
        self._minimum_period_s = 1.0 / rate_hz
        self._last_publish = {"raw": -float("inf"), "aligned": -float("inf")}
        self._warned_encodings: set[str] = set()

        enabled_streams = []
        if enable_raw:
            raw_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("raw_debug_topic").value),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("raw_depth_topic").value),
                self._callback("raw", raw_publisher.publish),
                qos_profile_sensor_data,
            )
            enabled_streams.append("raw")
        if enable_aligned:
            aligned_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("aligned_debug_topic").value),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("aligned_depth_topic").value),
                self._callback("aligned", aligned_publisher.publish),
                qos_profile_sensor_data,
            )
            enabled_streams.append("aligned")
        self.get_logger().info(
            "Depth diagnostics ready: "
            f"streams={','.join(enabled_streams)} "
            f"valid={self._sensor_min:.2f}..{self._sensor_max:.2f} m "
            f"display={self._display_min:.2f}..{self._display_max:.2f} m "
            f"rate<={rate_hz:.1f} Hz"
        )

    def _callback(self, stream: str, publish: Callable[[Image], None]):
        def convert(message: Image) -> None:
            now = time.monotonic()
            if now - self._last_publish[stream] < self._minimum_period_s:
                return
            try:
                depth = image_depth_array(message)
            except ValueError as error:
                key = str(error)
                if key not in self._warned_encodings:
                    self._warned_encodings.add(key)
                    self.get_logger().warning(key)
                return
            colored = colorize_depth(
                depth,
                self._scale,
                self._sensor_min,
                self._sensor_max,
                self._display_min,
                self._display_max,
            )
            output = Image()
            output.header = message.header
            output.height = message.height
            output.width = message.width
            output.encoding = "rgb8"
            output.is_bigendian = 0
            output.step = int(message.width) * 3
            output.data = colored.tobytes()
            publish(output)
            self._last_publish[stream] = now

        return convert


def main() -> None:
    rclpy.init()
    node = DepthDebugVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
