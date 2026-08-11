from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .models import CameraIntrinsics


@dataclass(slots=True)
class RGBDFrame:
    color_bgr: np.ndarray
    aligned_depth_m: np.ndarray
    aligned_depth_raw: np.ndarray
    intrinsics: CameraIntrinsics
    timestamp_ms: float
    depth_scale_m: float


class RealSenseSource:
    """D435i source using default camera controls; only streams are configured."""

    def __init__(self, camera_config: dict[str, Any]):
        import pyrealsense2 as rs

        self.rs = rs
        self.config_values = camera_config
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.profile = None
        self.depth_scale_m = 0.001
        self.device_name = ""
        self.serial_number = ""

    def start(self) -> None:
        rs = self.rs
        values = self.config_values
        config = rs.config()
        serial = str(values.get("serial", "")).strip()
        if serial:
            config.enable_device(serial)
        config.enable_stream(
            rs.stream.depth,
            int(values["depth_width"]),
            int(values["depth_height"]),
            rs.format.z16,
            int(values["fps"]),
        )
        config.enable_stream(
            rs.stream.color,
            int(values["color_width"]),
            int(values["color_height"]),
            rs.format.bgr8,
            int(values["fps"]),
        )
        self.profile = self.pipeline.start(config)
        device = self.profile.get_device()
        sensor = device.first_depth_sensor()
        self.device_name = device.get_info(rs.camera_info.name)
        self.serial_number = device.get_info(rs.camera_info.serial_number)
        self.depth_scale_m = float(sensor.get_depth_scale())
        for _ in range(int(values.get("warmup_frames", 20))):
            self.pipeline.wait_for_frames(5000)

    def read(self, timeout_ms: int = 5000) -> RGBDFrame:
        if self.profile is None:
            raise RuntimeError("RealSenseSource.start() must be called first")
        frames = self.pipeline.wait_for_frames(timeout_ms)
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("D435i did not return an aligned RGB-D pair")
        color = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_m = depth_raw.astype(np.float32) * self.depth_scale_m
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        intrinsics = CameraIntrinsics(
            fx=float(intr.fx),
            fy=float(intr.fy),
            cx=float(intr.ppx),
            cy=float(intr.ppy),
            width=int(intr.width),
            height=int(intr.height),
        )
        return RGBDFrame(
            color_bgr=color,
            aligned_depth_m=depth_m,
            aligned_depth_raw=depth_raw,
            intrinsics=intrinsics,
            timestamp_ms=float(color_frame.get_timestamp()),
            depth_scale_m=self.depth_scale_m,
        )

    def stop(self) -> None:
        if self.profile is not None:
            self.pipeline.stop()
            self.profile = None

    def __enter__(self) -> "RealSenseSource":
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.stop()
