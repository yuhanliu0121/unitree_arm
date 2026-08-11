from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the depth image after alignment to RGB."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(slots=True)
class RawDetection:
    instance_id: int
    class_id: int
    class_name: str
    confidence: float
    mask: np.ndarray = field(repr=False)
    bbox_xyxy: tuple[int, int, int, int] = (0, 0, 0, 0)


@dataclass(slots=True)
class Detection3D:
    instance_id: int
    class_id: int
    class_name: str
    confidence: float
    accepted: bool
    status: str
    reject_reasons: tuple[str, ...]
    bbox_xyxy: tuple[int, int, int, int]
    center_pixel: tuple[int, int]
    depth_m: float | None
    depth_valid_ratio: float
    depth_spread_m: float | None
    position_camera_m: tuple[float, float, float] | None
    dimensions_m: tuple[float, float, float] | None
    longest_size_m: float | None
    apparent_size_m: float | None
    bearing_deg: float | None
    mask_pixels: int
    mask_fraction: float
    core_pixels: int
    mask: np.ndarray = field(repr=False)
    core_mask: np.ndarray = field(repr=False)
    point_cloud_camera_m: np.ndarray | None = field(default=None, repr=False)

    @property
    def depth_verified(self) -> bool:
        return self.status == "depth_verified"

    def to_dict(self) -> dict[str, Any]:
        return {
            "instance_id": self.instance_id,
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": round(self.confidence, 6),
            "accepted": self.accepted,
            "status": self.status,
            "reject_reasons": list(self.reject_reasons),
            "bbox_xyxy": list(self.bbox_xyxy),
            "center_pixel": list(self.center_pixel),
            "depth_m": self.depth_m,
            "depth_valid_ratio": self.depth_valid_ratio,
            "depth_spread_m": self.depth_spread_m,
            "position_camera_m": list(self.position_camera_m) if self.position_camera_m else None,
            "dimensions_m": list(self.dimensions_m) if self.dimensions_m else None,
            "longest_size_m": self.longest_size_m,
            "apparent_size_m": self.apparent_size_m,
            "bearing_deg": self.bearing_deg,
            "mask_pixels": self.mask_pixels,
            "mask_fraction": self.mask_fraction,
            "core_pixels": self.core_pixels,
        }
