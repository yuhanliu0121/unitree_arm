"""Pure NumPy helpers shared by the ROS adapter and unit tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True, slots=True)
class MatchResult:
    detection: Any
    mask_distance_px: float
    position_distance_m: float


def image_to_bgr(
    data: bytes | bytearray | memoryview,
    height: int,
    width: int,
    step: int,
    encoding: str,
) -> np.ndarray:
    """Decode a ROS rgb8/bgr8 image while respecting padded row strides."""
    if encoding not in {"rgb8", "bgr8"}:
        raise ValueError(f"unsupported color encoding: {encoding}")
    if step < width * 3:
        raise ValueError("color image step is smaller than width * 3")
    rows = np.frombuffer(data, dtype=np.uint8)
    if rows.size != height * step:
        raise ValueError("color image data size does not match height and step")
    image = rows.reshape(height, step)[:, : width * 3].reshape(height, width, 3)
    return image[..., ::-1].copy() if encoding == "rgb8" else image.copy()


def ros_depth_to_meters(
    data: bytes | bytearray | memoryview,
    height: int,
    width: int,
    step: int,
    encoding: str,
    is_bigendian: bool,
    depth_scale_m_per_unit: float,
) -> np.ndarray:
    """Decode 16UC1 units or 32FC1 metres while respecting row strides."""
    if encoding == "16UC1":
        dtype = np.dtype(">u2" if is_bigendian else "<u2")
        itemsize = 2
        scale = float(depth_scale_m_per_unit)
    elif encoding == "32FC1":
        dtype = np.dtype(">f4" if is_bigendian else "<f4")
        itemsize = 4
        scale = 1.0
    else:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    if step < width * itemsize or step % itemsize:
        raise ValueError("depth image step is invalid")
    values = np.frombuffer(data, dtype=dtype)
    row_values = step // itemsize
    if values.size != height * row_values:
        raise ValueError("depth image data size does not match height and step")
    result = values.reshape(height, row_values)[:, :width].astype(np.float32)
    result *= scale
    return result


def project_plumb_bob(
    point_camera: Sequence[float],
    intrinsic: Sequence[float],
    distortion: Sequence[float],
) -> tuple[float, float]:
    """Project an optical-frame point into a raw plumb-bob RGB image."""
    point = np.asarray(point_camera, dtype=np.float64)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("camera point must contain three finite values")
    if point[2] <= 0.0:
        raise ValueError("target hint is behind the color optical frame")
    k = np.asarray(intrinsic, dtype=np.float64)
    if k.size != 9:
        raise ValueError("camera intrinsic matrix must contain nine values")
    x = point[0] / point[2]
    y = point[1] / point[2]
    coefficients = np.asarray(distortion, dtype=np.float64)
    if coefficients.size:
        if coefficients.size < 5:
            raise ValueError("plumb_bob distortion requires five coefficients")
        k1, k2, p1, p2, k3 = coefficients[:5]
        radius2 = x * x + y * y
        radial = 1.0 + radius2 * (k1 + radius2 * (k2 + radius2 * k3))
        x, y = (
            x * radial + 2.0 * p1 * x * y + p2 * (radius2 + 2.0 * x * x),
            y * radial + p1 * (radius2 + 2.0 * y * y) + 2.0 * p2 * x * y,
        )
    return float(k[0] * x + k[2]), float(k[4] * y + k[5])


def _mask_distance(mask: np.ndarray, pixel: tuple[float, float]) -> float:
    y_indices, x_indices = np.nonzero(mask)
    if x_indices.size == 0:
        return float("inf")
    pixel_x, pixel_y = pixel
    rounded_x = int(round(pixel_x))
    rounded_y = int(round(pixel_y))
    if (
        0 <= rounded_y < mask.shape[0]
        and 0 <= rounded_x < mask.shape[1]
        and mask[rounded_y, rounded_x]
    ):
        return 0.0
    return float(np.hypot(x_indices - pixel_x, y_indices - pixel_y).min())


def match_target_detection(
    detections: Sequence[Any],
    hint_pixel: tuple[float, float],
    hint_position_camera_m: Sequence[float],
    max_mask_distance_px: float,
) -> MatchResult | None:
    """Choose the depth-verified instance nearest the caller's target hint."""
    hint_position = np.asarray(hint_position_camera_m, dtype=np.float64)
    candidates: list[MatchResult] = []
    for detection in detections:
        if (
            not bool(detection.accepted)
            or detection.status != "depth_verified"
            or detection.position_camera_m is None
        ):
            continue
        mask_distance = _mask_distance(detection.mask, hint_pixel)
        if mask_distance > max_mask_distance_px:
            continue
        position_distance = float(
            np.linalg.norm(
                np.asarray(detection.position_camera_m, dtype=np.float64)
                - hint_position
            )
        )
        candidates.append(
            MatchResult(detection, mask_distance, position_distance)
        )
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda item: (
            item.mask_distance_px,
            item.position_distance_m,
            -float(item.detection.confidence),
        ),
    )


def transform_point(
    point: Sequence[float],
    translation: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """Apply a geometry_msgs Transform rotation and translation to a point."""
    vector = np.asarray(point, dtype=np.float64)
    translation_array = np.asarray(translation, dtype=np.float64)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if vector.shape != (3,) or translation_array.shape != (3,) or quaternion.shape != (4,):
        raise ValueError("invalid transform dimensions")
    norm = np.linalg.norm(quaternion)
    if norm <= 1e-12:
        raise ValueError("transform quaternion has zero norm")
    x, y, z, w = quaternion / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return rotation @ vector + translation_array
