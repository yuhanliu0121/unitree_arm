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


def transform_rotation(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Return the 3x3 rotation represented by an xyzw quaternion."""
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if quaternion.shape != (4,):
        raise ValueError("quaternion must contain four values")
    norm = np.linalg.norm(quaternion)
    if norm <= 1e-12:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def undistorted_rays(pixels: np.ndarray, intrinsic: Sequence[float], distortion: Sequence[float]) -> np.ndarray:
    """Convert raw plumb-bob pixels to unit optical-frame bearing vectors."""
    import cv2

    values = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    camera = np.asarray(intrinsic, dtype=np.float64).reshape(3, 3)
    coefficients = np.asarray(distortion, dtype=np.float64)
    normalized = cv2.undistortPoints(values, camera, coefficients).reshape(-1, 2)
    rays = np.column_stack((normalized, np.ones(len(normalized))))
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


def fit_zucchini_axis_on_plane(
    mask: np.ndarray,
    camera_origin: Sequence[float],
    camera_rotation: np.ndarray,
    intrinsic: Sequence[float],
    distortion: Sequence[float],
    plane_normal: Sequence[float],
    plane_offset: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    """Fit a robust middle-segment axis and metric footprint on a known plane."""
    import cv2

    binary = np.asarray(mask, dtype=np.uint8)
    if binary.ndim != 2 or int(binary.sum()) < 20:
        raise ValueError("zucchini mask is too small")
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)
    if count < 2:
        raise ValueError("zucchini mask has no connected component")
    label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    component = np.asarray(labels == label, dtype=np.uint8)
    rows, columns = np.nonzero(component)
    pixels = np.column_stack((columns, rows)).astype(np.float64)
    pixel_center = centroids[label]
    covariance = np.cov(pixels - pixel_center, rowvar=False)
    values, vectors = np.linalg.eigh(covariance)
    global_axis = vectors[:, int(np.argmax(values))]
    skeleton = np.zeros_like(component)
    working = component.copy()
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while cv2.countNonZero(working):
        eroded = cv2.erode(working, kernel)
        skeleton |= working & ~cv2.dilate(eroded, kernel)
        working = eroded
    skeleton_rows, skeleton_columns = np.nonzero(skeleton)
    skeleton_pixels = np.column_stack((skeleton_columns, skeleton_rows)).astype(np.float64)
    if len(skeleton_pixels) < 8:
        skeleton_pixels = pixels
    global_projection = (skeleton_pixels - pixel_center) @ global_axis
    half_window = max(6.0, 0.20 * float(np.ptp(global_projection)))
    middle = skeleton_pixels[np.abs(global_projection) <= half_window]
    if len(middle) < 20:
        middle = pixels
    middle_center = middle.mean(axis=0)
    middle_covariance = np.cov(middle - middle_center, rowvar=False)
    values, vectors = np.linalg.eigh(middle_covariance)
    local_axis_px = vectors[:, int(np.argmax(values))]
    if local_axis_px.dot(global_axis) < 0.0:
        local_axis_px = -local_axis_px

    all_rays = undistorted_rays(pixels, intrinsic, distortion) @ np.asarray(camera_rotation).T
    all_points = intersect_rays_with_plane(
        camera_origin, all_rays, plane_normal, plane_offset
    )
    center_ray = undistorted_rays([middle_center], intrinsic, distortion) @ np.asarray(camera_rotation).T
    center = intersect_rays_with_plane(
        camera_origin, center_ray, plane_normal, plane_offset
    )[0]
    sample_pixels = np.vstack((middle_center - 12.0 * local_axis_px,
                               middle_center + 12.0 * local_axis_px))
    sample_rays = undistorted_rays(sample_pixels, intrinsic, distortion) @ np.asarray(camera_rotation).T
    sample_points = intersect_rays_with_plane(
        camera_origin, sample_rays, plane_normal, plane_offset
    )
    axis = sample_points[1] - sample_points[0]
    normal = np.asarray(plane_normal, dtype=np.float64)
    normal /= np.linalg.norm(normal)
    axis -= axis.dot(normal) * normal
    axis /= np.linalg.norm(axis)
    relative = all_points - center
    along = relative @ axis
    perpendicular = np.cross(normal, axis)
    across = relative @ perpendicular
    length = float(np.quantile(along, 0.98) - np.quantile(along, 0.02))
    width = float(np.quantile(across, 0.98) - np.quantile(across, 0.02))
    segment_half = 0.18 * length
    segment = np.vstack((center - segment_half * axis, center + segment_half * axis))
    return center, axis, segment, length, width


def fit_ground_plane_ransac(
    points: np.ndarray,
    up: Sequence[float],
    distance_threshold: float = 0.008,
    normal_tolerance_deg: float = 15.0,
    iterations: int = 160,
    seed: int = 0,
) -> tuple[np.ndarray, float, np.ndarray]:
    """Fit n.p+d=0 while rejecting planes inconsistent with gravity."""
    cloud = np.asarray(points, dtype=np.float64)
    direction = np.asarray(up, dtype=np.float64)
    valid = np.isfinite(cloud).all(axis=1)
    cloud = cloud[valid]
    direction /= np.linalg.norm(direction)
    if len(cloud) < 3:
        raise ValueError("not enough valid points for ground RANSAC")
    cosine_limit = np.cos(np.deg2rad(normal_tolerance_deg))
    rng = np.random.default_rng(seed)
    best: np.ndarray | None = None
    for _ in range(iterations):
        sample = cloud[rng.choice(len(cloud), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if normal.dot(direction) < 0:
            normal = -normal
        if normal.dot(direction) < cosine_limit:
            continue
        offset = -normal.dot(sample[0])
        inliers = np.abs(cloud @ normal + offset) <= distance_threshold
        if best is None or int(inliers.sum()) > int(best.sum()):
            best = inliers
    if best is None or int(best.sum()) < 50:
        raise ValueError("no gravity-consistent ground plane found")
    selected = cloud[best]
    centroid = selected.mean(axis=0)
    covariance = (selected - centroid).T @ (selected - centroid)
    _, _, vh = np.linalg.svd(covariance, full_matrices=False)
    normal = vh[-1]
    if normal.dot(direction) < 0:
        normal = -normal
    if normal.dot(direction) < cosine_limit:
        raise ValueError("refined ground normal violates gravity constraint")
    offset = -float(normal.dot(centroid))
    inliers = np.abs(cloud @ normal + offset) <= distance_threshold
    return normal, offset, inliers


def intersect_rays_with_plane(
    origins: np.ndarray,
    directions: np.ndarray,
    normal: Sequence[float],
    offset: float,
) -> np.ndarray:
    """Intersect one or many world-frame rays with n.p+d=0."""
    starts = np.asarray(origins, dtype=np.float64)
    rays = np.asarray(directions, dtype=np.float64)
    n = np.asarray(normal, dtype=np.float64)
    starts = np.broadcast_to(starts, rays.shape)
    denominator = rays @ n
    if np.any(np.abs(denominator) < 1e-9):
        raise ValueError("ray is parallel to plane")
    distance = -(starts @ n + float(offset)) / denominator
    if np.any(distance <= 0):
        raise ValueError("plane intersection lies behind camera")
    return starts + distance[:, None] * rays


def fit_square_on_plane(points: np.ndarray, normal: Sequence[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fit a minimum-area rectangle and return center, unit edge, four corners."""
    import cv2

    cloud = np.asarray(points, dtype=np.float64)
    n = np.asarray(normal, dtype=np.float64)
    n /= np.linalg.norm(n)
    reference = np.array([1.0, 0.0, 0.0])
    if abs(reference.dot(n)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    axis_u = reference - reference.dot(n) * n
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(n, axis_u)
    coordinates = np.column_stack((cloud @ axis_u, cloud @ axis_v)).astype(np.float32)
    rectangle = cv2.minAreaRect(coordinates)
    corners_2d = cv2.boxPoints(rectangle).astype(np.float64)
    normal_coordinate = float(np.mean(cloud @ n))
    corners = (
        corners_2d[:, :1] * axis_u
        + corners_2d[:, 1:] * axis_v
        + normal_coordinate * n
    )
    center = corners.mean(axis=0)
    edge = corners[1] - corners[0]
    if np.linalg.norm(corners[2] - corners[1]) > np.linalg.norm(edge):
        edge = corners[2] - corners[1]
    edge /= np.linalg.norm(edge)
    return center, edge, corners
