"""ROS-independent geometry and matching helpers for wrist perception."""

from .core import (
    MatchResult,
    image_to_bgr,
    fit_ground_plane_ransac,
    fit_square_on_plane,
    fit_zucchini_axis_on_plane,
    intersect_rays_with_plane,
    match_target_detection,
    project_plumb_bob,
    ros_depth_to_meters,
    select_class_mask_near_pixel,
    target_detection_diagnostics,
    transform_point,
    transform_rotation,
    undistorted_rays,
)
from .safe_region_calibration import (
    BOUNDARY_NAMES,
    SLAB_BOUNDARY_NAMES,
    build_safe_region,
    build_safe_slab,
)

__all__ = [
    "MatchResult",
    "image_to_bgr",
    "fit_ground_plane_ransac",
    "fit_square_on_plane",
    "fit_zucchini_axis_on_plane",
    "intersect_rays_with_plane",
    "match_target_detection",
    "project_plumb_bob",
    "ros_depth_to_meters",
    "select_class_mask_near_pixel",
    "target_detection_diagnostics",
    "transform_point",
    "transform_rotation",
    "undistorted_rays",
    "BOUNDARY_NAMES",
    "SLAB_BOUNDARY_NAMES",
    "build_safe_region",
    "build_safe_slab",
]
