from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "python"),
)

from d1_perception_adapter import (  # noqa: E402
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
    undistorted_rays,
)


@dataclass
class Detection:
    accepted: bool
    status: str
    position_camera_m: tuple[float, float, float] | None
    confidence: float
    mask: np.ndarray
    class_name: str = "yellow_cube"


def test_ros_image_adapters_respect_encoding_stride_and_scale() -> None:
    rgb_padded = bytes([1, 2, 3, 4, 5, 6, 99, 99])
    np.testing.assert_array_equal(
        image_to_bgr(rgb_padded, 1, 2, 8, "rgb8"),
        [[[3, 2, 1], [6, 5, 4]]],
    )
    depth = np.array([[250, 500, 999]], dtype="<u2")
    result = ros_depth_to_meters(
        depth.tobytes(), 1, 2, 6, "16UC1", False, 0.001
    )
    np.testing.assert_allclose(result, [[0.25, 0.5]])


def test_project_plumb_bob_and_transform_point() -> None:
    pixel = project_plumb_bob(
        [0.0, 0.0, 1.0],
        [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0],
        [0.1, -0.2, 0.0, 0.0, 0.0],
    )
    assert pixel == (640.0, 360.0)
    np.testing.assert_allclose(
        transform_point([1, 0, 0], [1, 2, 3], [0, 0, np.sqrt(0.5), np.sqrt(0.5)]),
        [1, 3, 3],
        atol=1e-12,
    )


def test_coarse_match_prefers_optical_centre_over_3d_hint_distance() -> None:
    near_mask = np.zeros((20, 20), dtype=bool)
    near_mask[9:12, 9:12] = True
    far_mask = np.zeros((20, 20), dtype=bool)
    far_mask[2:5, 2:5] = True
    detections = [
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 0.99, far_mask),
        Detection(True, "depth_verified", (0.0, 0.0, 0.9), 0.80, near_mask),
        Detection(False, "rejected", (0.0, 0.0, 0.5), 1.0, near_mask),
    ]
    match = match_target_detection(detections, (10.0, 10.0), (0.0, 0.0, 0.5), 5.0)
    assert match is not None
    assert match.detection is detections[1]
    assert match.mask_distance_px == 0.0
    assert np.isclose(match.position_distance_m, 0.4)


def test_coarse_match_rejects_unknown_and_low_confidence_targets() -> None:
    centered_mask = np.zeros((20, 20), dtype=bool)
    centered_mask[9:12, 9:12] = True
    valid_mask = np.zeros((20, 20), dtype=bool)
    valid_mask[13:16, 13:16] = True
    detections = [
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 0.99,
                  centered_mask, "unknown_object"),
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 0.69,
                  centered_mask, "bowl"),
        Detection(True, "depth_verified", (0.0, 0.0, 0.6), 0.91,
                  valid_mask, "yellow_cube"),
    ]
    match = match_target_detection(
        detections,
        (10.0, 10.0),
        (0.0, 0.0, 0.5),
        10.0,
        ("yellow_cube", "zucchini", "bowl"),
        0.70,
    )
    assert match is not None
    assert match.detection is detections[2]


def test_coarse_match_fails_closed_when_only_unsafe_candidates_exist() -> None:
    mask = np.ones((4, 4), dtype=bool)
    detections = [
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 1.0,
                  mask, "not_a_task_class"),
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 0.40,
                  mask, "yellow_cube"),
    ]
    assert match_target_detection(
        detections,
        (2.0, 2.0),
        (0.0, 0.0, 0.5),
        10.0,
        ("yellow_cube", "zucchini", "bowl"),
        0.70,
    ) is None


def test_coarse_candidate_diagnostics_report_every_rejection_gate() -> None:
    centered_mask = np.zeros((20, 20), dtype=bool)
    centered_mask[9:12, 9:12] = True
    far_mask = np.zeros((20, 20), dtype=bool)
    far_mask[0:2, 0:2] = True
    detections = [
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 0.62,
                  centered_mask, "zucchini"),
        Detection(False, "rejected", None, 0.95,
                  far_mask, "unknown_object"),
    ]
    diagnostics = target_detection_diagnostics(
        detections,
        (10.0, 10.0),
        5.0,
        ("yellow_cube", "zucchini", "bowl"),
        0.70,
    )
    assert len(diagnostics) == 2
    assert "class=zucchini" in diagnostics[0]
    assert "confidence=0.620" in diagnostics[0]
    assert "confidence_below_min(0.620<0.700)" in diagnostics[0]
    assert "runtime_rejected" in diagnostics[1]
    assert "status=rejected" in diagnostics[1]
    assert "missing_3d_position" in diagnostics[1]
    assert "class_not_allowed" in diagnostics[1]
    assert "mask_too_far" in diagnostics[1]


def test_fine_reacquisition_uses_class_and_optical_centre_not_3d_hint() -> None:
    centered_mask = np.zeros((20, 20), dtype=bool)
    centered_mask[9:12, 9:12] = True
    off_center_mask = np.zeros((20, 20), dtype=bool)
    off_center_mask[1:4, 1:4] = True
    detections = [
        Detection(True, "depth_verified", (9.0, 9.0, 9.0), 0.70, centered_mask),
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 0.99, off_center_mask),
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 1.00,
                  centered_mask, "zucchini"),
    ]
    selected = select_class_mask_near_pixel(
        detections, "yellow_cube", (10.0, 10.0)
    )
    assert selected is detections[0]


def test_fine_reacquisition_accepts_rgb_mask_without_object_depth() -> None:
    mask = np.ones((4, 4), dtype=bool)
    detections = [
        Detection(True, "depth_verified", (0.0, 0.0, 0.5), 0.9,
                  mask, "bowl"),
        Detection(False, "rejected", None, 1.0, mask),
    ]
    assert select_class_mask_near_pixel(
        detections, "yellow_cube", (2.0, 2.0)
    ) is detections[1]


def test_fine_reacquisition_rejects_empty_masks() -> None:
    detection = Detection(
        True,
        "depth_verified",
        (0.0, 0.0, 0.5),
        1.0,
        np.zeros((4, 4), dtype=bool),
    )
    assert select_class_mask_near_pixel(
        [detection], "yellow_cube", (2.0, 2.0)
    ) is None


def test_ground_ransac_rejects_object_points_and_obeys_gravity() -> None:
    rng = np.random.default_rng(4)
    ground = np.column_stack(
        (rng.uniform(-0.4, 0.4, 800), rng.uniform(-0.3, 0.3, 800),
         rng.normal(-0.2, 0.001, 800))
    )
    object_points = rng.uniform([-0.05, -0.05, -0.17], [0.05, 0.05, -0.12], (100, 3))
    normal, offset, inliers = fit_ground_plane_ransac(
        np.vstack((ground, object_points)), [0, 0, 1], seed=7
    )
    np.testing.assert_allclose(normal, [0, 0, 1], atol=0.01)
    assert np.isclose(offset, 0.2, atol=0.003)
    assert inliers.sum() > 750


def test_ray_plane_intersection_and_metric_square_fit() -> None:
    directions = np.array([[0, 0, -1], [0.1, 0, -1]], dtype=np.float64)
    points = intersect_rays_with_plane([0, 0, 1], directions, [0, 0, 1], 0)
    np.testing.assert_allclose(points, [[0, 0, 0], [0.1, 0, 0]], atol=1e-12)
    angle = np.deg2rad(27.0)
    edge = np.array([np.cos(angle), np.sin(angle), 0.0])
    perpendicular = np.array([-edge[1], edge[0], 0.0])
    center = np.array([0.3, -0.1, -0.15])
    samples = []
    for coordinate in np.linspace(-0.025, 0.025, 50):
        samples.extend(
            [center + coordinate * edge - 0.025 * perpendicular,
             center + coordinate * edge + 0.025 * perpendicular,
             center - 0.025 * edge + coordinate * perpendicular,
             center + 0.025 * edge + coordinate * perpendicular]
        )
    fitted_center, fitted_edge, corners = fit_square_on_plane(samples, [0, 0, 1])
    np.testing.assert_allclose(fitted_center, center, atol=1e-5)
    assert abs(fitted_edge.dot(edge)) > 0.999
    assert corners.shape == (4, 3)


def test_undistorted_ray_at_principal_point_is_optical_z() -> None:
    ray = undistorted_rays([[640, 360]], [900, 0, 640, 0, 900, 360, 0, 0, 1], [0, 0, 0, 0, 0])
    np.testing.assert_allclose(ray[0], [0, 0, 1], atol=1e-12)


def test_zucchini_middle_axis_fit_ignores_curved_ends() -> None:
    import cv2

    mask = np.zeros((480, 848), dtype=np.uint8)
    points = np.array(
        [[300, 215], [390, 225], [500, 270], [535, 295],
         [520, 315], [405, 265], [300, 250]], dtype=np.int32
    )
    cv2.fillPoly(mask, [points], 1)
    center, axis, segment, length, width = fit_zucchini_axis_on_plane(
        mask,
        [0.0, 0.0, 1.0],
        np.diag([1.0, -1.0, -1.0]),
        [600.0, 0.0, 424.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
        [0, 0, 0, 0, 0],
        [0, 0, 1],
        0.0,
    )
    assert center.shape == (3,)
    assert segment.shape == (2, 3)
    assert abs(axis[0]) > 0.85
    assert length > width > 0.0
