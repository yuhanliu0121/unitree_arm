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
    match_target_detection,
    project_plumb_bob,
    ros_depth_to_meters,
    transform_point,
)


@dataclass
class Detection:
    accepted: bool
    status: str
    position_camera_m: tuple[float, float, float] | None
    confidence: float
    mask: np.ndarray


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


def test_match_prefers_hint_mask_then_3d_distance() -> None:
    near_mask = np.zeros((20, 20), dtype=bool)
    near_mask[9:12, 9:12] = True
    far_mask = np.zeros((20, 20), dtype=bool)
    far_mask[2:5, 2:5] = True
    detections = [
        Detection(True, "depth_verified", (0.0, 0.0, 0.6), 0.99, far_mask),
        Detection(True, "depth_verified", (0.0, 0.0, 0.52), 0.80, near_mask),
        Detection(False, "rejected", (0.0, 0.0, 0.5), 1.0, near_mask),
    ]
    match = match_target_detection(detections, (10.0, 10.0), (0.0, 0.0, 0.5), 5.0)
    assert match is not None
    assert match.detection is detections[1]
    assert match.mask_distance_px == 0.0
    assert np.isclose(match.position_distance_m, 0.02)
