from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from perception_runtime.config import load_config
from perception_runtime.depth_filter import DepthFilter
from perception_runtime.models import CameraIntrinsics, RawDetection


ROOT = Path(__file__).resolve().parent.parent


def rectangle_detection(name: str, x1: int, y1: int, x2: int, y2: int) -> RawDetection:
    mask = np.zeros((240, 320), dtype=bool)
    mask[y1:y2, x1:x2] = True
    return RawDetection(1, {"bowl": 0, "yellow_cube": 1, "zucchini": 2}[name], name, 0.9, mask, (x1, y1, x2, y2))


class DepthFilterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.intrinsics = CameraIntrinsics(400.0, 400.0, 160.0, 120.0, 320, 240)

    def test_plausible_bowl_is_verified(self) -> None:
        config = load_config(ROOT / "configs" / "runtime.yaml", "dog")
        result = DepthFilter(config).measure(
            rectangle_detection("bowl", 136, 96, 184, 144),
            np.ones((240, 320), dtype=np.float32),
            self.intrinsics,
        )
        self.assertTrue(result.accepted)
        self.assertEqual(result.status, "depth_verified")
        self.assertAlmostEqual(result.depth_m or 0.0, 1.0, places=3)

    def test_wall_sized_bowl_is_rejected(self) -> None:
        config = load_config(ROOT / "configs" / "runtime.yaml", "dog")
        result = DepthFilter(config).measure(
            rectangle_detection("bowl", 20, 20, 300, 220),
            np.ones((240, 320), dtype=np.float32),
            self.intrinsics,
        )
        self.assertFalse(result.accepted)
        self.assertIn("size_out_of_range", result.reject_reasons)

    def test_floor_depth_edge_does_not_reject_real_cube(self) -> None:
        config = load_config(ROOT / "configs" / "runtime.yaml", "dog")
        detection = rectangle_detection("yellow_cube", 151, 110, 169, 130)
        depth = np.zeros((240, 320), dtype=np.float32)
        # Mimic a far cube mask whose lower part leaks onto a sloping floor.
        depth[110:120, 151:169] = 1.80
        depth[120:130, 151:169] = 1.90
        result = DepthFilter(config).measure(detection, depth, self.intrinsics)
        self.assertTrue(result.accepted)
        self.assertEqual(result.status, "depth_verified")
        self.assertLess(result.apparent_size_m or 1.0, 0.10)

    def test_dog_keeps_small_rgb_only_candidate(self) -> None:
        config = load_config(ROOT / "configs" / "runtime.yaml", "dog")
        result = DepthFilter(config).measure(
            rectangle_detection("yellow_cube", 155, 115, 165, 125),
            np.zeros((240, 320), dtype=np.float32),
            self.intrinsics,
        )
        self.assertTrue(result.accepted)
        self.assertEqual(result.status, "rgb_only_far")

    def test_arm_rejects_small_rgb_only_candidate(self) -> None:
        config = load_config(ROOT / "configs" / "runtime.yaml", "arm")
        result = DepthFilter(config).measure(
            rectangle_detection("yellow_cube", 150, 110, 170, 130),
            np.zeros((240, 320), dtype=np.float32),
            self.intrinsics,
        )
        self.assertFalse(result.accepted)
        self.assertEqual(result.status, "rejected")


if __name__ == "__main__":
    unittest.main()
