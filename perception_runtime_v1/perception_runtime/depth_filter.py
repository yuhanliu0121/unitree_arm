from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from .geometry import deproject_pixels, optical_bearing_deg, robust_pca_dimensions
from .models import CameraIntrinsics, Detection3D, RawDetection


@dataclass(slots=True)
class DepthFilter:
    config: dict[str, Any]

    def _core_mask(self, mask: np.ndarray) -> np.ndarray:
        """Select an adaptive interior without destroying tiny distant masks."""
        mask_u8 = mask.astype(np.uint8)
        mask_pixels = int(mask_u8.sum())
        if mask_pixels == 0:
            return mask.astype(bool)
        distance = cv2.distanceTransform(mask_u8, cv2.DIST_L2, 5)
        max_distance = float(distance.max())
        minimum = int(self.config["depth_filter"]["minimum_core_pixels"])
        ratio = float(self.config["depth_filter"]["core_distance_ratio"])
        if max_distance <= 1.25:
            return mask.astype(bool)
        core = distance >= max(1.0, max_distance * ratio)
        if int(core.sum()) < min(minimum, mask_pixels):
            kernel = np.ones((3, 3), np.uint8)
            eroded = cv2.erode(mask_u8, kernel, iterations=1).astype(bool)
            if int(eroded.sum()) >= min(minimum, mask_pixels):
                core = eroded
            else:
                core = mask.astype(bool)
        return core.astype(bool)

    def _empty_result(
        self,
        detection: RawDetection,
        core: np.ndarray,
        status: str,
        accepted: bool,
        reasons: list[str],
        mask_pixels: int,
        mask_fraction: float,
    ) -> Detection3D:
        ys, xs = np.nonzero(detection.mask)
        center = (int(np.median(xs)), int(np.median(ys))) if xs.size else (0, 0)
        return Detection3D(
            instance_id=detection.instance_id,
            class_id=detection.class_id,
            class_name=detection.class_name,
            confidence=detection.confidence,
            accepted=accepted,
            status=status,
            reject_reasons=tuple(reasons),
            bbox_xyxy=detection.bbox_xyxy,
            center_pixel=center,
            depth_m=None,
            depth_valid_ratio=0.0,
            depth_spread_m=None,
            position_camera_m=None,
            dimensions_m=None,
            longest_size_m=None,
            apparent_size_m=None,
            bearing_deg=None,
            mask_pixels=mask_pixels,
            mask_fraction=mask_fraction,
            core_pixels=int(core.sum()),
            mask=detection.mask,
            core_mask=core,
        )

    def measure(
        self,
        detection: RawDetection,
        depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> Detection3D:
        mask = detection.mask.astype(bool)
        if mask.shape != depth_m.shape:
            raise ValueError(f"mask/depth shape mismatch: {mask.shape} vs {depth_m.shape}")

        profile = self.config["profile"]
        depth_cfg = self.config["depth_filter"]
        mask_pixels = int(mask.sum())
        frame_pixels = int(mask.size)
        mask_fraction = mask_pixels / max(frame_pixels, 1)
        core = self._core_mask(mask)
        reasons: list[str] = []

        if mask_pixels < int(profile["minimum_mask_pixels"]):
            reasons.append("mask_too_small")
            return self._empty_result(
                detection, core, "rejected", False, reasons, mask_pixels, mask_fraction
            )

        sensor_min = float(depth_cfg["sensor_valid_min_m"])
        sensor_max = float(depth_cfg["sensor_valid_max_m"])
        valid = core & np.isfinite(depth_m) & (depth_m >= sensor_min) & (depth_m <= sensor_max)
        valid_pixels = int(valid.sum())
        core_pixels = int(core.sum())
        valid_ratio = valid_pixels / max(core_pixels, 1)

        if valid_pixels == 0:
            if mask_fraction >= float(profile["near_mask_fraction"]):
                return self._empty_result(
                    detection, core, "rgb_only_near", True, [], mask_pixels, mask_fraction
                )
            keep = bool(profile["keep_far_unconfirmed"])
            return self._empty_result(
                detection,
                core,
                "rgb_only_far" if keep else "rejected",
                keep,
                [] if keep else ["no_valid_depth_and_small_mask"],
                mask_pixels,
                mask_fraction,
            )

        ys, xs = np.nonzero(valid)
        zs = depth_m[ys, xs].astype(np.float32)
        low_q = float(depth_cfg["point_percentile_low"])
        high_q = float(depth_cfg["point_percentile_high"])
        z_low, z_high = np.percentile(zs, [low_q, high_q])
        robust = (zs >= z_low) & (zs <= z_high)
        xs, ys, zs = xs[robust], ys[robust], zs[robust]
        depth_value = float(np.median(zs))
        depth_spread = float(np.percentile(zs, 95) - np.percentile(zs, 5))
        x1, y1, x2, y2 = detection.bbox_xyxy
        apparent_width_m = max(x2 - x1, 1) * depth_value / intrinsics.fx
        apparent_height_m = max(y2 - y1, 1) * depth_value / intrinsics.fy
        apparent_size_m = float(max(apparent_width_m, apparent_height_m))

        max_points = int(depth_cfg["maximum_point_samples"])
        if xs.size > max_points:
            take = np.linspace(0, xs.size - 1, max_points, dtype=np.int64)
            xs, ys, zs = xs[take], ys[take], zs[take]
        points = deproject_pixels(xs, ys, zs, intrinsics)

        position: tuple[float, float, float] | None = None
        dimensions: tuple[float, float, float] | None = None
        longest: float | None = None
        center_pixel = (int(np.median(xs)), int(np.median(ys)))
        if points.shape[0] >= int(depth_cfg["minimum_size_points"]):
            center, dims, _ = robust_pca_dimensions(
                points,
                float(depth_cfg["dimension_percentile_low"]),
                float(depth_cfg["dimension_percentile_high"]),
            )
            position = tuple(float(v) for v in center)
            dimensions = tuple(float(v) for v in dims)
            longest = float(dims[0])

        depth_usable = (
            valid_ratio >= float(profile["minimum_valid_depth_ratio"])
            and points.shape[0] >= int(depth_cfg["minimum_size_points"])
            and longest is not None
        )

        if depth_usable:
            if not (float(profile["minimum_depth_m"]) <= depth_value <= float(profile["maximum_depth_m"])):
                reasons.append("outside_work_range")
            size_range = self.config["classes"].get(detection.class_name, {}).get("longest_size_range_m")
            # Gate with the 2-D mask extent projected at robust median depth.  A raw
            # 3-D PCA extent is kept for later pose work, but must not veto a real
            # object when a few mask-edge pixels land on the supporting floor.
            if size_range and not (
                float(size_range[0]) <= apparent_size_m <= float(size_range[1])
            ):
                reasons.append("size_out_of_range")
            status = "rejected" if reasons else "depth_verified"
            accepted = not reasons
        else:
            # Sparse depth can still reject a wall-sized blob using its apparent image size,
            # but it must not trim a real slanted zucchini with a fixed depth band.
            size_range = self.config["classes"].get(detection.class_name, {}).get("longest_size_range_m")
            observed_px = max(x2 - x1, y2 - y1)
            if size_range and depth_value > 0.05:
                plausible_px = float(size_range[1]) * intrinsics.fx / depth_value
                if observed_px > plausible_px * float(depth_cfg["apparent_size_tolerance"]):
                    reasons.append("too_large_for_distance")
            if reasons:
                status, accepted = "rejected", False
            elif mask_fraction >= float(profile["near_mask_fraction"]):
                status, accepted = "rgb_only_near", True
            else:
                accepted = bool(profile["keep_far_unconfirmed"])
                status = "rgb_only_far" if accepted else "rejected"
                if not accepted:
                    reasons.append("depth_quality_too_low")

        bearing = optical_bearing_deg(position) if position is not None else None
        return Detection3D(
            instance_id=detection.instance_id,
            class_id=detection.class_id,
            class_name=detection.class_name,
            confidence=detection.confidence,
            accepted=accepted,
            status=status,
            reject_reasons=tuple(reasons),
            bbox_xyxy=detection.bbox_xyxy,
            center_pixel=center_pixel,
            depth_m=depth_value,
            depth_valid_ratio=float(valid_ratio),
            depth_spread_m=depth_spread,
            position_camera_m=position,
            dimensions_m=dimensions,
            longest_size_m=longest,
            apparent_size_m=apparent_size_m,
            bearing_deg=bearing,
            mask_pixels=mask_pixels,
            mask_fraction=mask_fraction,
            core_pixels=core_pixels,
            mask=mask,
            core_mask=core,
            point_cloud_camera_m=points,
        )
