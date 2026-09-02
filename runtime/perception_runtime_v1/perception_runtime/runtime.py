from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from .config import load_config
from .depth_filter import DepthFilter
from .models import CameraIntrinsics, Detection3D
from .segmenter import YoloSegmenter


class PerceptionRuntime:
    """Stable team-facing API: RGB + aligned depth -> independent 3D detections."""

    def __init__(
        self,
        config_path: str | Path,
        profile: str = "dog",
        model_path: str | Path | None = None,
    ) -> None:
        self.config_path = Path(config_path).resolve()
        self.root = self.config_path.parent.parent
        self.config: dict[str, Any] = load_config(self.config_path, profile)
        configured_model = Path(self.config["model"]["path"])
        if not configured_model.is_absolute():
            configured_model = self.root / configured_model
        self.model_path = Path(model_path).resolve() if model_path else configured_model.resolve()
        self.segmenter = YoloSegmenter(
            self.model_path,
            self.config,
            self.root / ".runtime_cache" / "ultralytics",
        )
        self.depth_filter = DepthFilter(self.config)

    @property
    def profile_name(self) -> str:
        return str(self.config["profile_name"])

    def process(
        self,
        color_bgr: np.ndarray,
        aligned_depth_m: np.ndarray,
        intrinsics: CameraIntrinsics,
    ) -> list[Detection3D]:
        if color_bgr.shape[:2] != aligned_depth_m.shape:
            raise ValueError(
                f"RGB/depth must already be aligned: {color_bgr.shape[:2]} vs {aligned_depth_m.shape}"
            )
        raw = self.segmenter.predict(color_bgr)
        return [self.depth_filter.measure(item, aligned_depth_m, intrinsics) for item in raw]
