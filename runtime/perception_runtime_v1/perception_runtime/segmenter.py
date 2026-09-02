from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

from .models import RawDetection


class YoloSegmenter:
    def __init__(self, model_path: str | Path, config: dict[str, Any], cache_dir: str | Path):
        # A launcher or integrator may keep the release directory read-only.
        # Honour its writable cache location before touching the package tree.
        cache_dir = Path(os.environ.get("YOLO_CONFIG_DIR", str(cache_dir))).resolve()
        cache_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("YOLO_CONFIG_DIR", str(cache_dir))
        # Import only after YOLO_CONFIG_DIR is writable.
        from ultralytics import YOLO

        self.model_path = Path(model_path).resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"Model not found: {self.model_path}")
        self.config = config
        self.device = self._choose_device(str(config["model"].get("device", "auto")))
        self.model = YOLO(str(self.model_path))

    @staticmethod
    def _choose_device(requested: str) -> str:
        if requested != "auto":
            return requested
        return "0" if torch.cuda.is_available() else "cpu"

    def predict(self, color_bgr: np.ndarray) -> list[RawDetection]:
        model_cfg = self.config["model"]
        result = self.model.predict(
            source=color_bgr,
            imgsz=int(model_cfg["imgsz"]),
            conf=float(model_cfg["confidence"]),
            iou=float(model_cfg["nms_iou"]),
            max_det=int(model_cfg["maximum_detections"]),
            device=self.device,
            retina_masks=True,  # Explicit every call: Ultralytics keeps prior call arguments.
            verbose=False,
        )[0]
        if result.boxes is None or result.masks is None:
            return []

        height, width = color_bgr.shape[:2]
        masks = result.masks.data.detach().cpu().numpy()
        classes = result.boxes.cls.detach().cpu().numpy().astype(int)
        confidences = result.boxes.conf.detach().cpu().numpy()
        boxes = result.boxes.xyxy.detach().cpu().numpy()
        names = result.names
        detections: list[RawDetection] = []
        for index, (mask, class_id, confidence, box) in enumerate(
            zip(masks, classes, confidences, boxes), start=1
        ):
            if mask.shape != (height, width):
                mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
            mask_bool = mask >= float(model_cfg["mask_threshold"])
            x1, y1, x2, y2 = box
            bbox = (
                max(0, int(round(x1))),
                max(0, int(round(y1))),
                min(width - 1, int(round(x2))),
                min(height - 1, int(round(y2))),
            )
            detections.append(
                RawDetection(
                    instance_id=index,
                    class_id=int(class_id),
                    class_name=str(names[int(class_id)]),
                    confidence=float(confidence),
                    mask=mask_bool,
                    bbox_xyxy=bbox,
                )
            )
        return detections
