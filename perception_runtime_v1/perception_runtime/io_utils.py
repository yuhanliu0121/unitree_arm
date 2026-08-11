from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def write_image(path: str | Path, image: np.ndarray, params: list[int] | None = None) -> Path:
    """Write through imencode so Windows paths containing Chinese characters work."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower() or ".png"
    ok, encoded = cv2.imencode(suffix, image, params or [])
    if not ok:
        raise RuntimeError(f"Could not encode image: {path}")
    path.write_bytes(encoded.tobytes())
    return path


def write_json(path: str | Path, value: Any) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    return path
