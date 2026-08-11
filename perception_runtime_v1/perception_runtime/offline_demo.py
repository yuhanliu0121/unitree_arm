from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .io_utils import write_image, write_json
from .models import CameraIntrinsics
from .runtime import PerceptionRuntime
from .visualization import colorize_depth, compose_view, draw_overlay


def read_image(path: Path, flags: int) -> np.ndarray:
    data = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    image = cv2.imdecode(data, flags)
    if image is None:
        raise RuntimeError(f"Could not decode {path}")
    return image


def manifest_entry(path: Path, stem: str) -> dict:
    for line in path.read_text(encoding="utf-8").splitlines():
        value = json.loads(line)
        if value.get("stem") == stem:
            return value
    raise KeyError(f"Stem {stem!r} not found in {path}")


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Run the perception package on a saved RGB-D pair")
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--stem", required=True)
    parser.add_argument("--config", type=Path, default=root / "configs" / "runtime.yaml")
    parser.add_argument("--profile", choices=("dog", "arm"), default="dog")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image_candidates = list((args.dataset / "images" / "test").glob(f"{args.stem}.*"))
    if not image_candidates:
        raise FileNotFoundError(f"RGB image not found for {args.stem}")
    depth_path = args.dataset / "depth" / f"{args.stem}.png"
    entry = manifest_entry(args.dataset / "capture_manifest.jsonl", args.stem)
    color = read_image(image_candidates[0], cv2.IMREAD_COLOR)
    depth_raw = read_image(depth_path, cv2.IMREAD_UNCHANGED)
    depth_m = depth_raw.astype(np.float32) * float(entry["depth_scale_m"])
    intr = entry["intrinsics"]
    intrinsics = CameraIntrinsics(
        fx=float(intr["fx"]), fy=float(intr["fy"]), cx=float(intr["cx"]), cy=float(intr["cy"]),
        width=color.shape[1], height=color.shape[0],
    )
    runtime = PerceptionRuntime(args.config, args.profile, args.model)
    detections = runtime.process(color, depth_m, intrinsics)
    overlay = draw_overlay(color, detections, runtime.profile_name, 0.0, True)
    depth_color = colorize_depth(depth_m, float(runtime.config["visualization"]["depth_colormap_max_m"]), detections)
    canvas = compose_view(overlay, depth_color, "split", 1600)
    args.output.mkdir(parents=True, exist_ok=True)
    write_image(args.output / f"{args.stem}_visualization.jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 95])
    write_json(args.output / f"{args.stem}_objects.json", [item.to_dict() for item in detections])
    print(json.dumps([item.to_dict() for item in detections], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
