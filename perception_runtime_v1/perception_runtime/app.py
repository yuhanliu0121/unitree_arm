from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2

from .config import load_config
from .io_utils import write_image, write_json
from .realsense_source import RealSenseSource
from .runtime import PerceptionRuntime
from .visualization import colorize_depth, compose_view, draw_overlay


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Go2 RGB-D perception visualization")
    parser.add_argument("--config", type=Path, default=root / "configs" / "runtime.yaml")
    parser.add_argument("--profile", choices=("dog", "arm"), default="dog")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--view", choices=("split", "overlay", "depth"), default="split")
    parser.add_argument("--output", type=Path, default=root / "captures")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--save-first", action="store_true")
    parser.add_argument("--show-rejected", action="store_true", default=True)
    return parser.parse_args()


def save_snapshot(output: Path, frame, canvas, detections) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    target = output / stamp
    target.mkdir(parents=True, exist_ok=False)
    write_image(target / "rgb.png", frame.color_bgr)
    write_image(target / "depth_mm.png", frame.aligned_depth_raw)
    write_image(target / "visualization.jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 95])
    write_json(
        target / "objects.json",
        {
            "timestamp_ms": frame.timestamp_ms,
            "depth_scale_m": frame.depth_scale_m,
            "intrinsics": {
                "fx": frame.intrinsics.fx,
                "fy": frame.intrinsics.fy,
                "cx": frame.intrinsics.cx,
                "cy": frame.intrinsics.cy,
                "width": frame.intrinsics.width,
                "height": frame.intrinsics.height,
            },
            "objects": [item.to_dict() for item in detections],
        },
    )
    return target


def main() -> int:
    args = parse_args()
    config = load_config(args.config, args.profile)
    runtime = PerceptionRuntime(args.config, args.profile, args.model)
    camera = RealSenseSource(config["camera"])
    view_mode = args.view
    show_rejected = args.show_rejected
    paused = False
    frame_count = 0
    saved_first = False
    fps = 0.0
    last_time = time.perf_counter()
    last_frame = None
    last_canvas = None
    last_detections = []

    print(f"Model: {runtime.model_path}")
    print(f"Profile: {runtime.profile_name}; device: {runtime.segmenter.device}")
    print("Keys: Q/Esc quit, S save, D change view, R show rejected, Space pause")
    try:
        camera.start()
        print(f"Camera: {camera.device_name}; serial: {camera.serial_number}")
        while True:
            if not paused or last_frame is None:
                frame = camera.read()
                detections = runtime.process(frame.color_bgr, frame.aligned_depth_m, frame.intrinsics)
                now = time.perf_counter()
                instantaneous = 1.0 / max(now - last_time, 1e-6)
                fps = instantaneous if fps == 0.0 else 0.85 * fps + 0.15 * instantaneous
                last_time = now
                overlay = draw_overlay(
                    frame.color_bgr, detections, runtime.profile_name, fps, show_rejected
                )
                depth_color = colorize_depth(
                    frame.aligned_depth_m,
                    float(config["visualization"]["depth_colormap_max_m"]),
                    detections,
                )
                canvas = compose_view(
                    overlay,
                    depth_color,
                    view_mode,
                    int(config["visualization"]["display_width"]),
                )
                last_frame, last_canvas, last_detections = frame, canvas, detections
                frame_count += 1
                if args.save_first and not saved_first:
                    target = save_snapshot(args.output, frame, canvas, detections)
                    print(f"Saved: {target}")
                    saved_first = True
            else:
                frame, canvas, detections = last_frame, last_canvas, last_detections

            if args.headless:
                if args.max_frames and frame_count >= args.max_frames:
                    break
                continue

            cv2.imshow("Go2 Perception Runtime V1 - RGB segmentation + aligned depth", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                target = save_snapshot(args.output, frame, canvas, detections)
                print(f"Saved: {target}")
            elif key == ord("d"):
                view_mode = {"split": "overlay", "overlay": "depth", "depth": "split"}[view_mode]
            elif key == ord("r"):
                show_rejected = not show_rejected
            elif key == ord(" "):
                paused = not paused
            if args.max_frames and frame_count >= args.max_frames:
                break
    except RuntimeError as error:
        print(f"[ERROR] {error}")
        print("Check the USB cable, and close RealSense Viewer or any program using the D435i.")
        return 2
    finally:
        camera.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
