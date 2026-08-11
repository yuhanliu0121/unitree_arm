from __future__ import annotations

from typing import Iterable

import cv2
import numpy as np

from .models import Detection3D


CLASS_COLORS = {
    "bowl": (255, 170, 20),
    "yellow_cube": (0, 220, 255),
    "zucchini": (30, 220, 40),
}
STATUS_COLORS = {
    "rgb_only_near": (0, 165, 255),
    "rgb_only_far": (0, 165, 255),
    "rejected": (40, 40, 230),
}


def _label_lines(detection: Detection3D) -> tuple[str, str]:
    status = {
        "depth_verified": "DEPTH OK",
        "rgb_only_near": "RGB NEAR",
        "rgb_only_far": "RGB ?",
        "rejected": "REJECT",
    }.get(detection.status, detection.status.upper())
    first = f"#{detection.instance_id} {detection.class_name} {detection.confidence:.2f} {status}"
    values: list[str] = []
    if detection.depth_m is not None:
        values.append(f"Z={detection.depth_m:.2f}m")
    if detection.apparent_size_m is not None:
        values.append(f"S={detection.apparent_size_m * 100:.1f}cm")
    if detection.bearing_deg is not None:
        values.append(f"yaw={detection.bearing_deg:+.1f}deg")
    if detection.reject_reasons:
        values.append(",".join(detection.reject_reasons))
    return first, "  ".join(values)


def draw_overlay(
    color_bgr: np.ndarray,
    detections: Iterable[Detection3D],
    profile_name: str,
    fps: float,
    show_rejected: bool = True,
) -> np.ndarray:
    output = color_bgr.copy()
    blend = output.copy()
    detections = list(detections)
    for detection in detections:
        if not detection.accepted and not show_rejected:
            continue
        color = CLASS_COLORS.get(detection.class_name, (220, 220, 220))
        if detection.status != "depth_verified":
            color = STATUS_COLORS.get(detection.status, color)
        mask = detection.mask.astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if detection.status == "depth_verified":
            blend[detection.mask] = color
            cv2.drawContours(output, contours, -1, color, 2)
        elif detection.accepted:
            cv2.drawContours(output, contours, -1, color, 2)
        else:
            cv2.drawContours(output, contours, -1, color, 1)
        u, v = detection.center_pixel
        cv2.drawMarker(output, (u, v), color, cv2.MARKER_CROSS, 14, 2)
        line1, line2 = _label_lines(detection)
        x1, y1, _, _ = detection.bbox_xyxy
        text_y = max(25, y1 - 28)
        cv2.putText(output, line1, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2, cv2.LINE_AA)
        if line2:
            cv2.putText(output, line2, (x1, text_y + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv2.LINE_AA)
    output = cv2.addWeighted(blend, 0.25, output, 0.75, 0)
    verified = sum(item.status == "depth_verified" for item in detections)
    rgb_only = sum(item.accepted and not item.depth_verified for item in detections)
    rejected = sum(not item.accepted for item in detections)
    header = f"{profile_name.upper()} | FPS {fps:.1f} | verified {verified} | RGB-only {rgb_only} | rejected {rejected}"
    cv2.rectangle(output, (0, 0), (min(output.shape[1], 820), 38), (20, 20, 20), -1)
    cv2.putText(output, header, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2, cv2.LINE_AA)
    return output


def colorize_depth(depth_m: np.ndarray, maximum_m: float, detections: Iterable[Detection3D]) -> np.ndarray:
    normalized = np.clip(depth_m / max(maximum_m, 0.01), 0.0, 1.0)
    depth_u8 = np.uint8((1.0 - normalized) * 255.0)
    colored = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
    colored[depth_m <= 0.0] = 0
    for detection in detections:
        color = CLASS_COLORS.get(detection.class_name, (255, 255, 255))
        if detection.status != "depth_verified":
            color = STATUS_COLORS.get(detection.status, color)
        contours, _ = cv2.findContours(
            detection.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(colored, contours, -1, color, 2 if detection.accepted else 1)
    cv2.putText(colored, f"Aligned depth 0-{maximum_m:.1f} m", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return colored


def compose_view(
    overlay: np.ndarray,
    depth_color: np.ndarray,
    view_mode: str = "split",
    display_width: int = 1600,
) -> np.ndarray:
    if view_mode == "overlay":
        canvas = overlay
    elif view_mode == "depth":
        canvas = depth_color
    else:
        height = min(overlay.shape[0], depth_color.shape[0])
        left = cv2.resize(overlay, (round(overlay.shape[1] * height / overlay.shape[0]), height))
        right = cv2.resize(depth_color, (round(depth_color.shape[1] * height / depth_color.shape[0]), height))
        canvas = np.hstack((left, right))
    if display_width > 0 and canvas.shape[1] > display_width:
        scale = display_width / canvas.shape[1]
        canvas = cv2.resize(canvas, (display_width, round(canvas.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    return canvas
