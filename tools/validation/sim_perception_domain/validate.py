#!/usr/bin/env python3
"""Offline MuJoCo-to-frozen-YOLO domain-gap validation."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw
import yaml


ROOT = Path(__file__).resolve().parents[3]
SIM_ROOT = ROOT / "simulation" / "d1_mujoco_sim"
RUNTIME_ROOT = ROOT / "runtime" / "perception_runtime_v1"
sys.path.insert(0, str(SIM_ROOT / "src"))
sys.path.insert(0, str(RUNTIME_ROOT))

from d1_mujoco_sim.model import build_model  # noqa: E402
from d1_mujoco_sim.ros_camera import (  # noqa: E402
    apply_distortion,
    distortion_source_indices,
)
from d1_mujoco_sim.scene import OBJECT_NAMES  # noqa: E402
from d1_mujoco_sim.simulator import D1Simulator  # noqa: E402


TARGET_LOCAL_CENTERS_M = {
    "yellow_cube": np.array([-0.000787, -0.000889, 0.025]),
    "bowl": np.array([-0.00023, -0.00005, 0.025]),
    "zucchini": np.array([0.00047, -0.00478, 0.01656]),
}
VIEW_SPECS = {
    "oblique": {"elevation_deg": 65.0, "distance_m": 0.60},
    "top_down": {"elevation_deg": 90.0, "distance_m": 0.45},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "artifacts" / "latest",
    )
    parser.add_argument(
        "--generate-only",
        action="store_true",
        help="Render RGB and GT masks without loading the perception model",
    )
    parser.add_argument(
        "--inference-only",
        action="store_true",
        help="Reuse existing RGB/GT files and only run model evaluation",
    )
    args = parser.parse_args()
    if args.generate_only and args.inference_only:
        parser.error("--generate-only and --inference-only are mutually exclusive")
    return args


def load_sim_config() -> dict[str, Any]:
    return yaml.safe_load(
        (SIM_ROOT / "config" / "sim.yaml").read_text(encoding="utf-8")
    )


def build_scene() -> tuple[mujoco.MjModel, mujoco.MjData, dict[str, Any]]:
    config = load_sim_config()
    controller = dict(config["controller"])
    controller["physics_timestep_s"] = config["simulation"][
        "physics_timestep_s"
    ]
    model = build_model(
        ROOT / "src" / "d1_description",
        controller,
        simulation=config["simulation"],
        scene=config["scene"],
        objects_root=ROOT / "simulation" / "objects",
        camera=config["camera"],
        mobile_base=config["mobile_base"],
    )
    simulator = D1Simulator(model, config)
    # Two simulated seconds are sufficient for the fixed-seed objects to settle.
    for _ in range(round(2.0 / model.opt.timestep)):
        simulator.step()
    return model, simulator.data, config


def object_center_world(
    model: mujoco.MjModel, data: mujoco.MjData, object_name: str
) -> np.ndarray:
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, f"object_{object_name}"
    )
    rotation = data.xmat[body_id].reshape(3, 3)
    return data.xpos[body_id] + rotation @ TARGET_LOCAL_CENTERS_M[object_name]


def optical_pose_world(target: np.ndarray, view_name: str) -> tuple[np.ndarray, np.ndarray]:
    spec = VIEW_SPECS[view_name]
    elevation = np.deg2rad(spec["elevation_deg"])
    # Horizontal target-to-base direction. It reproduces the preferred
    # observation side while remaining independent from the object's yaw.
    horizontal = np.array([-target[0], -target[1], 0.0], dtype=np.float64)
    norm = np.linalg.norm(horizontal)
    horizontal = horizontal / norm if norm > 1e-9 else np.array([-1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    target_to_camera = np.cos(elevation) * horizontal + np.sin(elevation) * up
    position = target + float(spec["distance_m"]) * target_to_camera
    forward = -target_to_camera  # RealSense optical +Z.

    image_up = up - np.dot(up, forward) * forward
    if np.linalg.norm(image_up) < 1e-8:
        image_up = horizontal
    image_up /= np.linalg.norm(image_up)
    optical_y = -image_up  # Optical +Y points down in the image.
    optical_x = np.cross(optical_y, forward)
    optical_x /= np.linalg.norm(optical_x)
    optical_y = np.cross(forward, optical_x)
    optical_y /= np.linalg.norm(optical_y)
    rotation = np.column_stack((optical_x, optical_y, forward))
    return position, rotation


def set_camera_pose_world(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
    optical_position: np.ndarray,
    optical_rotation: np.ndarray,
) -> None:
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
    )
    body_id = int(model.cam_bodyid[camera_id])
    body_rotation = data.xmat[body_id].reshape(3, 3)
    body_position = data.xpos[body_id]

    # MuJoCo camera axes are +X right, +Y up, -Z forward.
    optical_from_mujoco = np.diag([1.0, -1.0, -1.0])
    world_camera_rotation = optical_rotation @ optical_from_mujoco
    local_position = body_rotation.T @ (optical_position - body_position)
    local_rotation = body_rotation.T @ world_camera_rotation
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, local_rotation.reshape(-1))
    model.cam_pos[camera_id] = local_position
    model.cam_quat[camera_id] = quaternion
    mujoco.mj_forward(model, data)


def distort_mask(mask: np.ndarray, indices: np.ndarray | None) -> np.ndarray:
    return apply_distortion(mask[..., None].astype(np.uint8), indices)[..., 0] > 0


def render_case(
    renderer: mujoco.Renderer,
    render_options: mujoco.MjvOption,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
    object_name: str,
    view_name: str,
    distortion_indices: np.ndarray | None,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    target = object_center_world(model, data, object_name)
    position, rotation = optical_pose_world(target, view_name)
    set_camera_pose_world(model, data, camera_name, position, rotation)

    renderer.disable_segmentation_rendering()
    renderer.update_scene(data, camera=camera_name, scene_option=render_options)
    rgb = renderer.render().copy()

    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=camera_name, scene_option=render_options)
    segmentation = renderer.render().copy()
    renderer.disable_segmentation_rendering()

    if segmentation.ndim != 3 or segmentation.shape[2] != 2:
        raise RuntimeError(f"Unexpected MuJoCo segmentation shape: {segmentation.shape}")
    # Renderer returns [model object id, mjtObj type] for each visible pixel.
    masks: dict[str, np.ndarray] = {}
    for name in OBJECT_NAMES:
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"object_visual_{name}"
        )
        mask = (
            (segmentation[..., 0] == geom_id)
            & (segmentation[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
        )
        masks[name] = distort_mask(mask, distortion_indices)
    if not np.any(masks[object_name]):
        labels, counts = np.unique(segmentation.reshape(-1, 2), axis=0, return_counts=True)
        visible = sorted(
            ((pair.tolist(), int(count)) for pair, count in zip(labels, counts)),
            key=lambda item: item[1],
            reverse=True,
        )[:12]
        raise RuntimeError(
            f"No GT pixels for {object_name}/{view_name}; "
            f"largest segmentation labels={visible}"
        )
    return apply_distortion(rgb, distortion_indices), masks


def save_generated_cases(output_dir: Path) -> list[dict[str, Any]]:
    images_dir = output_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)
    model, data, config = build_scene()
    color = config["camera"]["color"]
    model.vis.global_.offwidth = int(color["width"])
    model.vis.global_.offheight = int(color["height"])
    renderer = mujoco.Renderer(
        model, height=int(color["height"]), width=int(color["width"])
    )
    render_options = mujoco.MjvOption()
    render_options.geomgroup[1] = 0  # Arm, Go2, camera, and camera mount visuals.
    render_options.geomgroup[3] = 0  # Diagnostic collision/site geometry.
    distortion_indices = distortion_source_indices(color)
    manifest: list[dict[str, Any]] = []
    try:
        for object_name in OBJECT_NAMES:
            for view_name in VIEW_SPECS:
                case_name = f"{object_name}__{view_name}"
                rgb, gt_masks = render_case(
                    renderer,
                    render_options,
                    model,
                    data,
                    str(color["name"]),
                    object_name,
                    view_name,
                    distortion_indices,
                )
                rgb_path = images_dir / f"{case_name}_rgb.png"
                gt_path = images_dir / f"{case_name}_gt.png"
                Image.fromarray(rgb).save(rgb_path)
                Image.fromarray((gt_masks[object_name] * 255).astype(np.uint8)).save(
                    gt_path
                )
                gt_by_class: dict[str, str] = {}
                for class_name, class_mask in gt_masks.items():
                    class_gt_path = images_dir / f"{case_name}_gt_{class_name}.png"
                    Image.fromarray((class_mask * 255).astype(np.uint8)).save(
                        class_gt_path
                    )
                    gt_by_class[class_name] = str(
                        class_gt_path.relative_to(output_dir)
                    )
                manifest.append(
                    {
                        "case": case_name,
                        "target_class": object_name,
                        "view": view_name,
                        "rgb": str(rgb_path.relative_to(output_dir)),
                        "gt": str(gt_path.relative_to(output_dir)),
                        "gt_by_class": gt_by_class,
                        "gt_pixels": int(gt_masks[object_name].sum()),
                    }
                )
                print(
                    f"rendered {case_name}: "
                    f"{int(gt_masks[object_name].sum())} GT pixels"
                )
    finally:
        renderer.close()
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return manifest


def mask_metrics(gt: np.ndarray, prediction: np.ndarray) -> tuple[float, float]:
    intersection = np.logical_and(gt, prediction).sum()
    union = np.logical_or(gt, prediction).sum()
    iou = float(intersection / union) if union else 0.0
    gt_y, gt_x = np.nonzero(gt)
    pred_y, pred_x = np.nonzero(prediction)
    if not len(gt_x) or not len(pred_x):
        return iou, float("nan")
    centroid_error = float(
        np.hypot(gt_x.mean() - pred_x.mean(), gt_y.mean() - pred_y.mean())
    )
    return iou, centroid_error


def boundary(mask: np.ndarray) -> np.ndarray:
    interior = mask.copy()
    interior[1:, :] &= mask[:-1, :]
    interior[:-1, :] &= mask[1:, :]
    interior[:, 1:] &= mask[:, :-1]
    interior[:, :-1] &= mask[:, 1:]
    return mask & ~interior


def make_overlay(
    rgb: np.ndarray,
    gt: np.ndarray,
    prediction: np.ndarray,
    title: str,
) -> Image.Image:
    canvas = rgb.copy()
    canvas[boundary(gt)] = [0, 255, 255]
    canvas[boundary(prediction)] = [255, 0, 255]
    image = Image.fromarray(canvas)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, 34), fill=(20, 20, 20))
    draw.text((12, 10), title, fill=(255, 255, 255))
    return image


def evaluate_cases(output_dir: Path) -> list[dict[str, Any]]:
    os.environ.setdefault("MPLCONFIGDIR", str(output_dir / "matplotlib_cache"))
    from perception_runtime.config import load_config
    from perception_runtime.segmenter import YoloSegmenter

    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    runtime_config_path = RUNTIME_ROOT / "configs" / "runtime.yaml"
    runtime_config = load_config(runtime_config_path, "arm")
    segmenter = YoloSegmenter(
        RUNTIME_ROOT / "weights" / "best.pt",
        runtime_config,
        output_dir / "yolo_cache",
    )
    results: list[dict[str, Any]] = []
    for case in manifest:
        rgb = np.asarray(Image.open(output_dir / case["rgb"]).convert("RGB"))
        gt = np.asarray(Image.open(output_dir / case["gt"]).convert("L")) > 0
        gt_by_class = {
            class_name: np.asarray(
                Image.open(output_dir / path).convert("L")
            )
            > 0
            for class_name, path in case["gt_by_class"].items()
        }
        detections = segmenter.predict(rgb[..., ::-1].copy())
        candidates = [d for d in detections if d.class_name == case["target_class"]]
        scored = [(mask_metrics(gt, d.mask), d) for d in candidates]
        matched = max(scored, key=lambda item: item[0][0]) if scored else None
        if matched is None:
            prediction = np.zeros_like(gt)
            iou = 0.0
            centroid_error = None
            confidence = None
            detected = False
        else:
            (iou, centroid_value), detection = matched
            prediction = detection.mask
            centroid_error = None if np.isnan(centroid_value) else centroid_value
            confidence = detection.confidence
            detected = True

        case_name = case["case"]
        prediction_path = output_dir / "images" / f"{case_name}_prediction.png"
        overlay_path = output_dir / "images" / f"{case_name}_overlay.png"
        Image.fromarray((prediction * 255).astype(np.uint8)).save(prediction_path)
        title = (
            f"{case_name} | conf={confidence:.3f} IoU={iou:.3f} "
            f"centroid={centroid_error:.1f}px"
            if detected and centroid_error is not None
            else f"{case_name} | target class not detected"
        )
        make_overlay(rgb, gt, prediction, title).save(overlay_path)
        result = {
            **case,
            "detected": detected,
            "confidence": confidence,
            "mask_iou": iou,
            "centroid_error_px": centroid_error,
            "prediction": str(prediction_path.relative_to(output_dir)),
            "overlay": str(overlay_path.relative_to(output_dir)),
            "all_detections": [
                {
                    "class": detection.class_name,
                    "confidence": detection.confidence,
                    "bbox_xyxy": list(detection.bbox_xyxy),
                    "mask_pixels": int(detection.mask.sum()),
                    "same_class_gt_iou": mask_metrics(
                        gt_by_class[detection.class_name], detection.mask
                    )[0],
                }
                for detection in detections
            ],
        }
        results.append(result)
        confidence_text = "-" if confidence is None else f"{confidence:.3f}"
        centroid_text = "-" if centroid_error is None else f"{centroid_error:.1f}px"
        print(
            f"evaluated {case_name}: detected={detected} conf={confidence_text} "
            f"IoU={iou:.3f} centroid={centroid_text}"
        )
    write_reports(output_dir, runtime_config_path, results)
    return results


def write_reports(
    output_dir: Path, runtime_config_path: Path, results: list[dict[str, Any]]
) -> None:
    report = {
        "scope": "MuJoCo RGB asset-domain diagnostic; no arm IK/execution validation",
        "runtime_config": str(runtime_config_path.relative_to(ROOT)),
        "weights": str((RUNTIME_ROOT / "weights" / "best.pt").relative_to(ROOT)),
        "views": VIEW_SPECS,
        "cases": results,
    }
    (output_dir / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    lines = [
        "# Simulation perception domain-gap report",
        "",
        "| Target | View | Detected | Confidence | Mask IoU | Centroid error | Predictions | Zero-overlap |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        confidence = result["confidence"]
        centroid = result["centroid_error_px"]
        lines.append(
            "| {target} | {view} | {detected} | {confidence} | {iou:.3f} | {centroid} | {predictions} | {zero_overlap} |".format(
                target=result["target_class"],
                view=result["view"],
                detected="yes" if result["detected"] else "no",
                confidence="-" if confidence is None else f"{confidence:.3f}",
                iou=result["mask_iou"],
                centroid="-" if centroid is None else f"{centroid:.1f} px",
                predictions=len(result["all_detections"]),
                zero_overlap=sum(
                    detection["same_class_gt_iou"] == 0.0
                    for detection in result["all_detections"]
                ),
            )
        )
    lines.extend(
        [
            "",
            "Cyan boundary is MuJoCo GT; magenta boundary is the matched YOLO mask.",
            "No acceptance threshold is applied in this first measurement.",
        ]
    )
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if not args.inference_only:
        save_generated_cases(output_dir)
    if not args.generate_only:
        evaluate_cases(output_dir)
        print(f"report: {output_dir / 'report.md'}")


if __name__ == "__main__":
    main()
