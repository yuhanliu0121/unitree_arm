#!/usr/bin/env python3
"""Render an RGB/aligned-depth acceptance triptych from the seed-0 scene."""

from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw
import yaml

from d1_mujoco_sim.model import build_model, default_description_root
from d1_mujoco_sim.ros_camera import (
    align_depth_to_color,
    apply_distortion,
    colorize_depth,
    depth_to_uint16,
    distortion_source_indices,
)
from d1_mujoco_sim.scene import default_objects_root


ROOT = Path(__file__).resolve().parents[2]


def _set_overhead_inspection_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    color_config: dict,
) -> None:
    """Choose a deterministic arm pose that puts the task area in view."""
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"Joint{i}")
        for i in range(6)
    ]
    qpos_ids = [int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids]
    limits = np.asarray([model.jnt_range[joint_id] for joint_id in joint_ids])
    camera_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        str(color_config["name"]),
    )
    target = np.asarray([0.30, 0.0, 0.04])
    rng = np.random.default_rng(0)
    best_score = np.inf
    best_qpos = None
    for candidate in rng.uniform(limits[:, 0], limits[:, 1], (20000, 6)):
        data.qpos[qpos_ids] = candidate
        mujoco.mj_forward(model, data)
        camera_from_world = data.cam_xmat[camera_id].reshape(3, 3).T
        point_mujoco = camera_from_world @ (
            target - data.cam_xpos[camera_id]
        )
        point_optical = np.asarray(
            [point_mujoco[0], -point_mujoco[1], -point_mujoco[2]]
        )
        if point_optical[2] <= 0.2:
            continue
        pixel_x = (
            float(color_config["fx"])
            * point_optical[0]
            / point_optical[2]
            + float(color_config["cx"])
        )
        pixel_y = (
            float(color_config["fy"])
            * point_optical[1]
            / point_optical[2]
            + float(color_config["cy"])
        )
        centre_error = (
            (pixel_x - float(color_config["cx"]))
            / float(color_config["width"])
        ) ** 2 + (
            (pixel_y - float(color_config["cy"]))
            / float(color_config["height"])
        ) ** 2
        distance_error = (point_optical[2] - 0.55) ** 2
        height_error = max(0.0, 0.30 - data.cam_xpos[camera_id, 2]) ** 2
        score = centre_error + distance_error + 4.0 * height_error
        if score < best_score:
            best_score = score
            best_qpos = candidate.copy()
    if best_qpos is None:
        raise RuntimeError("Could not find a camera inspection pose")
    data.qpos[qpos_ids] = best_qpos
    mujoco.mj_forward(model, data)
    print("Inspection joint pose:", np.round(best_qpos, 5).tolist())


def _depth_edges(depth_raw: np.ndarray) -> np.ndarray:
    """Return object/surface boundaries in an aligned Z16 image."""
    valid = depth_raw > 0
    edges = np.zeros(depth_raw.shape, dtype=bool)
    horizontal = (
        valid[:, 1:] != valid[:, :-1]
    ) | (
        valid[:, 1:]
        & valid[:, :-1]
        & (np.abs(
            depth_raw[:, 1:].astype(np.int32)
            - depth_raw[:, :-1].astype(np.int32)
        ) > 20)
    )
    vertical = (
        valid[1:, :] != valid[:-1, :]
    ) | (
        valid[1:, :]
        & valid[:-1, :]
        & (np.abs(
            depth_raw[1:, :].astype(np.int32)
            - depth_raw[:-1, :].astype(np.int32)
        ) > 20)
    )
    edges[:, 1:] |= horizontal
    edges[:, :-1] |= horizontal
    edges[1:, :] |= vertical
    edges[:-1, :] |= vertical
    return edges


def main() -> None:
    config = yaml.safe_load(
        (ROOT / "d1_mujoco_sim" / "config" / "sim.yaml").read_text()
    )
    controller = dict(config["controller"])
    controller["physics_timestep_s"] = config["simulation"][
        "physics_timestep_s"
    ]
    model = build_model(
        default_description_root(),
        controller,
        simulation=config["simulation"],
        scene=config["scene"],
        objects_root=default_objects_root(),
        camera=config["camera"],
    )
    data = mujoco.MjData(model)
    color_config = config["camera"]["color"]
    depth_config = config["camera"]["depth"]
    _set_overhead_inspection_pose(model, data, color_config)
    model.vis.global_.offwidth = max(
        int(color_config["width"]),
        int(depth_config["width"]),
    )
    model.vis.global_.offheight = max(
        int(color_config["height"]),
        int(depth_config["height"]),
    )
    color_renderer = mujoco.Renderer(
        model,
        height=int(color_config["height"]),
        width=int(color_config["width"]),
    )
    depth_renderer = mujoco.Renderer(
        model,
        height=int(depth_config["height"]),
        width=int(depth_config["width"]),
    )
    depth_renderer.enable_depth_rendering()
    try:
        render_started = time.perf_counter()
        color_renderer.update_scene(data, camera=str(color_config["name"]))
        color = color_renderer.render()
        color_render_ms = 1000.0 * (time.perf_counter() - render_started)
        distortion_started = time.perf_counter()
        color = apply_distortion(
            color,
            distortion_source_indices(color_config),
        )
        distortion_ms = 1000.0 * (
            time.perf_counter() - distortion_started
        )
        depth_render_started = time.perf_counter()
        depth_renderer.update_scene(data, camera=str(depth_config["name"]))
        depth_m = depth_renderer.render()
        depth_render_ms = 1000.0 * (
            time.perf_counter() - depth_render_started
        )
    finally:
        color_renderer.close()
        depth_renderer.close()

    depth_raw = depth_to_uint16(
        depth_m,
        float(depth_config["depth_scale_m_per_unit"]),
        float(config["camera"]["min_depth_m"]),
        float(config["camera"]["max_depth_m"]),
    )
    alignment_started = time.perf_counter()
    aligned_raw = align_depth_to_color(
        depth_raw,
        depth_config,
        color_config,
        np.asarray(config["camera"]["T_color_depth_optical"]),
    )
    alignment_ms = 1000.0 * (time.perf_counter() - alignment_started)
    aligned_m = aligned_raw.astype(np.float32) * float(
        depth_config["depth_scale_m_per_unit"]
    )
    aligned_plasma = colorize_depth(
        aligned_m,
        float(config["camera"]["display_min_depth_m"]),
        float(config["camera"]["display_max_depth_m"]),
    )
    overlay = color.copy()
    overlay[_depth_edges(aligned_raw)] = [0, 255, 255]

    panel_width = 640
    panel_height = 360
    label_height = 34
    panels = []
    for label, image in (
        ("RGB", color),
        ("Aligned depth (Plasma, 0.20-2.00 m)", aligned_plasma),
        ("Depth edges on RGB (cyan)", overlay),
    ):
        panel = Image.fromarray(image).resize(
            (panel_width, panel_height),
            Image.Resampling.LANCZOS,
        )
        labelled = Image.new(
            "RGB",
            (panel_width, panel_height + label_height),
            (28, 28, 28),
        )
        labelled.paste(panel, (0, label_height))
        ImageDraw.Draw(labelled).text((10, 9), label, fill=(255, 255, 255))
        panels.append(labelled)
    triptych = Image.new(
        "RGB",
        (panel_width * len(panels), panel_height + label_height),
    )
    for index, panel in enumerate(panels):
        triptych.paste(panel, (index * panel_width, 0))

    destination = ROOT / "doc" / "wrist_camera_alignment_acceptance.png"
    triptych.save(destination)
    valid_ratio = float(np.count_nonzero(aligned_raw)) / aligned_raw.size
    print(f"Saved {destination}")
    print(
        f"Aligned depth: {aligned_raw.shape[1]}x{aligned_raw.shape[0]}, "
        f"valid={valid_ratio:.1%}, alignment={alignment_ms:.1f} ms"
    )
    print(
        f"Render timings: color={color_render_ms:.1f} ms, "
        f"distortion={distortion_ms:.1f} ms, depth={depth_render_ms:.1f} ms"
    )


if __name__ == "__main__":
    main()
