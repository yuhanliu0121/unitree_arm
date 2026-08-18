#!/usr/bin/env python3
"""Render a conservative object grasp workspace from offline scan CSV data."""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
import numpy as np


CUBE_PHYSICALLY_VALIDATED_XY = (0.28, -0.24)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--object-type",
        choices=("yellow_cube", "zucchini", "bowl"),
        default="yellow_cube",
    )
    return parser.parse_args()


def load_trials(paths: list[Path]):
    trials = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                trials.append(
                    {
                        "x": float(row["x_m"]),
                        "y": float(row["y_m"]),
                        "yaw": float(row["object_yaw_deg"]),
                        "success": row["success"] == "1",
                        "failed_stage": row["failed_stage"],
                        "raw": row,
                    }
                )
    if not trials:
        raise RuntimeError("scan CSV inputs are empty")
    return trials


def robustness_map(conservative: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    step_x = float(np.min(np.diff(xs))) if len(xs) > 1 else 0.0
    step_y = float(np.min(np.diff(ys))) if len(ys) > 1 else 0.0
    failed = np.argwhere(~conservative)
    result = np.zeros_like(conservative, dtype=float)
    for iy, ix in np.argwhere(conservative):
        point_x = xs[ix]
        point_y = ys[iy]
        distances = []
        if failed.size:
            dx = xs[failed[:, 1]] - point_x
            dy = ys[failed[:, 0]] - point_y
            distances.append(float(np.min(np.hypot(dx, dy))))
        # Outside the scanned rectangle remains unknown. Cap the reported
        # positional margin at the nearest scan boundary rather than silently
        # treating unscanned space as feasible.
        distances.extend(
            [
                point_x - (xs[0] - 0.5 * step_x),
                (xs[-1] + 0.5 * step_x) - point_x,
                point_y - (ys[0] - 0.5 * step_y),
                (ys[-1] + 0.5 * step_y) - point_y,
            ]
        )
        result[iy, ix] = max(0.0, min(distances))
    return result


def render_robot_top_view(workspace: Path) -> tuple[np.ndarray, list[float]]:
    """Render the live MuJoCo Go2+D1 assembly on a transparent XY canvas."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    sys.path.insert(0, str(workspace / "d1_mujoco_sim" / "src"))
    import mujoco
    import yaml

    from d1_mujoco_sim.model import build_model
    from d1_mujoco_sim.simulator import D1Simulator

    config = yaml.safe_load(
        (workspace / "d1_mujoco_sim" / "config" / "sim.yaml").read_text(
            encoding="utf-8"
        )
    )
    controller = dict(config["controller"])
    controller["physics_timestep_s"] = config["simulation"][
        "physics_timestep_s"
    ]
    model = build_model(
        workspace / "d1_constrained_description_20260728",
        controller,
        simulation=config["simulation"],
        camera=config["camera"],
        mobile_base=config["mobile_base"],
    )
    simulator = D1Simulator(model, config)

    # A distant, narrow-FOV top camera is effectively orthographic while still
    # using MuJoCo's real visual meshes and current stowed joint state.
    span = 1.10
    distance = 50.0
    model.vis.global_.fovy = math.degrees(2.0 * math.atan(span / (2.0 * distance)))
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, 0.0]
    camera.distance = distance
    camera.azimuth = 90.0
    camera.elevation = -90.0
    options = mujoco.MjvOption()
    options.geomgroup[:] = 0
    options.geomgroup[1] = 1

    renderer = mujoco.Renderer(model, height=480, width=480)
    renderer.update_scene(simulator.data, camera=camera, scene_option=options)
    rgb = renderer.render().copy()
    renderer.enable_segmentation_rendering()
    renderer.update_scene(simulator.data, camera=camera, scene_option=options)
    segmentation = renderer.render().copy()
    renderer.close()

    visible = segmentation[..., 0] >= 0
    go2_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "go2_visual"
    )
    arm = visible & (segmentation[..., 0] != go2_geom_id)
    expanded_arm = arm.copy()
    for _ in range(3):
        expanded_arm = np.logical_or.reduce(
            [
                expanded_arm,
                np.roll(expanded_arm, 1, axis=0),
                np.roll(expanded_arm, -1, axis=0),
                np.roll(expanded_arm, 1, axis=1),
                np.roll(expanded_arm, -1, axis=1),
            ]
        )
    arm_outline = expanded_arm & ~arm
    rgb[arm_outline] = [255, 112, 0]
    alpha = np.where(visible | arm_outline, 242, 0).astype(np.uint8)
    rgba = np.dstack((rgb, alpha))
    half = span / 2.0
    return rgba, [-half, half, -half, half]


def add_robot(ax, rgba: np.ndarray, extent: list[float]):
    ax.imshow(rgba, origin="upper", extent=extent, interpolation="bilinear", zorder=5)
    ax.scatter([0.0], [0.0], marker="o", s=45, c="black", label="arm base_link", zorder=7)


def select_recommended(
    conservative: np.ndarray, robustness: np.ndarray, xs: np.ndarray, ys: np.ndarray
):
    if not np.any(conservative):
        return None, []
    maximum = float(np.max(robustness))
    maxima = np.argwhere(np.isclose(robustness, maximum) & conservative)
    centroid = np.mean(np.array([(xs[ix], ys[iy]) for iy, ix in maxima]), axis=0)
    chosen = min(
        maxima,
        key=lambda index: math.hypot(
            xs[index[1]] - centroid[0], ys[index[0]] - centroid[1]
        ),
    )
    point = (float(xs[chosen[1]]), float(ys[chosen[0]]), maximum)
    all_maxima = [(float(xs[ix]), float(ys[iy])) for iy, ix in maxima]
    return point, all_maxima


def main() -> None:
    args = parse_args()
    slug = {"yellow_cube": "cube", "zucchini": "zucchini", "bowl": "bowl"}[
        args.object_type
    ]
    display_name = {
        "yellow_cube": "Yellow cube",
        "zucchini": "Zucchini",
        "bowl": "Bowl",
    }[args.object_type]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    trials = load_trials(args.csv)
    trials.sort(key=lambda item: (item["y"], item["x"], item["yaw"]))
    merged_csv = args.output_dir / f"{slug}_workspace.csv"
    fieldnames = list(trials[0]["raw"].keys())
    with merged_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(trial["raw"] for trial in trials)
    xs = np.array(sorted({trial["x"] for trial in trials}))
    ys = np.array(sorted({trial["y"] for trial in trials}))
    grouped = defaultdict(list)
    for trial in trials:
        grouped[(trial["x"], trial["y"])].append(trial)

    success_fraction = np.zeros((len(ys), len(xs)), dtype=float)
    conservative = np.zeros_like(success_fraction, dtype=bool)
    expected_yaw_count = max(len(value) for value in grouped.values())
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            values = grouped[(float(x), float(y))]
            successes = sum(value["success"] for value in values)
            success_fraction[iy, ix] = successes / expected_yaw_count
            conservative[iy, ix] = len(values) == expected_yaw_count and successes == expected_yaw_count

    robustness = robustness_map(conservative, xs, ys)
    recommended, maxima = select_recommended(conservative, robustness, xs, ys)
    step_x = float(np.min(np.diff(xs))) if len(xs) > 1 else 0.05
    step_y = float(np.min(np.diff(ys))) if len(ys) > 1 else 0.05
    extent = [
        xs[0] - 0.5 * step_x,
        xs[-1] + 0.5 * step_x,
        ys[0] - 0.5 * step_y,
        ys[-1] + 0.5 * step_y,
    ]

    robot_rgba, robot_extent = render_robot_top_view(Path(__file__).resolve().parents[2])
    figure, axis = plt.subplots(figsize=(9, 8), constrained_layout=True)
    fraction_image = axis.imshow(
        conservative.astype(float),
        origin="lower",
        extent=extent,
        interpolation="nearest",
        vmin=0.0,
        vmax=1.0,
        cmap=ListedColormap(["#5b5b5b", "#228b22"]),
    )
    yaw_subtitle = {
        "yellow_cube": "object yaw 0-80 deg, 10 deg step",
        "zucchini": "long-axis yaw 0-170 deg, 10 deg step",
        "bowl": "rotationally symmetric rim grasp",
    }[args.object_type]
    axis.set_title(f"{display_name} grasp zone\n{yaw_subtitle}")
    add_robot(axis, robot_rgba, robot_extent)
    axis.contour(
        xs,
        ys,
        conservative.astype(float),
        levels=[0.5],
        colors="red",
        linewidths=1.5,
    )
    axis.set_xlabel("Target X in arm base_link (m), forward +X")
    axis.set_ylabel("Target Y (m), left +Y")
    axis.set_aspect("equal")
    axis.grid(alpha=0.2)
    marker_handles, marker_labels = axis.get_legend_handles_labels()
    axis.legend(
        [
            Patch(facecolor="#228b22", edgecolor="red", label="Graspable for every sampled yaw"),
            Patch(facecolor="#5b5b5b", label="Not graspable"),
            Patch(facecolor="#c5cfd8", label="Go2 top view"),
            Patch(
                facecolor="#f0f0f0",
                edgecolor="#ff7000",
                linewidth=2.0,
                label="D1 stowed outline",
            ),
            *marker_handles,
        ],
        [
            "Graspable for every sampled yaw",
            "Not graspable",
            "Go2 top view",
            "D1 stowed outline",
            *marker_labels,
        ],
        loc="lower right",
        fontsize=8,
    )
    figure.savefig(args.output_dir / f"{slug}_sweet_zone.png", dpi=180)
    plt.close(figure)

    failure_counts = Counter(
        trial["failed_stage"] for trial in trials if not trial["success"]
    )
    conservative_points = int(np.count_nonzero(conservative))
    with (args.output_dir / f"{slug}_sweet_zone.yaml").open("w", encoding="utf-8") as stream:
        stream.write(f"{slug}_sweet_zone:\n")
        stream.write("  frame_id: base_link\n")
        stream.write(f"  xy_step_m: {step_x:.6f}\n")
        yaw_values = sorted({trial["yaw"] for trial in trials})
        stream.write(
            "  yaw_samples_degrees: ["
            + ", ".join(f"{value:g}" for value in yaw_values)
            + "]\n"
        )
        stream.write(f"  conservative_xy_point_count: {conservative_points}\n")
        if args.object_type == "yellow_cube":
            stream.write("  physically_validated_xy:\n")
            stream.write("    - [0.280000, -0.240000]\n")
        if recommended is None:
            stream.write("  recommended_target_xy: null\n")
            stream.write("  recommended_robustness_radius_m: 0.0\n")
        else:
            stream.write(f"  recommended_target_xy: [{recommended[0]:.6f}, {recommended[1]:.6f}]\n")
            stream.write(f"  recommended_robustness_radius_m: {recommended[2]:.6f}\n")
            stream.write("  equal_maximum_candidates_xy:\n")
            for x, y in maxima:
                stream.write(f"    - [{x:.6f}, {y:.6f}]\n")

    with (args.output_dir / "README.md").open("w", encoding="utf-8") as stream:
        stream.write(f"# {display_name} sweet-zone coarse scan\n\n")
        stream.write(f"- XY grid: `{xs[0]:.2f}..{xs[-1]:.2f} m` x `{ys[0]:.2f}..{ys[-1]:.2f} m`, `{step_x:.2f} m` step.\n")
        stream.write(f"- Orientation samples: `{', '.join(f'{value:g}' for value in yaw_values)} deg`.\n")
        stream.write(f"- Conservative points: `{conservative_points}/{len(xs) * len(ys)}`.\n")
        if recommended is not None:
            stream.write(f"- Recommended coarse-grid point: `({recommended[0]:.2f}, {recommended[1]:.2f}) m`.\n")
            stream.write(f"- Grid-derived XY robustness radius: `{recommended[2]:.2f} m`.\n")
        stream.write("\nThis is an offline candidate map using calibrated camera extrinsics, observation endpoint IK/collision checks, the current MoveIt collision model, ground, Go2 proxy, object-specific pregrasp/descent/lift geometry and carried-object OMPL planning to CARRY. Observation trajectories are OMPL-validated only at the later physical spot checks. It does not by itself prove RGB/depth perception or MuJoCo contact success at every grid point.\n")
        stream.write("\nFailure counts by stage:\n\n")
        for stage, count in sorted(failure_counts.items()):
            stream.write(f"- `{stage}`: {count}\n")


if __name__ == "__main__":
    main()
