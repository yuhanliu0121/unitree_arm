#!/usr/bin/env python3
"""Generate the concise Go2 handoff YAML from full workspace scan chunks."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


OBJECTS = {
    "yellow_cube": "cube",
    "zucchini": "zucchini",
    "bowl": "bowl",
}
MOUNT_TRANSFORM = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.057961769402],
    [0.0, 0.0, 0.0, 1.0],
]
MOTOR_ORDER = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]
POSTURE = [
    0.0473455, 1.22187, -2.44375,
    -0.0473455, 1.22187, -2.44375,
    0.0473455, 1.22187, -2.44375,
    -0.0473455, 1.22187, -2.44375,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def load_grid(paths: list[Path]):
    grouped = defaultdict(list)
    trial_keys = set()
    yaw_values = set()
    for path in paths:
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                x = round(float(row["x_m"]), 6)
                y = round(float(row["y_m"]), 6)
                yaw = float(row["object_yaw_deg"])
                key = (x, y, yaw)
                if key in trial_keys:
                    raise ValueError(f"duplicate scan trial {key} in {path}")
                trial_keys.add(key)
                grouped[(x, y)].append((yaw, row["success"] == "1"))
                yaw_values.add(yaw)

    xs = sorted({point[0] for point in grouped})
    ys = sorted({point[1] for point in grouped})
    missing = set((x, y) for x in xs for y in ys) - set(grouped)
    if missing:
        raise ValueError(f"scan grid is missing {len(missing)} XY points")
    for point, values in grouped.items():
        if {yaw for yaw, _ in values} != yaw_values:
            raise ValueError(f"incomplete orientation samples at {point}")

    green = {
        point for point, values in grouped.items()
        if all(success for _, success in values)
    }
    return xs, ys, green


def grid_clearances(green, xs, ys):
    step = min(
        min(b - a for a, b in zip(xs, xs[1:])),
        min(b - a for a, b in zip(ys, ys[1:])),
    )
    grey = set((x, y) for x in xs for y in ys) - green
    result = {}
    for point in green:
        distances = [math.dist(point, failed) for failed in grey]
        distances.extend(
            [
                point[0] - (xs[0] - 0.5 * step),
                (xs[-1] + 0.5 * step) - point[0],
                point[1] - (ys[0] - 0.5 * step),
                (ys[-1] + 0.5 * step) - point[1],
            ]
        )
        result[point] = min(distances)
    return step, result


def main() -> None:
    args = parse_args()
    workspace = args.workspace.resolve()
    objects = {}
    resolution = None
    for object_type, slug in OBJECTS.items():
        source = (
            workspace / "artifacts" / f"{slug}_sweet_zone" /
            f"{slug}_workspace.csv"
        )
        if not source.is_file():
            raise FileNotFoundError(f"workspace scan not found: {source}")
        xs, ys, green = load_grid([source])
        step, clearance = grid_clearances(green, xs, ys)
        resolution = step if resolution is None else resolution
        if not math.isclose(step, resolution):
            raise ValueError("object scans use different XY resolutions")
        objects[object_type] = [
            [point[0], point[1], round(clearance[point], 6)]
            for point in sorted(
                green, key=lambda point: (-clearance[point], point[1], point[0])
            )
        ]

    lines = [
        "schema_version: 1",
        "",
        "frames:",
        "  go2_frame: base_link",
        "  go2_frame_definition: Unitree_official_torso_root",
        "  official_urdf_alias: base",
        "  arm_frame: D1_base_link",
        "  convention: x_forward_y_left_z_up",
        "  transform_equation: p_go2 = T_go2_from_arm @ p_arm",
        "  T_go2_from_arm:",
    ]
    lines.extend(
        "    - [" + ", ".join(f"{value:.12g}" for value in row) + "]"
        for row in MOUNT_TRANSFORM
    )
    lines.extend(
        [
            "",
            "go2_posture:",
            "  name: manipulation_lie_down",
            "  joint_order: [" + ", ".join(MOTOR_ORDER) + "]",
            "  joint_positions_rad: ["
            + ", ".join(f"{value:.7g}" for value in POSTURE) + "]",
            "",
            "grasp_zones:",
            "  point_frame: D1_base_link",
            "  point_format: [x_m, y_m, clearance_m]",
            f"  xy_resolution_m: {resolution:.6g}",
            "  clearance_definition: distance_to_nearest_non_green_sample_or_scan_boundary",
            "  selection_rule: prefer_larger_clearance",
            "  objects:",
        ]
    )
    for object_type, points in objects.items():
        lines.append(f"    {object_type}:")
        lines.append(f"      green_point_count: {len(points)}")
        lines.append("      green_points:")
        lines.extend(
            "        - [" + ", ".join(f"{value:.6g}" for value in point) + "]"
            for point in points
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
