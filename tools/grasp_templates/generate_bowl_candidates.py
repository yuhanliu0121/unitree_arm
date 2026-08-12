#!/usr/bin/env python3
"""Expand the accepted bowl template into its ordered TCP candidates.

This is a development/reference utility. Runtime code may implement the same
matrix equation directly, using grasp_templates.yaml as the source of truth.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = WORKSPACE / "d1_manipulation" / "config" / "grasp_templates.yaml"


def quaternion_matrix_xyzw(values: list[float]) -> np.ndarray:
    x, y, z, w = values
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


def pose_matrix(values: list[float]) -> np.ndarray:
    if len(values) != 7:
        raise ValueError("pose requires x y z qx qy qz qw")
    matrix = quaternion_matrix_xyzw(values[3:])
    matrix[:3, 3] = values[:3]
    return matrix


def rotation_z(degrees: float) -> np.ndarray:
    radians = np.deg2rad(degrees)
    cosine = float(np.cos(radians))
    sine = float(np.sin(radians))
    return np.array(
        [
            [cosine, -sine, 0, 0],
            [sine, cosine, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


def matrix_quaternion_xyzw(matrix: np.ndarray) -> list[float]:
    # Stable branch-based conversion for a proper 3x3 rotation matrix.
    rotation = matrix[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion.tolist()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--world-from-rim",
        type=float,
        nargs=7,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        help="Detected bowl-rim pose; identity prints candidates in bowl_rim.",
    )
    parser.add_argument("--json", action="store_true", help="Print all candidate records as JSON.")
    args = parser.parse_args()

    with args.config.open("r", encoding="utf-8") as stream:
        bowl = yaml.safe_load(stream)["objects"]["bowl"]
    canonical = np.asarray(
        bowl["canonical_grasp"]["rim_from_tcp_matrix_row_major"], dtype=np.float64
    )
    search = bowl["candidate_search"]
    world_from_rim = pose_matrix(list(args.world_from_rim))

    candidates = []
    for theta in search["rim_azimuth_degrees_ordered"]:
        for flip in search["tcp_z_flip_degrees_ordered"]:
            world_from_tcp = (
                world_from_rim @ rotation_z(theta) @ canonical @ rotation_z(flip)
            )
            candidates.append(
                {
                    "index": len(candidates) + 1,
                    "rim_azimuth_deg": float(theta),
                    "tcp_z_flip_deg": float(flip),
                    "translation_m": world_from_tcp[:3, 3].tolist(),
                    "quaternion_xyzw": matrix_quaternion_xyzw(world_from_tcp),
                    "world_from_tcp_matrix_row_major": world_from_tcp.tolist(),
                }
            )

    expected = int(search["candidate_count"])
    if len(candidates) != expected:
        raise RuntimeError(f"configuration says {expected} candidates, generated {len(candidates)}")
    if args.json:
        print(json.dumps(candidates, indent=2))
    else:
        first = candidates[0]
        last = candidates[-1]
        print(f"generated {len(candidates)} ordered bowl TCP candidates")
        print(
            "first: theta={rim_azimuth_deg:g} flip={tcp_z_flip_deg:g} "
            "translation={translation_m}".format(**first)
        )
        print(
            "last:  theta={rim_azimuth_deg:g} flip={tcp_z_flip_deg:g} "
            "translation={translation_m}".format(**last)
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
