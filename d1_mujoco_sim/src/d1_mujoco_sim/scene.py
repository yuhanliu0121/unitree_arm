from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np


OBJECT_NAMES = ("yellow_cube", "bowl", "zucchini")
OBJECT_VISUAL_GROUP = 2
OBJECT_COLLISION_GROUP = 4
OBJECT_COLLISION_PREFIX = "object_collision_"
MESH_UP_TO_Z_QUAT = [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]


@dataclass(frozen=True)
class SpawnPose:
    name: str
    x: float
    y: float
    yaw: float
    footprint_radius: float


_FOOTPRINT_RADIUS = {
    "yellow_cube": 0.036,
    "bowl": 0.060,
    "zucchini": 0.078,
}


def default_objects_root() -> Path:
    return Path(__file__).resolve().parents[3] / "objects"


def sample_spawn_poses(scene: dict) -> tuple[SpawnPose, ...]:
    """Deterministically place all objects without footprint overlap."""
    if scene.get("placement_mode", "random") == "fixed":
        configured = scene["fixed_object_poses"]
        return tuple(
            SpawnPose(
                name=name,
                x=float(configured[name]["x_m"]),
                y=float(configured[name]["y_m"]),
                yaw=math.radians(float(configured[name].get("yaw_deg", 0.0))),
                footprint_radius=_FOOTPRINT_RADIUS[name],
            )
            for name in OBJECT_NAMES
        )

    region = scene["spawn_region"]
    depth = float(region["depth_x_m"])
    width = float(region["width_y_m"])
    gap = float(scene["object_gap_m"])
    base_radius = float(scene["base_exclusion_radius_m"])
    base_box = scene.get("base_exclusion_box")
    rng = np.random.default_rng(int(scene["random_seed"]))

    # Largest-first rejection sampling is deterministic and avoids cases where
    # small objects fragment the limited 0.3 m x 0.4 m workspace.
    order = sorted(OBJECT_NAMES, key=_FOOTPRINT_RADIUS.get, reverse=True)
    placed: list[SpawnPose] = []
    for name in order:
        radius = _FOOTPRINT_RADIUS[name]
        for _ in range(10_000):
            x = float(rng.uniform(radius, depth - radius))
            y = float(rng.uniform(-width / 2 + radius, width / 2 - radius))
            yaw = float(rng.uniform(-math.pi, math.pi))
            if math.hypot(x, y) < base_radius + radius + gap:
                continue
            if base_box is not None:
                min_x = float(base_box["min_x_m"]) - radius - gap
                max_x = float(base_box["max_x_m"]) + radius + gap
                half_y = float(base_box["half_width_y_m"]) + radius + gap
                if min_x <= x <= max_x and abs(y) <= half_y:
                    continue
            if any(
                math.hypot(x - other.x, y - other.y)
                < radius + other.footprint_radius + gap
                for other in placed
            ):
                continue
            placed.append(SpawnPose(name, x, y, yaw, radius))
            break
        else:
            raise RuntimeError(
                f"Could not place {name} in the configured spawn region"
            )

    by_name = {pose.name: pose for pose in placed}
    return tuple(by_name[name] for name in OBJECT_NAMES)


def _add_region_outline(spec: mujoco.MjSpec, scene: dict) -> None:
    region = scene["spawn_region"]
    depth = float(region["depth_x_m"])
    width = float(region["width_y_m"])
    line_width = float(region["outline_width_m"])
    rgba = [1.0, 1.0, 1.0, 1.0]
    common = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "contype": 0,
        "conaffinity": 0,
        "group": 0,
        "rgba": rgba,
    }
    spec.worldbody.add_geom(
        name="spawn_region_near",
        pos=[0.0, 0.0, 0.0005],
        size=[line_width / 2, width / 2, 0.001],
        **common,
    )
    spec.worldbody.add_geom(
        name="spawn_region_far",
        pos=[depth, 0.0, 0.0005],
        size=[line_width / 2, width / 2, 0.001],
        **common,
    )
    for side, y in (("left", -width / 2), ("right", width / 2)):
        spec.worldbody.add_geom(
            name=f"spawn_region_{side}",
            pos=[depth / 2, y, 0.0005],
            size=[depth / 2, line_width / 2, 0.001],
            **common,
        )


def _add_pbr_assets(
    spec: mujoco.MjSpec,
    objects_root: Path,
    name: str,
) -> tuple[str, str]:
    asset_root = objects_root / name / f"{name}-obj"
    obj_path = asset_root / f"{name}.obj"
    if not obj_path.is_file():
        raise FileNotFoundError(f"Object OBJ not found: {obj_path}")

    mesh_name = f"object_mesh_{name}"
    material_name = f"object_material_{name}"
    spec.add_mesh(name=mesh_name, file=str(obj_path), smoothnormal=1)
    material = spec.add_material(
        name=material_name,
        metallic=0.0,
        roughness=0.5,
        rgba=[1.0, 1.0, 1.0, 1.0],
    )
    texture_files = {
        mujoco.mjtTextureRole.mjTEXROLE_RGB:
            "texture_pbr_20250901.png",
        mujoco.mjtTextureRole.mjTEXROLE_METALLIC:
            "texture_pbr_20250901_metallic.png",
        mujoco.mjtTextureRole.mjTEXROLE_ROUGHNESS:
            "texture_pbr_20250901_roughness.png",
        mujoco.mjtTextureRole.mjTEXROLE_NORMAL:
            "texture_pbr_20250901_normal.png",
    }
    for role, filename in texture_files.items():
        path = asset_root / filename
        if not path.is_file():
            raise FileNotFoundError(f"Object texture not found: {path}")
        texture_name = f"object_texture_{name}_{role.name.lower()}"
        spec.add_texture(
            name=texture_name,
            type=mujoco.mjtTexture.mjTEXTURE_2D,
            colorspace=(
                mujoco.mjtColorSpace.mjCOLORSPACE_SRGB
                if role == mujoco.mjtTextureRole.mjTEXROLE_RGB
                else mujoco.mjtColorSpace.mjCOLORSPACE_LINEAR
            ),
            file=str(path),
        )
        material.textures[role] = texture_name
    return mesh_name, material_name


def _collision_rgba(show_collisions: bool) -> list[float]:
    return [0.55, 1.0, 0.05, 0.62 if show_collisions else 0.0]


def _collision_common(show_collisions: bool) -> dict:
    return {
        "contype": 1,
        "conaffinity": 1,
        "condim": 4,
        "friction": [1.0, 0.02, 0.002],
        "solref": [0.002, 1.0],
        "solimp": [0.95, 0.99, 0.001, 0.5, 2.0],
        "group": OBJECT_COLLISION_GROUP,
        "rgba": _collision_rgba(show_collisions),
    }


def _add_cube_collision(body, show_collisions: bool, mass: float) -> None:
    body.add_geom(
        name="object_collision_yellow_cube",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[-0.000787, -0.000889, 0.025],
        size=[0.025, 0.025, 0.025],
        mass=mass,
        **_collision_common(show_collisions),
    )


def _add_zucchini_collision(body, show_collisions: bool, mass: float) -> None:
    # The measured centre slice is 40.0 mm wide by 33.1 mm high. An ellipsoid
    # matches that non-circular cross-section and the tapered ends better than
    # the previous 45 mm-diameter capsule.
    body.add_geom(
        name="object_collision_zucchini",
        type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
        pos=[0.00047, -0.00478, 0.01656],
        size=[0.0200, 0.0750, 0.01656],
        mass=mass,
        **_collision_common(show_collisions),
    )


def _add_bowl_collision(body, show_collisions: bool, mass: float) -> None:
    common = _collision_common(show_collisions)
    center_x = -0.00023
    center_y = -0.00005
    body.add_geom(
        name="object_collision_bowl_bottom",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[center_x, center_y, 0.004],
        size=[0.0405, 0.004, 0.0],
        mass=mass * 0.25,
        **common,
    )

    segment_count = 12
    bottom_radius = 0.039
    top_radius = 0.058
    bottom_z = 0.004
    top_z = 0.049
    radial_delta = top_radius - bottom_radius
    vertical_delta = top_z - bottom_z
    slope = math.atan2(radial_delta, vertical_delta)
    mid_radius = (bottom_radius + top_radius) / 2
    mid_z = (bottom_z + top_z) / 2
    slant_half = math.hypot(radial_delta, vertical_delta) / 2
    tangent_half = mid_radius * math.tan(math.pi / segment_count) + 0.0008
    for index in range(segment_count):
        theta = 2 * math.pi * index / segment_count
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        tangent = [-sin_theta, cos_theta, 0.0]
        normal = [
            -math.cos(slope) * cos_theta,
            -math.cos(slope) * sin_theta,
            math.sin(slope),
        ]
        body.add_geom(
            name=f"object_collision_bowl_wall_{index:02d}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[
                center_x + mid_radius * cos_theta,
                center_y + mid_radius * sin_theta,
                mid_z,
            ],
            xyaxes=[*tangent, *normal],
            size=[tangent_half, 0.002, slant_half],
            mass=mass * 0.75 / segment_count,
            **common,
        )


def add_object_scene(
    spec: mujoco.MjSpec,
    objects_root: Path,
    scene: dict,
    *,
    show_collisions: bool,
) -> tuple[SpawnPose, ...]:
    objects_root = Path(objects_root).resolve()
    _add_region_outline(spec, scene)
    poses = sample_spawn_poses(scene)
    masses = scene["objects"]

    for pose in poses:
        mesh_name, material_name = _add_pbr_assets(
            spec, objects_root, pose.name
        )
        body = spec.worldbody.add_body(
            name=f"object_{pose.name}",
            pos=[pose.x, pose.y, 0.0],
            quat=[
                math.cos(pose.yaw / 2),
                0.0,
                0.0,
                math.sin(pose.yaw / 2),
            ],
        )
        body.add_freejoint(name=f"object_{pose.name}_free")
        body.add_geom(
            name=f"object_visual_{pose.name}",
            type=mujoco.mjtGeom.mjGEOM_MESH,
            quat=MESH_UP_TO_Z_QUAT,
            meshname=mesh_name,
            material=material_name,
            contype=0,
            conaffinity=0,
            mass=0.0,
            group=OBJECT_VISUAL_GROUP,
        )
        mass = float(masses[pose.name]["mass_kg"])
        if pose.name == "yellow_cube":
            _add_cube_collision(body, show_collisions, mass)
        elif pose.name == "bowl":
            _add_bowl_collision(body, show_collisions, mass)
        elif pose.name == "zucchini":
            _add_zucchini_collision(body, show_collisions, mass)
        else:
            raise AssertionError(f"Unhandled object: {pose.name}")
    return poses
