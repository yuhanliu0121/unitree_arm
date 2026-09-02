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
TRASH_BIN_VISUAL_PREFIX = "trash_bin_visual_"
TRASH_BIN_COLLISION_PREFIX = "trash_bin_collision_"
MESH_UP_TO_Z_QUAT = [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]


@dataclass(frozen=True)
class SpawnPose:
    name: str
    x: float
    y: float
    yaw: float
    footprint_radius: float


@dataclass(frozen=True)
class SceneLayout:
    objects: tuple[SpawnPose, ...]
    trash_bin: SpawnPose


_FOOTPRINT_RADIUS = {
    "yellow_cube": 0.036,
    "bowl": 0.060,
    "zucchini": 0.078,
}


def default_objects_root() -> Path:
    return Path(__file__).resolve().parents[4] / "simulation" / "objects"


def _ring_bounds(scene: dict) -> tuple[float, float, float, float, float, float]:
    inner = scene["base_exclusion_box"]
    inner_min_x = float(inner["min_x_m"])
    inner_max_x = float(inner["max_x_m"])
    inner_half_y = float(inner["half_width_y_m"])
    expansion = float(scene["spawn_region"]["outer_expansion_m"])
    return (
        inner_min_x,
        inner_max_x,
        -inner_half_y,
        inner_half_y,
        expansion,
        float(scene["spawn_region"]["outline_width_m"]),
    )


def _fits_rectangular_ring(
    x: float,
    y: float,
    radius: float,
    bounds: tuple[float, float, float, float, float, float],
) -> bool:
    tolerance = 1e-9
    inner_min_x, inner_max_x, inner_min_y, inner_max_y, expansion, _ = bounds
    outer_min_x = inner_min_x - expansion
    outer_max_x = inner_max_x + expansion
    outer_min_y = inner_min_y - expansion
    outer_max_y = inner_max_y + expansion
    if not (
        outer_min_x + radius - tolerance
        <= x
        <= outer_max_x - radius + tolerance
        and outer_min_y + radius - tolerance
        <= y
        <= outer_max_y - radius + tolerance
    ):
        return False
    # Only use the arm's forward half-plane: +X is forward, with azimuth
    # spanning -90 to +90 degrees. Keep the entire footprint in front.
    if x < radius - tolerance:
        return False
    # Distance from the circle centre to the inner rectangle. Requiring this
    # to be at least the footprint radius keeps the full item in the ring.
    dx = max(inner_min_x - x, 0.0, x - inner_max_x)
    dy = max(inner_min_y - y, 0.0, y - inner_max_y)
    return math.hypot(dx, dy) >= radius - tolerance


def sample_scene_layout(scene: dict) -> SceneLayout:
    """Jointly place three grasp objects and the bin in the Go2 ring."""
    if scene.get("placement_mode", "random") == "fixed":
        configured = scene["fixed_object_poses"]
        objects = tuple(
            SpawnPose(
                name=name,
                x=float(configured[name]["x_m"]),
                y=float(configured[name]["y_m"]),
                yaw=math.radians(float(configured[name].get("yaw_deg", 0.0))),
                footprint_radius=_FOOTPRINT_RADIUS[name],
            )
            for name in OBJECT_NAMES
        )
        bin_config = scene["trash_bin"]
        return SceneLayout(
            objects=objects,
            trash_bin=SpawnPose(
                name="trash_bin",
                x=float(bin_config["fixed_center_x_m"]),
                y=float(bin_config["fixed_center_y_m"]),
                yaw=0.0,
                footprint_radius=float(bin_config["outer_radius_m"]),
            ),
        )

    bounds = _ring_bounds(scene)
    inner_min_x, inner_max_x, inner_min_y, inner_max_y, expansion, _ = bounds
    outer_min_x = inner_min_x - expansion
    outer_max_x = inner_max_x + expansion
    outer_min_y = inner_min_y - expansion
    outer_max_y = inner_max_y + expansion
    gap = float(scene["object_gap_m"])
    rng = np.random.default_rng(int(scene["random_seed"]))

    radii = dict(_FOOTPRINT_RADIUS)
    radii["trash_bin"] = float(scene["trash_bin"]["outer_radius_m"])
    # Largest-first sampling preserves the scarce full-width locations needed
    # by the 30 cm-diameter bin.
    order = sorted((*OBJECT_NAMES, "trash_bin"), key=radii.get, reverse=True)
    placed: list[SpawnPose] = []
    for name in order:
        radius = radii[name]
        for _ in range(100_000):
            if name == "trash_bin":
                # The 30 cm ring width equals the 30 cm bin diameter. To keep
                # the complete bin outside the Go2 box, its centre must lie on
                # one of the two side-band centre lines; sample along that line.
                y = float(
                    rng.choice((-1.0, 1.0))
                    * (inner_max_y + radius)
                )
                min_distance = float(
                    scene["trash_bin"]["min_base_distance_m"]
                )
                max_distance = float(
                    scene["trash_bin"]["max_base_distance_m"]
                )
                min_x = max(radius, math.sqrt(max(0.0, min_distance**2 - y**2)))
                max_x = min(
                    outer_max_x - radius,
                    math.sqrt(max(0.0, max_distance**2 - y**2)),
                )
                if min_x > max_x:
                    raise RuntimeError(
                        "Trash-bin distance limits do not intersect the side ring"
                    )
                x = float(rng.uniform(min_x, max_x))
            else:
                x = float(rng.uniform(outer_min_x + radius, outer_max_x - radius))
                y = float(rng.uniform(outer_min_y + radius, outer_max_y - radius))
            yaw = float(rng.uniform(-math.pi, math.pi))
            if not _fits_rectangular_ring(x, y, radius, bounds):
                continue
            if name == "trash_bin":
                bin_config = scene["trash_bin"]
                base_distance = math.hypot(x, y)
                if not (
                    float(bin_config["min_base_distance_m"])
                    <= base_distance
                    <= float(bin_config["max_base_distance_m"])
                ):
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
                f"Could not place {name} in the configured rectangular ring"
            )

    by_name = {pose.name: pose for pose in placed}
    return SceneLayout(
        objects=tuple(by_name[name] for name in OBJECT_NAMES),
        trash_bin=by_name["trash_bin"],
    )


def sample_spawn_poses(scene: dict) -> tuple[SpawnPose, ...]:
    """Compatibility helper returning only grasp-object poses."""
    return sample_scene_layout(scene).objects


def _add_region_outline(spec: mujoco.MjSpec, scene: dict) -> None:
    inner_min_x, inner_max_x, inner_min_y, inner_max_y, expansion, line_width = (
        _ring_bounds(scene)
    )
    min_x = inner_min_x - expansion
    max_x = inner_max_x + expansion
    min_y = inner_min_y - expansion
    max_y = inner_max_y + expansion
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    half_x = (max_x - min_x) / 2.0
    half_y = (max_y - min_y) / 2.0
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
        pos=[min_x, center_y, 0.0005],
        size=[line_width / 2, half_y, 0.001],
        **common,
    )
    spec.worldbody.add_geom(
        name="spawn_region_far",
        pos=[max_x, center_y, 0.0005],
        size=[line_width / 2, half_y, 0.001],
        **common,
    )
    for side, y in (("left", min_y), ("right", max_y)):
        spec.worldbody.add_geom(
            name=f"spawn_region_{side}",
            pos=[center_x, y, 0.0005],
            size=[half_x, line_width / 2, 0.001],
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


def _add_trash_bin(
    spec: mujoco.MjSpec,
    scene: dict,
    pose: SpawnPose,
    show_collisions: bool,
) -> None:
    """Add a static segmented open-top bin with separate render proxies."""
    config = scene["trash_bin"]
    center_x = pose.x
    center_y = pose.y
    outer_radius = float(config["outer_radius_m"])
    height = float(config["height_m"])
    wall_thickness = float(config["wall_thickness_m"])
    bottom_thickness = float(config["bottom_thickness_m"])
    segment_count = int(config["wall_segments"])
    if not (
        outer_radius > wall_thickness > 0.0
        and height > bottom_thickness > 0.0
        and segment_count >= 8
    ):
        raise ValueError("Invalid trash-bin dimensions")

    wall_radius = outer_radius - wall_thickness / 2.0
    tangent_half = wall_radius * math.tan(math.pi / segment_count) + 0.001
    wall_half_height = (height - bottom_thickness) / 2.0
    wall_z = bottom_thickness + wall_half_height
    visual_rgba = [0.16, 0.32, 0.42, 1.0]
    collision_rgba = _collision_rgba(show_collisions)
    collision_common = {
        "contype": 1,
        "conaffinity": 1,
        "condim": 4,
        "friction": [0.8, 0.01, 0.001],
        "group": OBJECT_COLLISION_GROUP,
        "rgba": collision_rgba,
    }
    spec.worldbody.add_site(
        name="trash_bin_bottom_center",
        pos=[center_x, center_y, 0.0],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.001, 0.0, 0.0],
        rgba=[1.0, 1.0, 1.0, 0.0],
    )

    spec.worldbody.add_geom(
        name=f"{TRASH_BIN_VISUAL_PREFIX}bottom",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[center_x, center_y, bottom_thickness / 2.0],
        size=[outer_radius, bottom_thickness / 2.0, 0.0],
        contype=0,
        conaffinity=0,
        group=OBJECT_VISUAL_GROUP,
        rgba=visual_rgba,
    )
    spec.worldbody.add_geom(
        name=f"{TRASH_BIN_COLLISION_PREFIX}bottom",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[center_x, center_y, bottom_thickness / 2.0],
        size=[outer_radius, bottom_thickness / 2.0, 0.0],
        **collision_common,
    )

    for index in range(segment_count):
        theta = 2.0 * math.pi * index / segment_count
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        position = [
            center_x + wall_radius * cos_theta,
            center_y + wall_radius * sin_theta,
            wall_z,
        ]
        # Local x follows the tangent, local y points inward, and local z is up.
        xyaxes = [-sin_theta, cos_theta, 0.0, -cos_theta, -sin_theta, 0.0]
        size = [tangent_half, wall_thickness / 2.0, wall_half_height]
        spec.worldbody.add_geom(
            name=f"{TRASH_BIN_VISUAL_PREFIX}wall_{index:02d}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=position,
            xyaxes=xyaxes,
            size=size,
            contype=0,
            conaffinity=0,
            group=OBJECT_VISUAL_GROUP,
            rgba=visual_rgba,
        )
        spec.worldbody.add_geom(
            name=f"{TRASH_BIN_COLLISION_PREFIX}wall_{index:02d}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=position,
            xyaxes=xyaxes,
            size=size,
            **collision_common,
        )


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
    layout = sample_scene_layout(scene)
    _add_trash_bin(spec, scene, layout.trash_bin, show_collisions)
    poses = layout.objects
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
