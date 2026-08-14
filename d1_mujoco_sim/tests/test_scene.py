from copy import deepcopy
import math
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from d1_mujoco_sim.model import build_model
from d1_mujoco_sim.scene import (
    OBJECT_COLLISION_GROUP,
    OBJECT_COLLISION_PREFIX,
    OBJECT_NAMES,
    OBJECT_VISUAL_GROUP,
    TRASH_BIN_COLLISION_PREFIX,
    TRASH_BIN_VISUAL_PREFIX,
    sample_scene_layout,
    sample_spawn_poses,
)
from d1_mujoco_sim.simulator import D1Simulator
from d1_mujoco_sim.scene_state import scene_state_payload


ROOT = Path(__file__).resolve().parents[2]
CONFIG = yaml.safe_load(
    (ROOT / "d1_mujoco_sim" / "config" / "sim.yaml").read_text()
)


def controller_config() -> dict:
    controller = dict(CONFIG["controller"])
    controller["physics_timestep_s"] = CONFIG["simulation"][
        "physics_timestep_s"
    ]
    return controller


@pytest.fixture(scope="module")
def scene_model() -> mujoco.MjModel:
    return build_model(
        ROOT / "d1_constrained_description_20260728",
        controller_config(),
        scene=CONFIG["scene"],
        objects_root=ROOT / "objects",
    )


def test_seed_zero_random_layout_is_deterministic_and_nonoverlapping() -> None:
    random_scene = deepcopy(CONFIG["scene"])
    random_scene["placement_mode"] = "random"
    first = sample_spawn_poses(random_scene)
    second = sample_spawn_poses(random_scene)
    assert first == second
    assert tuple(pose.name for pose in first) == OBJECT_NAMES

    expansion = random_scene["spawn_region"]["outer_expansion_m"]
    gap = random_scene["object_gap_m"]
    base_box = random_scene["base_exclusion_box"]
    layout = sample_scene_layout(random_scene)
    poses = (*layout.objects, layout.trash_bin)
    trash_bin_config = random_scene["trash_bin"]
    trash_bin_distance = math.hypot(
        layout.trash_bin.x,
        layout.trash_bin.y,
    )
    assert (
        trash_bin_config["min_base_distance_m"]
        <= trash_bin_distance
        <= trash_bin_config["max_base_distance_m"]
    )
    outer_min_x = base_box["min_x_m"] - expansion
    outer_max_x = base_box["max_x_m"] + expansion
    outer_min_y = -base_box["half_width_y_m"] - expansion
    outer_max_y = base_box["half_width_y_m"] + expansion
    tolerance = 1e-9
    for index, pose in enumerate(poses):
        radius = pose.footprint_radius
        assert pose.x >= radius - tolerance
        assert (
            outer_min_x + radius - tolerance
            <= pose.x
            <= outer_max_x - radius + tolerance
        )
        assert (
            outer_min_y + radius - tolerance
            <= pose.y
            <= outer_max_y - radius + tolerance
        )
        dx = max(
            base_box["min_x_m"] - pose.x,
            0.0,
            pose.x - base_box["max_x_m"],
        )
        dy = max(
            -base_box["half_width_y_m"] - pose.y,
            0.0,
            pose.y - base_box["half_width_y_m"],
        )
        assert math.hypot(dx, dy) >= radius - tolerance
        for other in poses[index + 1 :]:
            assert math.hypot(pose.x - other.x, pose.y - other.y) >= (
                pose.footprint_radius + other.footprint_radius + gap
            )


def test_seed_override_changes_layout() -> None:
    changed = deepcopy(CONFIG["scene"])
    changed["placement_mode"] = "random"
    changed["random_seed"] = 1
    baseline = deepcopy(changed)
    baseline["random_seed"] = 0
    assert sample_spawn_poses(changed) != sample_spawn_poses(baseline)


def test_fixed_layout_overrides_seed_without_removing_random_capability() -> None:
    fixed = deepcopy(CONFIG["scene"])
    fixed["placement_mode"] = "fixed"
    first = sample_spawn_poses(fixed)
    fixed["random_seed"] = 12345
    second = sample_spawn_poses(fixed)
    assert first == second
    cube = next(pose for pose in first if pose.name == "yellow_cube")
    assert cube.x == pytest.approx(0.28)
    assert cube.y == pytest.approx(-0.24)
    bowl = next(pose for pose in first if pose.name == "bowl")
    assert bowl.x == pytest.approx(0.20)
    assert bowl.y == pytest.approx(-0.32)
    zucchini = next(pose for pose in first if pose.name == "zucchini")
    assert zucchini.x == pytest.approx(0.42)
    assert zucchini.y == pytest.approx(0.08)

    layout = sample_scene_layout(fixed)
    poses = (*layout.objects, layout.trash_bin)
    for index, pose in enumerate(poses):
        for other in poses[index + 1 :]:
            assert math.hypot(pose.x - other.x, pose.y - other.y) >= (
                pose.footprint_radius + other.footprint_radius
            )

    random_scene = deepcopy(fixed)
    random_scene["placement_mode"] = "random"
    assert sample_spawn_poses(random_scene) != first


def test_scene_imports_pbr_visuals_and_dynamic_collision(
    scene_model: mujoco.MjModel,
) -> None:
    assert scene_model.nmesh == 12  # Nine arm meshes and three object meshes.
    assert scene_model.ntex == 14  # Sky, ground and four PBR maps per object.
    assert scene_model.nmat == 4
    expected_mass = {"yellow_cube": 0.08, "bowl": 0.08, "zucchini": 0.18}

    for name in OBJECT_NAMES:
        body_id = mujoco.mj_name2id(
            scene_model,
            mujoco.mjtObj.mjOBJ_BODY,
            f"object_{name}",
        )
        visual_id = mujoco.mj_name2id(
            scene_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            f"object_visual_{name}",
        )
        assert body_id >= 0
        assert visual_id >= 0
        assert scene_model.body_dofnum[body_id] == 6
        assert scene_model.body_mass[body_id] == pytest.approx(
            expected_mass[name]
        )
        assert scene_model.geom_group[visual_id] == OBJECT_VISUAL_GROUP
        assert scene_model.geom_contype[visual_id] == 0
        assert scene_model.geom_conaffinity[visual_id] == 0
        joint_id = mujoco.mj_name2id(
            scene_model,
            mujoco.mjtObj.mjOBJ_JOINT,
            f"object_{name}_free",
        )
        dof_address = scene_model.jnt_dofadr[joint_id]
        np.testing.assert_allclose(
            scene_model.dof_damping[dof_address:dof_address + 3],
            2.0,
        )
        np.testing.assert_allclose(
            scene_model.dof_damping[dof_address + 3:dof_address + 6],
            4.0,
        )

    object_collision_ids = [
        geom_id
        for geom_id in range(scene_model.ngeom)
        if (
            mujoco.mj_id2name(
                scene_model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
            ) or ""
        ).startswith(OBJECT_COLLISION_PREFIX)
    ]
    assert len(object_collision_ids) == 15  # Cube, zucchini and 13-part bowl.
    for geom_id in object_collision_ids:
        assert scene_model.geom_contype[geom_id] == 1
        assert scene_model.geom_conaffinity[geom_id] == 1
        assert scene_model.geom_rgba[geom_id, 3] == 0.0

    zucchini_id = mujoco.mj_name2id(
        scene_model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "object_collision_zucchini",
    )
    assert scene_model.geom_type[zucchini_id] == mujoco.mjtGeom.mjGEOM_ELLIPSOID
    np.testing.assert_allclose(
        scene_model.geom_size[zucchini_id], [0.0200, 0.0750, 0.01656]
    )
    np.testing.assert_allclose(
        scene_model.geom_pos[zucchini_id], [0.00047, -0.00478, 0.01656]
    )

    # RGB, roughness, metallic and normal roles are all populated.
    for name in OBJECT_NAMES:
        material_id = mujoco.mj_name2id(
            scene_model,
            mujoco.mjtObj.mjOBJ_MATERIAL,
            f"object_material_{name}",
        )
        for role in (
            mujoco.mjtTextureRole.mjTEXROLE_RGB,
            mujoco.mjtTextureRole.mjTEXROLE_ROUGHNESS,
            mujoco.mjtTextureRole.mjTEXROLE_METALLIC,
            mujoco.mjtTextureRole.mjTEXROLE_NORMAL,
        ):
            assert scene_model.mat_texid[material_id, role] >= 0


def test_ground_uses_rectified_photo_texture(
    scene_model: mujoco.MjModel,
) -> None:
    ground_id = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_GEOM, "ground"
    )
    material_id = scene_model.geom_matid[ground_id]
    assert mujoco.mj_id2name(
        scene_model, mujoco.mjtObj.mjOBJ_MATERIAL, material_id
    ) == "ground_material"
    texture_id = scene_model.mat_texid[
        material_id, mujoco.mjtTextureRole.mjTEXROLE_RGB
    ]
    assert mujoco.mj_id2name(
        scene_model, mujoco.mjtObj.mjOBJ_TEXTURE, texture_id
    ) == "ground_texture"
    assert scene_model.mat_texuniform[material_id]
    np.testing.assert_allclose(
        scene_model.mat_texrepeat[material_id],
        [1.0 / 1.2, 1.0 / 1.2],
    )

    skybox_id = mujoco.mj_name2id(
        scene_model, mujoco.mjtObj.mjOBJ_TEXTURE, "skybox"
    )
    assert skybox_id >= 0
    assert scene_model.tex_type[skybox_id] == (
        mujoco.mjtTexture.mjTEXTURE_SKYBOX
    )


def test_white_outline_matches_configured_region(
    scene_model: mujoco.MjModel,
) -> None:
    expected = {
        "spawn_region_near": ([-0.721484, 0.0], [0.0015, 0.469127]),
        "spawn_region_far": ([0.631958, 0.0], [0.0015, 0.469127]),
        "spawn_region_left": ([-0.044763, -0.469127], [0.676721, 0.0015]),
        "spawn_region_right": ([-0.044763, 0.469127], [0.676721, 0.0015]),
    }
    for name, (position, size) in expected.items():
        geom_id = mujoco.mj_name2id(
            scene_model, mujoco.mjtObj.mjOBJ_GEOM, name
        )
        assert geom_id >= 0
        np.testing.assert_allclose(scene_model.geom_pos[geom_id, :2], position)
        np.testing.assert_allclose(scene_model.geom_size[geom_id, :2], size)
        np.testing.assert_allclose(scene_model.geom_rgba[geom_id], [1, 1, 1, 1])
        assert scene_model.geom_contype[geom_id] == 0


def test_static_trash_bin_is_open_and_uses_sampled_ring_pose(
    scene_model: mujoco.MjModel,
) -> None:
    config = CONFIG["scene"]["trash_bin"]
    segment_count = config["wall_segments"]
    visual_ids = []
    collision_ids = []
    for geom_id in range(scene_model.ngeom):
        name = mujoco.mj_id2name(
            scene_model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
        ) or ""
        if name.startswith(TRASH_BIN_VISUAL_PREFIX):
            visual_ids.append(geom_id)
        if name.startswith(TRASH_BIN_COLLISION_PREFIX):
            collision_ids.append(geom_id)

    assert len(visual_ids) == segment_count + 1
    assert len(collision_ids) == segment_count + 1
    assert all(scene_model.geom_contype[index] == 0 for index in visual_ids)
    assert all(scene_model.geom_contype[index] == 1 for index in collision_ids)
    assert all(
        scene_model.geom_group[index] == OBJECT_COLLISION_GROUP
        for index in collision_ids
    )
    bottom_id = mujoco.mj_name2id(
        scene_model,
        mujoco.mjtObj.mjOBJ_GEOM,
        f"{TRASH_BIN_COLLISION_PREFIX}bottom",
    )
    np.testing.assert_allclose(
        scene_model.geom_pos[bottom_id, :2],
        [
            sample_scene_layout(CONFIG["scene"]).trash_bin.x,
            sample_scene_layout(CONFIG["scene"]).trash_bin.y,
        ],
    )
    # Only a bottom and segmented side walls exist; there is no lid geom.
    assert not any(
        "top" in (
            mujoco.mj_id2name(scene_model, mujoco.mjtObj.mjOBJ_GEOM, index)
            or ""
        )
        for index in (*visual_ids, *collision_ids)
    )


def test_objects_settle_on_ground_without_interobject_contact(
    scene_model: mujoco.MjModel,
) -> None:
    simulator = D1Simulator(scene_model, CONFIG)
    for _ in range(1500):
        simulator.step()
    assert np.isfinite(simulator.data.qpos).all()
    for contact in simulator.data.contact:
        body1 = scene_model.geom_bodyid[contact.geom1]
        body2 = scene_model.geom_bodyid[contact.geom2]
        name1 = mujoco.mj_id2name(
            scene_model, mujoco.mjtObj.mjOBJ_BODY, body1
        ) or "world"
        name2 = mujoco.mj_id2name(
            scene_model, mujoco.mjtObj.mjOBJ_BODY, body2
        ) or "world"
        assert not (name1.startswith("object_") and name2.startswith("object_"))


def test_scene_state_payload_contains_planning_frame_poses(
    scene_model: mujoco.MjModel,
) -> None:
    data = mujoco.MjData(scene_model)
    mujoco.mj_forward(scene_model, data)
    lines = scene_state_payload(scene_model, data).splitlines()
    assert lines[0].startswith("D1SCENE 1 ")
    names = [line.split()[0] for line in lines[1:]]
    assert names[:len(OBJECT_NAMES)] == list(OBJECT_NAMES)
    assert "trash_bin" in names
    assert {
        "debug_tcp_link",
        "debug_left_finger",
        "debug_right_finger",
        "debug_cube_contacts",
        "debug_gripper_state",
    }.issubset(names)
    assert all(len(line.split()) == 8 for line in lines[1:])


def test_scene_state_tracks_mobile_base_link_frame() -> None:
    model = build_model(
        ROOT / "d1_constrained_description_20260728",
        controller_config(),
        scene=CONFIG["scene"],
        objects_root=ROOT / "objects",
        mobile_base=CONFIG["mobile_base"],
    )
    simulator = D1Simulator(model, CONFIG)

    def object_position(name: str) -> np.ndarray:
        records = {
            line.split()[0]: line.split()[1:]
            for line in scene_state_payload(model, simulator.data).splitlines()[1:]
        }
        return np.asarray(records[name][:3], dtype=np.float64)

    cube_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "object_yellow_cube",
    )
    arm_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "base_link",
    )
    np.testing.assert_allclose(
        object_position("yellow_cube"),
        simulator.data.xpos[cube_id] - simulator.data.xpos[arm_id],
        atol=1e-9,
    )
    before = object_position("yellow_cube")
    simulator.set_mobile_base_xy(0.1, -0.05)
    np.testing.assert_allclose(
        object_position("yellow_cube") - before,
        [-0.1, 0.05, 0.0],
        atol=1e-9,
    )
