from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from d1_mujoco_sim.model import build_model
from d1_mujoco_sim.ros_scene import (
    go2_mesh_specs,
    mobile_base_transform_specs,
    object_mesh_specs,
    physical_collision_specs,
    trash_bin_visual_specs,
)
from d1_mujoco_sim.simulator import D1Simulator


ROOT = Path(__file__).resolve().parents[2]
CONFIG = yaml.safe_load(
    (ROOT / "d1_mujoco_sim" / "config" / "sim.yaml").read_text()
)


@pytest.fixture(scope="module")
def model_and_data() -> tuple[mujoco.MjModel, mujoco.MjData]:
    controller = dict(CONFIG["controller"])
    controller["physics_timestep_s"] = CONFIG["simulation"][
        "physics_timestep_s"
    ]
    model = build_model(
        ROOT / "d1_constrained_description_20260728",
        controller,
        scene=CONFIG["scene"],
        objects_root=ROOT / "objects",
        camera=CONFIG["camera"],
        mobile_base=CONFIG["mobile_base"],
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_go2_mesh_and_tf_follow_gt_xy_motion(
    model_and_data: tuple[mujoco.MjModel, mujoco.MjData],
) -> None:
    model, _ = model_and_data
    simulator = D1Simulator(model, CONFIG)
    mesh_path = (
        ROOT
        / "d1_mujoco_sim"
        / "src"
        / "d1_mujoco_sim"
        / "assets"
        / "go2"
        / "go2_lie_down.obj"
    )
    before = go2_mesh_specs(model, simulator.data, mesh_path)[0]
    assert before.resource == mesh_path.as_uri()
    assert not before.embedded_materials
    assert before.color_rgba == (1.0, 1.0, 1.0, 1.0)

    simulator.set_mobile_base_xy(0.2, -0.1)
    after = go2_mesh_specs(model, simulator.data, mesh_path)[0]
    np.testing.assert_allclose(
        np.asarray(after.position) - np.asarray(before.position),
        [0.2, -0.1, 0.0],
    )
    transforms = mobile_base_transform_specs(model, simulator.data)
    assert [(item.parent, item.child) for item in transforms] == [
        ("world", "go2_base"),
        ("go2_base", "base_link"),
    ]
    np.testing.assert_allclose(transforms[0].position[:2], [0.2, -0.1])
    np.testing.assert_allclose(
        transforms[1].position,
        [0.0, 0.0, 0.057961769402],
    )


def test_object_mesh_markers_use_original_assets_and_runtime_body_poses(
    model_and_data: tuple[mujoco.MjModel, mujoco.MjData],
) -> None:
    model, data = model_and_data
    specs = object_mesh_specs(model, data, ROOT / "objects")
    assert [spec.name for spec in specs] == [
        "yellow_cube",
        "bowl",
        "zucchini",
    ]
    for spec in specs:
        body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            f"object_{spec.name}",
        )
        assert spec.shape == "mesh"
        assert spec.resource == (
            ROOT
            / "objects"
            / spec.name
            / f"{spec.name}-obj"
            / f"{spec.name}.obj"
        ).as_uri()
        np.testing.assert_allclose(spec.position, data.xpos[body_id])
        assert spec.scale == (1.0, 1.0, 1.0)
        assert spec.opaque


def test_object_mesh_markers_follow_free_body_motion(
    model_and_data: tuple[mujoco.MjModel, mujoco.MjData],
) -> None:
    model, _ = model_and_data
    data = mujoco.MjData(model)
    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "object_yellow_cube_free",
    )
    qpos_address = model.jnt_qposadr[joint_id]
    expected = np.asarray([0.21, -0.12, 0.17])
    data.qpos[qpos_address:qpos_address + 3] = expected
    data.qpos[qpos_address + 3:qpos_address + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    cube = object_mesh_specs(model, data, ROOT / "objects")[0]
    np.testing.assert_allclose(cube.position, expected)


def test_physical_collision_markers_come_from_active_mujoco_geoms(
    model_and_data: tuple[mujoco.MjModel, mujoco.MjData],
) -> None:
    model, data = model_and_data
    specs = physical_collision_specs(model, data)
    active_geoms = sum(
        bool(model.geom_contype[index] or model.geom_conaffinity[index])
        for index in range(model.ngeom)
    )
    assert len(specs) == active_geoms
    by_name = {spec.name: spec for spec in specs}

    ground = by_name["ground"]
    assert ground.shape == "box"
    assert ground.scale == (4.0, 4.0, 0.002)
    np.testing.assert_allclose(ground.position, [-0.044763, 0.0, -0.001])
    assert ground.opaque

    zucchini = by_name["object_collision_zucchini"]
    assert zucchini.shape == "sphere"
    np.testing.assert_allclose(zucchini.scale, [0.04, 0.15, 0.03312])

    bowl_walls = [
        spec for spec in specs
        if spec.name.startswith("object_collision_bowl_wall_")
    ]
    assert len(bowl_walls) == 12
    assert all(spec.shape == "box" for spec in bowl_walls)

    wrist = by_name["collision_wrist_roll"]
    assert wrist.shape == "cylinder"
    np.testing.assert_allclose(wrist.scale, [0.0766, 0.0766, 0.132])

    mount = by_name["collision_camera_main_stand"]
    camera = by_name["collision_camera_d435i"]
    assert mount.shape == camera.shape == "box"
    np.testing.assert_allclose(mount.scale, [0.130145850286, 0.080071964063, 0.034361989941])
    np.testing.assert_allclose(camera.scale, [0.0899313762, 0.0257728751, 0.02533514892])


def test_trash_bin_visual_markers_are_static_open_container(
    model_and_data: tuple[mujoco.MjModel, mujoco.MjData],
) -> None:
    model, data = model_and_data
    specs = trash_bin_visual_specs(model, data)
    assert len(specs) == CONFIG["scene"]["trash_bin"]["wall_segments"] + 1
    assert specs[0].name == "trash_bin_visual_bottom"
    assert all(spec.opaque for spec in specs)
    assert all(spec.color_rgba == (0.16, 0.32, 0.42, 1.0) for spec in specs)


def test_rviz_layer_defaults_keep_meshes_on_and_collisions_off() -> None:
    rviz = yaml.safe_load(
        (ROOT / "d1_moveit_config" / "config" / "moveit.rviz").read_text()
    )
    displays = {
        display["Name"]: display
        for display in rviz["Visualization Manager"]["Displays"]
    }
    assert displays["RobotModel"]["Value"] is True
    assert displays["Go2 Mesh"]["Enabled"] is True
    assert displays["Object Meshes"]["Enabled"] is True
    assert displays["MuJoCo Physical Collisions"]["Enabled"] is False
    assert displays["MoveIt Planning Collisions"]["Value"] is False
    assert displays["Wrist RGB"]["Enabled"] is True
    assert displays["Wrist Aligned Depth (Plasma)"]["Enabled"] is True
    assert displays["Wrist Raw Depth (Plasma)"]["Enabled"] is False
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "world"
    assert (
        displays["Wrist RGB"]["Topic"]["Value"]
        == "/wrist_camera/color/image_raw"
    )
    assert (
        displays["Wrist Aligned Depth (Plasma)"]["Topic"]["Value"]
        == "/wrist_camera/debug/aligned_depth_plasma"
    )
    assert (
        displays["Wrist Raw Depth (Plasma)"]["Topic"]["Value"]
        == "/wrist_camera/debug/depth_plasma"
    )
