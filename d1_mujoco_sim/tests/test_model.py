from pathlib import Path

import mujoco
import numpy as np
import yaml

from d1_mujoco_sim.model import (
    COLLISION_GEOM_NAMES,
    JOINT_NAMES,
    build_model,
)
from d1_mujoco_sim.simulator import D1Simulator


ROOT = Path(__file__).resolve().parents[2]
CONFIG = yaml.safe_load(
    (ROOT / "d1_mujoco_sim" / "config" / "sim.yaml").read_text()
)


def make_simulator(manual_control: bool = False) -> D1Simulator:
    controller = dict(CONFIG["controller"])
    controller["physics_timestep_s"] = CONFIG["simulation"][
        "physics_timestep_s"
    ]
    model = build_model(
        ROOT / "d1_constrained_description_20260728",
        controller,
        simulation=CONFIG["simulation"],
        camera=CONFIG["camera"],
    )
    return D1Simulator(model, CONFIG, manual_control=manual_control)


def test_model_has_expected_joint_and_actuator_contract() -> None:
    simulator = make_simulator()
    assert simulator.model.nu == 7
    assert simulator.model.neq == 1
    assert simulator.model.nmesh == 11
    # Nine arm STLs, camera and mount visuals, nine proxies, and the ground.
    assert simulator.model.ngeom == 21
    assert simulator.model.ncam == 3
    for name in JOINT_NAMES:
        assert (
            mujoco.mj_name2id(
                simulator.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                name,
            )
            >= 0
        )

    gripper_actuator = mujoco.mj_name2id(
        simulator.model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "Joint6_position",
    )
    assert simulator.model.actuator_gainprm[gripper_actuator, 0] == 800.0
    assert simulator.model.actuator_biasprm[gripper_actuator, 2] == -80.0
    np.testing.assert_allclose(
        simulator.model.actuator_ctrlrange[gripper_actuator],
        [-0.005, 0.03],
    )


def test_contact_solver_suppresses_slow_grasp_slip() -> None:
    simulator = make_simulator()
    assert simulator.model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert simulator.model.opt.impratio == 10.0
    assert simulator.model.opt.noslip_iterations == 2


def test_collision_proxies_are_physical_but_hidden_by_default() -> None:
    simulator = make_simulator()
    for name in COLLISION_GEOM_NAMES:
        geom_id = mujoco.mj_name2id(
            simulator.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            name,
        )
        assert geom_id >= 0
        assert simulator.model.geom_contype[geom_id] == 1
        assert simulator.model.geom_conaffinity[geom_id] == 1
        assert simulator.model.geom_group[geom_id] == 3
        assert simulator.model.geom_rgba[geom_id, 3] == 0.0

    for name in ("collision_left_finger", "collision_right_finger"):
        geom_id = mujoco.mj_name2id(
            simulator.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            name,
        )
        np.testing.assert_allclose(
            simulator.model.geom_friction[geom_id],
            [5.0, 0.02, 0.002],
        )
        np.testing.assert_allclose(
            simulator.model.geom_solref[geom_id],
            [0.002, 1.0],
        )

    # Adjacent assembly overlap is excluded, leaving a contact-free nominal
    # pose while retaining non-adjacent self-collision and world collision.
    mujoco.mj_forward(simulator.model, simulator.data)
    assert simulator.data.ncon == 0


def _local_transform(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    rotation = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, quaternion)
    transform = np.eye(4)
    transform[:3, :3] = rotation.reshape(3, 3)
    transform[:3, 3] = position
    return transform


def test_d435i_visual_and_calibrated_optical_cameras() -> None:
    model = make_simulator().model
    for visual_name in ("d435i_visual", "d435i_main_stand_visual"):
        visual_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            visual_name,
        )
        assert visual_id >= 0
        assert model.geom_group[visual_id] == 1
        assert model.geom_contype[visual_id] == 0
        assert model.geom_conaffinity[visual_id] == 0

    link_from_color = np.asarray(
        CONFIG["camera"]["T_link6_color_optical"],
        dtype=np.float64,
    )
    color_from_depth = np.asarray(
        CONFIG["camera"]["T_color_depth_optical"],
        dtype=np.float64,
    )
    for frame, expected in (
        (CONFIG["camera"]["color"]["frame"], link_from_color),
        (
            CONFIG["camera"]["depth"]["frame"],
            link_from_color @ color_from_depth,
        ),
    ):
        site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            frame,
        )
        actual = _local_transform(
            model.site_pos[site_id],
            model.site_quat[site_id],
        )
        # MuJoCo normalizes the supplied quaternion during compilation.
        np.testing.assert_allclose(actual, expected, atol=5e-8)

    optical_from_mujoco_camera = np.diag([1.0, -1.0, -1.0, 1.0])
    for stream, expected_optical in (
        (CONFIG["camera"]["color"], link_from_color),
        (CONFIG["camera"]["depth"], link_from_color @ color_from_depth),
    ):
        camera_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            stream["name"],
        )
        actual_camera = _local_transform(
            model.cam_pos[camera_id],
            model.cam_quat[camera_id],
        )
        np.testing.assert_allclose(
            actual_camera,
            expected_optical @ optical_from_mujoco_camera,
            atol=5e-8,
        )
        np.testing.assert_array_equal(
            model.cam_resolution[camera_id],
            [stream["width"], stream["height"]],
        )
        np.testing.assert_allclose(
            model.cam_intrinsic[camera_id],
            [
                stream["fx"],
                stream["fy"],
                stream["width"] / 2.0 - stream["cx"],
                stream["height"] / 2.0 - stream["cy"],
            ],
            rtol=1e-7,
        )


def test_collision_debug_mode_reveals_proxies() -> None:
    controller = dict(CONFIG["controller"])
    controller["physics_timestep_s"] = CONFIG["simulation"][
        "physics_timestep_s"
    ]
    model = build_model(
        ROOT / "d1_constrained_description_20260728",
        controller,
        show_collisions=True,
    )
    alpha = []
    for name in COLLISION_GEOM_NAMES:
        geom_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            name,
        )
        alpha.append(model.geom_rgba[geom_id, 3])
    np.testing.assert_allclose(alpha, 0.62, atol=1e-6)


def test_collision_proxies_detect_ground_and_nonadjacent_arm_contact() -> None:
    simulator = make_simulator()
    joint_id = mujoco.mj_name2id(
        simulator.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "Joint2",
    )
    qpos_id = simulator.model.jnt_qposadr[joint_id]
    simulator.data.qpos[qpos_id] = simulator.model.jnt_range[joint_id, 1]
    mujoco.mj_forward(simulator.model, simulator.data)
    contact_names = {
        mujoco.mj_id2name(
            simulator.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            contact.geom1,
        )
        for contact in simulator.data.contact
    } | {
        mujoco.mj_id2name(
            simulator.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            contact.geom2,
        )
        for contact in simulator.data.contact
    }
    assert "ground" in contact_names
    assert "collision_base" in contact_names
    assert "collision_wrist_roll" in contact_names


def test_gripper_mimic_and_zero_hold() -> None:
    simulator = make_simulator()
    for _ in range(2000):
        simulator.step()
    mimic_id = mujoco.mj_name2id(
        simulator.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "Joint6_mimic",
    )
    mimic_qpos = simulator.data.qpos[simulator.model.jnt_qposadr[mimic_id]]
    np.testing.assert_allclose(
        mimic_qpos,
        -simulator.joint_positions[6],
        atol=2e-4,
    )
    np.testing.assert_allclose(
        simulator.sdk_angles_deg[:6],
        np.zeros(6),
        atol=0.05,
    )


def test_gripper_mimic_remains_symmetric_under_one_sided_load() -> None:
    simulator = make_simulator()
    left_finger = mujoco.mj_name2id(
        simulator.model,
        mujoco.mjtObj.mjOBJ_BODY,
        "left_finger",
    )
    mimic_id = mujoco.mj_name2id(
        simulator.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "Joint6_mimic",
    )
    mimic_qpos_id = simulator.model.jnt_qposadr[mimic_id]
    # Deliberately load only one jaw while closing. The mechanical mimic must
    # remain symmetric despite the unbalanced external force.
    simulator.data.xfrc_applied[left_finger, 1] = 20.0
    simulator.handle_payload(
        '{"seq":7,"address":1,"funcode":1,'
        '"data":{"id":6,"angle":-30,"delay_ms":0}}'
    )
    for _ in range(1000):
        simulator.step()
    symmetry_error = (
        simulator.data.qpos[mimic_qpos_id]
        + simulator.joint_positions[6]
    )
    assert abs(symmetry_error) < 5e-5


def test_full_command_moves_arm_and_gripper() -> None:
    simulator = make_simulator()
    payload = (
        '{"seq":7,"address":1,"funcode":2,"data":'
        '{"mode":0,"angle0":10,"angle1":-10,"angle2":15,'
        '"angle3":5,"angle4":-5,"angle5":8,"angle6":45}}'
    )
    command, responses = simulator.handle_payload(payload)
    assert command is not None
    assert len(responses) == 2
    for _ in range(3000):
        simulator.step()
    np.testing.assert_allclose(
        simulator.sdk_angles_deg,
        [10, -10, 15, 5, -5, 8, 45],
        atol=0.25,
    )


def test_manual_control_preserves_viewer_actuator_targets() -> None:
    simulator = make_simulator(manual_control=True)
    targets = np.asarray([0.2, -1.0, 1.047, 0.1, -0.2, 0.3, 0.02])
    simulator.data.ctrl[:] = targets

    for _ in range(10):
        simulator.step()

    np.testing.assert_allclose(simulator.data.ctrl, targets)
    _, responses = simulator.handle_payload(
        '{"seq":7,"address":1,"funcode":1,'
        '"data":{"id":1,"angle":10,"delay_ms":0}}'
    )
    assert responses[0].endswith('"recv_status":0}}')


def test_closed_gripper_command_applies_preload_beyond_joint_limit() -> None:
    simulator = make_simulator()
    simulator.handle_payload(
        '{"seq":7,"address":1,"funcode":1,'
        '"data":{"id":6,"angle":-30,"delay_ms":0}}'
    )
    assert simulator.commanded_qpos[6] == -0.005
    for _ in range(1000):
        simulator.step()
    assert simulator.data.ctrl[6] < 0.0
    assert -0.005 < simulator.joint_positions[6] < 0.0


def test_single_joint_command_preserves_other_commanded_targets() -> None:
    simulator = make_simulator()
    simulator.handle_payload(
        '{"seq":7,"address":1,"funcode":2,"data":'
        '{"mode":0,"angle0":10,"angle1":-10,"angle2":15,'
        '"angle3":5,"angle4":-5,"angle5":8,"angle6":45}}'
    )
    simulator.handle_payload(
        '{"seq":8,"address":1,"funcode":1,'
        '"data":{"id":6,"angle":30,"delay_ms":0}}'
    )
    commanded_deg = simulator.sdk_angles_deg.copy()
    for _ in range(3000):
        simulator.step()
    np.testing.assert_allclose(
        simulator.sdk_angles_deg,
        [10, -10, 15, 5, -5, 8, 30],
        atol=0.25,
    )
    # Before stepping, physical feedback remains near the initial state; the
    # preservation is therefore based on the queued target, not stale feedback.
    assert abs(commanded_deg[0]) < 0.1
