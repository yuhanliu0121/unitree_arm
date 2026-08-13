from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .scene import OBJECT_NAMES, add_object_scene, default_objects_root


JOINT_NAMES = tuple(f"Joint{i}" for i in range(7))
MIMIC_JOINT_NAME = "Joint6_mimic"
COLLISION_GEOM_PREFIX = "collision_"
COLLISION_GEOM_NAMES = (
    "collision_base",
    "collision_shoulder",
    "collision_upper_arm",
    "collision_elbow",
    "collision_forearm",
    "collision_wrist_pitch",
    "collision_wrist_roll",
    "collision_camera_main_stand",
    "collision_camera_d435i",
    "collision_left_finger",
    "collision_right_finger",
)
GO2_COLLISION_GEOM_NAME = "collision_go2_bounding_box"
D1_BODY_NAMES = (
    "base_link",
    "Link1",
    "Link2",
    "Link3",
    "Link4",
    "Link5",
    "Link6",
    "left_finger",
    "right_finger",
)


def _collision_rgba(
    color: tuple[float, float, float],
    show_collisions: bool,
) -> list[float]:
    return [*color, 0.62 if show_collisions else 0.0]


def _add_simplified_collisions(
    spec: mujoco.MjSpec,
    show_collisions: bool,
) -> None:
    """Add stable collision proxies measured from the vendor STL bounds."""
    cyan = _collision_rgba((0.05, 0.85, 1.0), show_collisions)
    orange = _collision_rgba((1.0, 0.48, 0.05), show_collisions)
    magenta = _collision_rgba((1.0, 0.05, 0.65), show_collisions)
    common = {
        "contype": 1,
        "conaffinity": 1,
        "condim": 4,
        "friction": [0.8, 0.01, 0.001],
        "group": 3,
    }

    spec.body("base_link").add_geom(
        name="collision_base",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0.0, 0.0, 0.0289],
        size=[0.059, 0.0289, 0.0],
        rgba=cyan,
        **common,
    )
    spec.body("Link1").add_geom(
        name="collision_shoulder",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[-0.00175, 0.0, 0.03465],
        size=[0.052, 0.03465, 0.0],
        rgba=cyan,
        **common,
    )
    spec.body("Link2").add_geom(
        name="collision_upper_arm",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.0009, 0.13965, -0.0271],
        size=[0.0211, 0.14265, 0.0305],
        rgba=cyan,
        **common,
    )
    spec.body("Link3").add_geom(
        name="collision_elbow",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.02295, 0.0345, -0.026],
        size=[0.03875, 0.0375, 0.032],
        rgba=orange,
        **common,
    )
    spec.body("Link4").add_geom(
        name="collision_forearm",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.00045, 0.0006, 0.075],
        size=[0.01255, 0.0263, 0.0755],
        rgba=cyan,
        **common,
    )
    spec.body("Link5").add_geom(
        name="collision_wrist_pitch",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.04175, 0.0086, -0.02509],
        size=[0.04475, 0.029, 0.02709],
        rgba=orange,
        **common,
    )
    spec.body("Link6").add_geom(
        name="collision_wrist_roll",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        fromto=[-0.0093, -0.066, 0.0378, -0.0093, 0.066, 0.0378],
        size=[0.0383, 0.0, 0.0],
        rgba=orange,
        **common,
    )

    finger_common = dict(common)
    # The physical D1 fingers have rubber contact pads. Use a higher sliding
    # coefficient than the bare plastic/painted object surfaces while keeping
    # torsional and rolling terms conservative.
    finger_common["friction"] = [5.0, 0.02, 0.002]
    # The default 20 ms MuJoCo contact time constant is too compliant for the
    # deliberately stiff task-level gripper. A 2 ms contact prevents the pads
    # from numerically passing through or squeezing past lightweight objects.
    finger_common["solref"] = [0.002, 1.0]
    finger_common["solimp"] = [0.95, 0.99, 0.001, 0.5, 2.0]
    spec.body("left_finger").add_geom(
        name="collision_left_finger",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.024, 0.006, 0.0105],
        size=[0.031, 0.013, 0.0105],
        rgba=magenta,
        **finger_common,
    )
    spec.body("right_finger").add_geom(
        name="collision_right_finger",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=[0.024, -0.006, 0.0105],
        size=[0.031, 0.013, 0.0105],
        rgba=magenta,
        **finger_common,
    )

    # Adjacent parts overlap by design at bearings and fasteners. Excluding
    # only those pairs preserves meaningful non-adjacent self-collision.
    for body1, body2 in (
        ("base_link", "Link1"),
        ("Link1", "Link2"),
        ("Link2", "Link3"),
        ("Link3", "Link4"),
        ("Link4", "Link5"),
        ("Link5", "Link6"),
        ("Link6", "left_finger"),
        ("Link6", "right_finger"),
        ("left_finger", "right_finger"),
    ):
        spec.add_exclude(
            name=f"exclude_{body1}_{body2}",
            bodyname1=body1,
            bodyname2=body2,
        )


def default_description_root() -> Path:
    return (
        Path(__file__).resolve().parents[3]
        / "d1_constrained_description_20260728"
    )


def default_ground_texture() -> Path:
    return Path(__file__).resolve().parent / "assets" / "ground_texture.png"


def default_camera_mesh() -> Path:
    return Path(__file__).resolve().parent / "assets" / "d435i_official_cad.obj"


def default_camera_mount_mesh() -> Path:
    return Path(__file__).resolve().parent / "assets" / "d435i_main_stand.obj"


def default_go2_mesh() -> Path:
    return Path(__file__).resolve().parent / "assets" / "go2" / "go2_lie_down.obj"


def _camera_transform(value: object, name: str) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 homogeneous transform")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"Invalid homogeneous row in {name}")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"Rotation in {name} is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError(f"Rotation in {name} must have determinant +1")
    return transform


def _mount_arm_on_go2(
    arm_spec: mujoco.MjSpec,
    config: dict,
    show_collisions: bool,
) -> mujoco.MjSpec:
    """Attach the D1 model to a fixed-pose, XY-movable Go2-shaped platform."""
    mesh_path = Path(config.get("mesh_path", default_go2_mesh())).resolve()
    if not mesh_path.is_file():
        raise FileNotFoundError(f"Go2 visual mesh not found: {mesh_path}")

    bounds_min = np.asarray(config["mesh_bounds_min_xyz_m"], dtype=np.float64)
    bounds_max = np.asarray(config["mesh_bounds_max_xyz_m"], dtype=np.float64)
    if bounds_min.shape != (3,) or bounds_max.shape != (3,):
        raise ValueError("Go2 mesh bounds must each contain exactly 3 values")
    if np.any(bounds_max <= bounds_min):
        raise ValueError("Go2 mesh bounding box must have positive extents")
    centre = (bounds_min + bounds_max) / 2.0
    half_size = (bounds_max - bounds_min) / 2.0

    initial_xy = np.asarray(config.get("initial_xy_m", [0.0, 0.0]), dtype=np.float64)
    if initial_xy.shape != (2,):
        raise ValueError("mobile_base.initial_xy_m must contain exactly 2 values")
    ground_clearance = float(config.get("ground_clearance_m", 0.0))
    base_z = -float(bounds_min[2]) + ground_clearance

    spec = mujoco.MjSpec()
    spec.add_mesh(name="go2_lie_down_mesh", file=str(mesh_path))
    go2 = spec.worldbody.add_body(
        name=str(config.get("body_name", "go2_base")),
        mocap=True,
        pos=[float(initial_xy[0]), float(initial_xy[1]), base_z],
    )
    go2.add_geom(
        name="go2_visual",
        type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname="go2_lie_down_mesh",
        contype=0,
        conaffinity=0,
        group=1,
        rgba=[1.0, 1.0, 1.0, 1.0],
    )
    go2.add_geom(
        name=GO2_COLLISION_GEOM_NAME,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=centre.tolist(),
        size=half_size.tolist(),
        contype=1,
        conaffinity=1,
        condim=4,
        friction=[0.8, 0.01, 0.001],
        group=3,
        rgba=_collision_rgba((0.25, 0.55, 1.0), show_collisions),
    )

    mount = _camera_transform(config["T_go2_base_d1_base"], "T_go2_base_d1_base")
    mount_position, mount_quaternion = _position_quaternion(mount)
    mount_frame = go2.add_frame(
        name="d1_mount",
        pos=mount_position,
        quat=mount_quaternion,
    )
    spec.attach(arm_spec, prefix="", suffix="", frame=mount_frame)

    # The single platform box intentionally overlaps the bolted-on D1 base.
    # It represents only external world/object collision, not platform-arm
    # self-collision.
    for body_name in D1_BODY_NAMES:
        spec.add_exclude(
            name=f"exclude_go2_{body_name}",
            bodyname1=go2.name,
            bodyname2=body_name,
        )
    return spec


def _position_quaternion(transform: np.ndarray) -> tuple[list[float], list[float]]:
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, transform[:3, :3].reshape(-1))
    return transform[:3, 3].tolist(), quaternion.tolist()


def _add_calibrated_camera(
    body: mujoco.MjsBody,
    optical_transform: np.ndarray,
    config: dict,
) -> None:
    optical_position, optical_quaternion = _position_quaternion(
        optical_transform
    )
    body.add_site(
        name=str(config["frame"]),
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        pos=optical_position,
        quat=optical_quaternion,
        size=[0.002, 0.0, 0.0],
        rgba=[0.1, 0.7, 1.0, 0.0],
        group=3,
    )

    # MuJoCo cameras look along local -Z with +Y pointing up. RealSense
    # optical frames use +Z forward and +Y down, hence a 180-degree X turn.
    optical_from_mujoco_camera = np.diag([1.0, -1.0, -1.0, 1.0])
    camera_transform = optical_transform @ optical_from_mujoco_camera
    camera_position, camera_quaternion = _position_quaternion(
        camera_transform
    )
    width = int(config["width"])
    height = int(config["height"])
    body.add_camera(
        name=str(config["name"]),
        pos=camera_position,
        quat=camera_quaternion,
        resolution=[width, height],
        # Only the ratio matters. Pixel dimensions make the compiled
        # focal lengths numerically identical to the values in K. Unlike
        # OpenCV, MuJoCo wants principal-point offsets from image centre; its
        # positive offset moves the rendered optical axis left/up.
        sensor_size=[float(width), float(height)],
        focal_pixel=[float(config["fx"]), float(config["fy"])],
        principal_pixel=[
            width / 2.0 - float(config["cx"]),
            height / 2.0 - float(config["cy"]),
        ],
    )


def _add_d435i(
    spec: mujoco.MjSpec,
    config: dict,
    mesh_path: Path,
    mount_mesh_path: Path,
    show_collisions: bool,
) -> None:
    if not config.get("enabled", True):
        return
    mesh_path = Path(mesh_path).resolve()
    if not mesh_path.is_file():
        raise FileNotFoundError(f"D435i CAD mesh not found: {mesh_path}")
    mount_mesh_path = Path(mount_mesh_path).resolve()
    if not mount_mesh_path.is_file():
        raise FileNotFoundError(
            f"D435i main stand mesh not found: {mount_mesh_path}"
        )

    link = spec.body(str(config.get("parent_link", "Link6")))
    link_from_color = _camera_transform(
        config["T_link6_color_optical"],
        "camera.T_link6_color_optical",
    )
    color_from_depth = _camera_transform(
        config["T_color_depth_optical"],
        "camera.T_color_depth_optical",
    )
    link_from_depth = link_from_color @ color_from_depth

    mesh_name = "d435i_official_cad"
    material_name = "d435i_aluminum"
    spec.add_mesh(name=mesh_name, file=str(mesh_path))
    spec.add_material(
        name=material_name,
        rgba=[0.32, 0.34, 0.37, 1.0],
        metallic=0.65,
        roughness=0.34,
    )
    mesh_position, mesh_quaternion = _position_quaternion(link_from_color)
    link.add_geom(
        name="d435i_visual",
        type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname=mesh_name,
        material=material_name,
        pos=mesh_position,
        quat=mesh_quaternion,
        group=1,
        contype=0,
        conaffinity=0,
    )

    # main_stand is exported independently in LYH_D1_FRAME_Link6, rather than
    # relative to the calibrated RGB frame.  A mismatch between the designed
    # mount and measured hand-eye pose therefore remains visible instead of
    # being hidden by construction.
    mount_mesh_name = "d435i_main_stand"
    mount_material_name = "d435i_main_stand_material"
    spec.add_mesh(name=mount_mesh_name, file=str(mount_mesh_path))
    spec.add_material(
        name=mount_material_name,
        rgba=[1.0, 0.22, 0.02, 1.0],
        metallic=0.0,
        roughness=0.52,
    )
    link.add_geom(
        name="d435i_main_stand_visual",
        type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname=mount_mesh_name,
        material=mount_material_name,
        group=1,
        contype=0,
        conaffinity=0,
    )

    collision_common = {
        "type": mujoco.mjtGeom.mjGEOM_BOX,
        "contype": 1,
        "conaffinity": 1,
        "condim": 4,
        "friction": [0.8, 0.01, 0.001],
        "group": 3,
    }
    link.add_geom(
        name="collision_camera_main_stand",
        pos=[-0.075368708772, -0.000022866946, -0.017460908006],
        quat=[0.916890031757, -0.000583760542, -0.399139226693, -0.000454535605],
        size=[0.065072925143, 0.040035982031, 0.017180994971],
        rgba=_collision_rgba((1.0, 0.48, 0.05), show_collisions),
        **collision_common,
    )
    camera_box = link_from_color.copy()
    camera_box[:3, 3] = (link_from_color @ np.asarray(
        [0.0324245356, -0.00030122605, -0.00787635474, 1.0],
        dtype=np.float64,
    ))[:3]
    camera_box_position, camera_box_quaternion = _position_quaternion(camera_box)
    link.add_geom(
        name="collision_camera_d435i",
        pos=camera_box_position,
        quat=camera_box_quaternion,
        size=[0.0449656881, 0.01288643755, 0.01266757446],
        rgba=_collision_rgba((0.45, 0.48, 0.52), show_collisions),
        **collision_common,
    )

    _add_calibrated_camera(link, link_from_color, config["color"])
    _add_calibrated_camera(link, link_from_depth, config["depth"])


def _add_ground(
    spec: mujoco.MjSpec,
    texture_path: Path,
    source_square_size_m: float,
    scene: dict | None,
) -> None:
    texture_path = Path(texture_path).resolve()
    if not texture_path.is_file():
        raise FileNotFoundError(f"Ground texture not found: {texture_path}")
    if source_square_size_m <= 0.0:
        raise ValueError("Ground texture source square size must be positive")

    texture_name = "ground_texture"
    material_name = "ground_material"
    spec.add_texture(
        name=texture_name,
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        colorspace=mujoco.mjtColorSpace.mjCOLORSPACE_SRGB,
        file=str(texture_path),
    )
    material = spec.add_material(
        name=material_name,
        rgba=[1.0, 1.0, 1.0, 1.0],
        metallic=0.0,
        roughness=0.82,
        texuniform=1,
        # The generated repeatable image mirrors two photographed squares
        # along each axis, so one full texture period spans twice this size.
        texrepeat=[
            1.0 / (2.0 * source_square_size_m),
            1.0 / (2.0 * source_square_size_m),
        ],
    )
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = texture_name
    scene_config = scene or {}
    region = scene_config.get("spawn_region", {})
    base_box = scene_config.get("base_exclusion_box", {})
    expansion = float(region.get("outer_expansion_m", 0.0))
    inner_min_x = float(base_box.get("min_x_m", -1.5))
    inner_max_x = float(base_box.get("max_x_m", 1.5))
    inner_half_y = float(base_box.get("half_width_y_m", 1.5))
    # Keep the whole forward workspace and the mobile base on the rendered
    # ground patch. The plane remains physically infinite.
    ground_center_x = (inner_min_x + inner_max_x) / 2.0
    ground_half_x = max(
        2.0, (inner_max_x - inner_min_x) / 2.0 + expansion + 0.5
    )
    ground_half_y = max(2.0, inner_half_y + expansion + 0.5)
    spec.worldbody.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[ground_half_x, ground_half_y, 0.05],
        pos=[ground_center_x, 0.0, -0.001],
        material=material_name,
        contype=1,
        conaffinity=1,
    )


def _add_skybox(spec: mujoco.MjSpec) -> None:
    """Add a neutral Isaac-Lab-like background without changing lighting."""
    spec.add_texture(
        name="skybox",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.42, 0.50, 0.58],
        rgb2=[0.76, 0.80, 0.83],
        width=512,
        height=3072,
    )


def build_model(
    description_root: Path,
    controller: dict,
    *,
    simulation: dict | None = None,
    show_collisions: bool = False,
    scene: dict | None = None,
    objects_root: Path | None = None,
    ground_texture_path: Path | None = None,
    camera: dict | None = None,
    camera_mesh_path: Path | None = None,
    camera_mount_mesh_path: Path | None = None,
    mobile_base: dict | None = None,
) -> mujoco.MjModel:
    description_root = Path(description_root).resolve()
    urdf_path = description_root / "urdf" / "d1_description.urdf"
    mesh_dir = description_root / "meshes"
    if not urdf_path.is_file():
        raise FileNotFoundError(f"D1 URDF not found: {urdf_path}")

    urdf_root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    for link in urdf_root.findall("link"):
        if link.attrib.get("name") != "Link6":
            continue
        for element in list(link):
            if (
                element.tag in {"visual", "collision"}
                and element.attrib.get("name", "").startswith("wrist_camera_")
            ):
                link.remove(element)
    xml = ET.tostring(urdf_root, encoding="unicode").replace(
        "package://d1_constrained_description/meshes/",
        "",
    )
    assets = {path.name: path.read_bytes() for path in mesh_dir.glob("*.STL")}
    spec = mujoco.MjSpec.from_string(xml, assets=assets)

    # The vendor mesh collisions overlap at nominal zero. Keep the STL geoms as
    # visual-only and use explicitly measured primitives for contact dynamics.
    for geom in spec.geoms:
        geom.contype = 0
        geom.conaffinity = 0
        geom.group = 1
        if show_collisions:
            geom.rgba = [*geom.rgba[:3], 0.20]

    _add_simplified_collisions(spec, show_collisions)
    if camera is not None:
        _add_d435i(
            spec,
            camera,
            camera_mesh_path or default_camera_mesh(),
            camera_mount_mesh_path or default_camera_mount_mesh(),
            show_collisions,
        )

    # Fixed URDF links are fused into their parent by MuJoCo. A retained site
    # gives the scene-state bridge the exact physical tcp_link frame pose.
    spec.body("tcp_link").add_site(
        name="debug_tcp_site",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.002, 0.0, 0.0],
        rgba=[1.0, 1.0, 0.0, 0.0],
        group=3,
    )

    # Position-served hardware compensates gravity. Applying MuJoCo's body
    # gravity compensation avoids artificial steady-state sag in this first
    # position-control model.
    for body in spec.bodies:
        body.gravcomp = 1.0

    if mobile_base is not None and mobile_base.get("enabled", True):
        spec = _mount_arm_on_go2(spec, mobile_base, show_collisions)

    spec.modelname = "unitree_d1_protocol_sim"
    # URDF geoms with both collision masks cleared are otherwise discarded by
    # MuJoCo as unused visual geometry during compilation.
    spec.compiler.discardvisual = False
    spec.option.timestep = float(controller["physics_timestep_s"])
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if simulation is not None:
        cone = str(simulation.get("friction_cone", "pyramidal")).lower()
        if cone not in {"pyramidal", "elliptic"}:
            raise ValueError(f"Unsupported MuJoCo friction cone: {cone}")
        spec.option.cone = int(
            mujoco.mjtCone.mjCONE_ELLIPTIC
            if cone == "elliptic"
            else mujoco.mjtCone.mjCONE_PYRAMIDAL
        )
        spec.option.impratio = float(
            simulation.get("friction_impedance_ratio", 1.0)
        )
        spec.option.noslip_iterations = int(
            simulation.get("noslip_iterations", 0)
        )

    gripper_mimic = spec.add_equality(
        name="gripper_mimic",
        type=mujoco.mjtEq.mjEQ_JOINT,
        name1=MIMIC_JOINT_NAME,
        name2="Joint6",
        data=[0.0, -1.0] + [0.0] * 9,
    )
    # The vendor gripper is mechanically symmetric. MuJoCo equality
    # constraints are compliant by default (20 ms), which lets one finger lag
    # under an asymmetric contact load. Match the stiff 2 ms contact model so
    # the passive mimic finger remains opposite the driven finger.
    gripper_mimic.solref = [0.002, 1.0]
    gripper_mimic.solimp = [0.99, 0.999, 0.0001, 0.5, 2.0]

    for index, name in enumerate(JOINT_NAMES):
        joint = spec.joint(name)
        if index < 6:
            kp = float(controller["arm_kp"])
            kv = float(controller["arm_kv"])
            damping = float(controller["arm_joint_damping"])
            armature = float(controller["arm_joint_armature"])
        else:
            kp = float(controller["gripper_kp"])
            kv = float(controller["gripper_kv"])
            damping = float(controller["gripper_joint_damping"])
            armature = float(controller["gripper_joint_armature"])

        joint.damping = [damping, 0.0, 0.0]
        joint.armature = armature
        lower, upper = joint.range
        control_lower = lower
        if index == 6:
            control_lower -= float(
                controller.get("gripper_closing_preload_m", 0.0)
            )
        spec.add_actuator(
            name=f"{name}_position",
            target=name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            gaintype=mujoco.mjtGain.mjGAIN_FIXED,
            gainprm=[kp] + [0.0] * 9,
            biastype=mujoco.mjtBias.mjBIAS_AFFINE,
            biasprm=[0.0, -kp, -kv] + [0.0] * 7,
            ctrllimited=1,
            ctrlrange=[control_lower, upper],
        )

    _add_ground(
        spec,
        ground_texture_path or default_ground_texture(),
        float((scene or {}).get("ground_texture_square_size_m", 0.6)),
        scene,
    )
    _add_skybox(spec)
    spec.worldbody.add_light(
        name="key_light",
        pos=[1.0, -1.0, 1.5],
        dir=[-1.0, 1.0, -1.5],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
    )
    spec.worldbody.add_camera(
        name="overview",
        pos=[1.1, -1.1, 0.85],
        xyaxes=[0.707, 0.707, 0.0, -0.35, 0.35, 0.87],
    )
    if scene is not None:
        add_object_scene(
            spec,
            objects_root or default_objects_root(),
            scene,
            show_collisions=show_collisions,
        )
    model = spec.compile()
    if scene is not None:
        linear_damping = float(scene.get("object_linear_damping", 0.0))
        angular_damping = float(scene.get("object_angular_damping", 0.0))
        for name in OBJECT_NAMES:
            joint_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_JOINT,
                f"object_{name}_free",
            )
            dof_address = model.jnt_dofadr[joint_id]
            model.dof_damping[dof_address:dof_address + 3] = linear_damping
            model.dof_damping[dof_address + 3:dof_address + 6] = (
                angular_damping
            )
    return model
