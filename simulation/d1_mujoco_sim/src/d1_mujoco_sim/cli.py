from __future__ import annotations

import argparse
from contextlib import nullcontext
import json
import logging
import math
from pathlib import Path

import mujoco.viewer
import yaml

from .dds_service import D1DdsService
from .model import build_model, default_description_root
from .scene import OBJECT_NAMES, default_objects_root, sample_scene_layout
from .scene_state import SceneStateUdpPublisher
from .simulator import D1Simulator


def _parse_arm_pose_rad(value: str) -> tuple[float, ...]:
    try:
        pose = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(
            "expected a JSON list such as '[0, -1.54, 1.55, 0, 0, 0]'"
        ) from exc
    if not isinstance(pose, list) or len(pose) != 6:
        raise argparse.ArgumentTypeError(
            "initial arm pose must contain exactly six joint values"
        )
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not math.isfinite(item)
        for item in pose
    ):
        raise argparse.ArgumentTypeError(
            "initial arm pose values must be finite numbers"
        )
    return tuple(float(item) for item in pose)


def parse_args() -> argparse.Namespace:
    package_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Run a D1-compatible MuJoCo DDS service",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=package_root / "config" / "sim.yaml",
    )
    parser.add_argument(
        "--description-root",
        type=Path,
        default=default_description_root(),
    )
    parser.add_argument(
        "--objects-root",
        type=Path,
        default=default_objects_root(),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Override the configured object placement seed",
    )
    layout_group = parser.add_mutually_exclusive_group()
    layout_group.add_argument(
        "--fixed-layout",
        action="store_true",
        help="Use the configured fixed object poses (the development default)",
    )
    layout_group.add_argument(
        "--random-layout",
        action="store_true",
        help="Randomly place the objects and bin, using --seed when provided",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without the MuJoCo viewer",
    )
    parser.add_argument(
        "--show-collisions",
        action="store_true",
        help="Show simplified collision proxies and contact diagnostics",
    )
    parser.add_argument(
        "--manual-control",
        action="store_true",
        help=(
            "Let the MuJoCo viewer Control sliders drive the joints directly "
            "instead of accepting DDS motion commands"
        ),
    )
    parser.add_argument(
        "--initial-arm-pose",
        type=_parse_arm_pose_rad,
        metavar="'[J0,J1,J2,J3,J4,J5]'",
        help=(
            "Initial arm joint targets in radians; requires --manual-control"
        ),
    )
    parser.add_argument(
        "--ros-camera",
        action="store_true",
        help="Publish RGB, depth, CameraInfo and TF on ROS 2 topics",
    )
    parser.add_argument(
        "--ros-scene",
        action="store_true",
        help="Publish object meshes and physical collisions for RViz",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional run duration in seconds",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    if args.initial_arm_pose is not None and not args.manual_control:
        parser.error("--initial-arm-pose requires --manual-control")
    return args


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if args.seed is not None:
        config["scene"]["random_seed"] = args.seed
    if args.fixed_layout:
        config["scene"]["placement_mode"] = "fixed"
    elif args.random_layout:
        config["scene"]["placement_mode"] = "random"
    if args.initial_arm_pose is not None:
        initial_angles_deg = list(
            config["simulation"]["initial_sdk_angles_deg"]
        )
        initial_angles_deg[:6] = [
            math.degrees(value) for value in args.initial_arm_pose
        ]
        config["simulation"]["initial_sdk_angles_deg"] = initial_angles_deg
    controller_config = dict(config["controller"])
    controller_config["physics_timestep_s"] = config["simulation"][
        "physics_timestep_s"
    ]
    model = build_model(
        args.description_root,
        controller_config,
        simulation=config["simulation"],
        show_collisions=args.show_collisions,
        scene=config["scene"],
        objects_root=args.objects_root,
        camera=config.get("camera"),
        mobile_base=config.get("mobile_base"),
    )
    object_positions = []
    for name in OBJECT_NAMES:
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"object_{name}"
        )
        position = model.body_pos[body_id]
        object_positions.append(
            f"{name}=({position[0]:.3f},{position[1]:.3f})"
        )
    trash_bin_pose = sample_scene_layout(config["scene"]).trash_bin
    object_positions.append(
        f"trash_bin=({trash_bin_pose.x:.3f},{trash_bin_pose.y:.3f})"
    )
    logging.getLogger(__name__).info(
        "Object layout mode=%s seed=%s: %s",
        config["scene"].get("placement_mode", "random"),
        config["scene"]["random_seed"],
        " ".join(object_positions),
    )
    simulator = D1Simulator(model, config, manual_control=args.manual_control)
    scene_state_config = config.get("scene_state", {})
    scene_state_publisher = (
        SceneStateUdpPublisher(
            str(scene_state_config.get("host", "127.0.0.1")),
            int(scene_state_config.get("port", 15002)),
        )
        if scene_state_config.get("enabled", True)
        else None
    )
    dds = D1DdsService(
        domain_id=config["dds"]["domain_id"],
        command_topic=config["dds"]["command_topic"],
        feedback_topic=config["dds"]["feedback_topic"],
        joint_feedback_topic=config["dds"]["joint_feedback_topic"],
    )

    logging.getLogger(__name__).info(
        "D1 MuJoCo service ready: domain=%s command=%s feedback=%.2f Hz",
        config["dds"]["domain_id"],
        config["dds"]["command_topic"],
        config["dds"]["feedback_rate_hz"],
    )
    if args.manual_control:
        logging.getLogger(__name__).info(
            "Manual joint control enabled: use the viewer's Control sliders; "
            "DDS motion commands will be rejected"
        )
        if args.initial_arm_pose is not None:
            logging.getLogger(__name__).info(
                "Initial manual arm pose (rad): %s",
                list(args.initial_arm_pose),
            )
    if config.get("camera", {}).get("enabled", False):
        camera_config = config["camera"]
        logging.getLogger(__name__).info(
            "D435i cameras ready: color=%sx%s@%s depth=%sx%s@%s",
            camera_config["color"]["width"],
            camera_config["color"]["height"],
            camera_config["color"]["fps"],
            camera_config["depth"]["width"],
            camera_config["depth"]["height"],
            camera_config["depth"]["fps"],
        )
    context = (
        nullcontext(None)
        if args.headless
        else mujoco.viewer.launch_passive(model, simulator.data)
    )
    try:
        with context as viewer:
            camera_publisher = None
            scene_publisher = None
            if args.ros_camera:
                from .ros_camera import RosCameraPublisher

                camera_publisher = RosCameraPublisher(
                    model,
                    simulator.data,
                    config["camera"],
                )
            if args.ros_scene:
                from .ros_scene import RosScenePublisher

                scene_publisher = RosScenePublisher(
                    model,
                    simulator.data,
                    args.objects_root,
                    config.get("ros_visualization", {}),
                )
            if viewer is not None:
                # Seed a useful orbit view without locking the user to the
                # model's fixed overview camera.
                mujoco.mjv_defaultFreeCamera(model, viewer.cam)
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                viewer.cam.lookat[:] = [0.0, 0.0, 0.28]
                viewer.cam.distance = 1.25
                viewer.cam.azimuth = 135.0
                viewer.cam.elevation = -22.0
                if args.show_collisions:
                    viewer.opt.geomgroup[3] = 1
                    viewer.opt.geomgroup[4] = 1
            try:
                simulator.run(
                    dds.take_commands,
                    dds.write_feedback,
                    dds.write_joint_feedback,
                    write_scene_state=(
                        lambda: scene_state_publisher.write(
                            model,
                            simulator.data,
                        )
                    ) if scene_state_publisher is not None else None,
                    publish_camera=(
                        camera_publisher.publish
                        if camera_publisher is not None
                        else None
                    ),
                    camera_rate_hz=(
                        camera_publisher.publish_rate_hz
                        if camera_publisher is not None
                        else 10.0
                    ),
                    publish_visualization=(
                        scene_publisher.publish
                        if scene_publisher is not None
                        else None
                    ),
                    visualization_rate_hz=(
                        scene_publisher.publish_rate_hz
                        if scene_publisher is not None
                        else 25.0
                    ),
                    viewer=viewer,
                    duration_s=args.duration,
                )
            finally:
                if scene_publisher is not None:
                    scene_publisher.close()
                if camera_publisher is not None:
                    camera_publisher.close()
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("D1 MuJoCo service stopped")


if __name__ == "__main__":
    main()
