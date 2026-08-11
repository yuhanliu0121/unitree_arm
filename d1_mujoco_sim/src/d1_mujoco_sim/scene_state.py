from __future__ import annotations

import socket

import mujoco
import numpy as np

from .scene import OBJECT_NAMES


SCENE_STATE_MAGIC = "D1SCENE"
SCENE_STATE_VERSION = 1


def _pose_line(
    name: str,
    position,
    quaternion,
) -> str:
    values = (*position, *quaternion)
    return name + " " + " ".join(
        f"{float(value):.9f}" for value in values
    )


def _geom_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    name: str,
):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        return None
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, data.geom_xmat[geom_id])
    return data.geom_xpos[geom_id], quaternion


def _site_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    name: str,
):
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site_id < 0:
        return None
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, data.site_xmat[site_id])
    return data.site_xpos[site_id], quaternion


def _pose_in_body_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    position: np.ndarray,
    quaternion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        body_name,
    )
    if body_id < 0:
        raise ValueError(f"Unknown reference body: {body_name}")
    world_from_body = data.xmat[body_id].reshape(3, 3)
    body_position = world_from_body.T @ (position - data.xpos[body_id])
    world_from_pose = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(world_from_pose, quaternion)
    body_from_pose = world_from_body.T @ world_from_pose.reshape(3, 3)
    body_quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(body_quaternion, body_from_pose.reshape(-1))
    return body_position, body_quaternion


def scene_state_payload(model: mujoco.MjModel, data: mujoco.MjData) -> str:
    """Serialize physical poses in MoveIt's ``base_link`` planning frame."""
    lines = [f"{SCENE_STATE_MAGIC} {SCENE_STATE_VERSION} {data.time:.9f}"]
    for name in OBJECT_NAMES:
        body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            f"object_{name}",
        )
        if body_id < 0:
            continue
        position = data.xpos[body_id]
        quaternion = data.xquat[body_id]  # MuJoCo order: w, x, y, z.
        position, quaternion = _pose_in_body_frame(
            model, data, "base_link", position, quaternion
        )
        lines.append(_pose_line(name, position, quaternion))

    # These diagnostics share the same compact pose-shaped record so the
    # simulation bridge stays dependency-free. They make the physical grasp
    # independently verifiable instead of trusting MoveIt's attached object.
    tcp_pose = _site_pose(model, data, "debug_tcp_site")
    if tcp_pose is not None:
        lines.append(
            _pose_line(
                "debug_tcp_link",
                *_pose_in_body_frame(model, data, "base_link", *tcp_pose),
            )
        )

    for output_name, geom_name in (
        ("debug_left_finger", "collision_left_finger"),
        ("debug_right_finger", "collision_right_finger"),
    ):
        pose = _geom_pose(model, data, geom_name)
        if pose is not None:
            lines.append(
                _pose_line(
                    output_name,
                    *_pose_in_body_frame(model, data, "base_link", *pose),
                )
            )

    cube_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "object_collision_yellow_cube"
    )
    finger_geoms = {
        "left": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "collision_left_finger"
        ),
        "right": mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "collision_right_finger"
        ),
    }
    contact_counts = {"left": 0, "right": 0}
    contact_forces = {"left": 0.0, "right": 0.0}
    if cube_geom >= 0:
        for contact_index, contact in enumerate(data.contact):
            pair = {int(contact.geom1), int(contact.geom2)}
            for side, finger_geom in finger_geoms.items():
                if finger_geom >= 0 and pair == {cube_geom, finger_geom}:
                    contact_counts[side] += 1
                    force = np.empty(6, dtype=np.float64)
                    mujoco.mj_contactForce(
                        model, data, contact_index, force
                    )
                    contact_forces[side] += abs(float(force[0]))
    lines.append(
        _pose_line(
            "debug_cube_contacts",
            [
                contact_counts["left"],
                contact_counts["right"],
                contact_counts["left"] + contact_counts["right"],
            ],
            [1.0, 0.0, 0.0, 0.0],
        )
    )
    gripper_joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "Joint6"
    )
    gripper_position = (
        float(data.qpos[model.jnt_qposadr[gripper_joint]])
        if gripper_joint >= 0 else 0.0
    )
    lines.append(
        _pose_line(
            "debug_gripper_state",
            [
                gripper_position,
                contact_forces["left"],
                contact_forces["right"],
            ],
            [1.0, 0.0, 0.0, 0.0],
        )
    )
    return "\n".join(lines)


class SceneStateUdpPublisher:
    """Send MuJoCo-only truth poses without loading ROS into the simulator."""

    def __init__(self, host: str, port: int) -> None:
        self.destination = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def write(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.socket.sendto(
            scene_state_payload(model, data).encode("ascii"),
            self.destination,
        )
