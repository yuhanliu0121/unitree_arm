from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .scene import MESH_UP_TO_Z_QUAT, OBJECT_NAMES


@dataclass(frozen=True)
class MarkerSpec:
    """ROS-independent description of one RViz marker."""

    name: str
    shape: str
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    scale: tuple[float, float, float]
    resource: str | None = None
    opaque: bool = False


def _quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    quaternion_wxyz = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion_wxyz, rotation.reshape(-1))
    return (
        float(quaternion_wxyz[1]),
        float(quaternion_wxyz[2]),
        float(quaternion_wxyz[3]),
        float(quaternion_wxyz[0]),
    )


def _mesh_local_rotation() -> np.ndarray:
    rotation = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(
        rotation,
        np.asarray(MESH_UP_TO_Z_QUAT, dtype=np.float64),
    )
    return rotation.reshape(3, 3)


def object_mesh_specs(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    objects_root: Path,
) -> tuple[MarkerSpec, ...]:
    """Describe original object meshes at their MuJoCo runtime poses."""
    objects_root = Path(objects_root).resolve()
    mesh_local_rotation = _mesh_local_rotation()
    specs = []
    for name in OBJECT_NAMES:
        body_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            f"object_{name}",
        )
        if body_id < 0:
            continue
        mesh_path = objects_root / name / f"{name}-obj" / f"{name}.obj"
        if not mesh_path.is_file():
            raise FileNotFoundError(f"RViz object mesh not found: {mesh_path}")
        body_rotation = data.xmat[body_id].reshape(3, 3)
        marker_rotation = body_rotation @ mesh_local_rotation
        specs.append(
            MarkerSpec(
                name=name,
                shape="mesh",
                position=tuple(float(value) for value in data.xpos[body_id]),
                orientation_xyzw=_quaternion_xyzw(marker_rotation),
                scale=(1.0, 1.0, 1.0),
                resource=mesh_path.as_uri(),
                opaque=True,
            )
        )
    return tuple(specs)


def _primitive_spec(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_id: int,
) -> tuple[MarkerSpec, ...]:
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    if not name:
        name = f"geom_{geom_id}"
    geom_type = model.geom_type[geom_id]
    size = model.geom_size[geom_id]
    position = np.asarray(data.geom_xpos[geom_id], dtype=np.float64)
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    orientation = _quaternion_xyzw(rotation)

    if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
        # MuJoCo planes are physically infinite. RViz receives the finite
        # rendering patch configured by the first two geom size values.
        scale = (2.0 * float(size[0]), 2.0 * float(size[1]), 0.002)
        return (
            MarkerSpec(
                name=name,
                shape="box",
                position=tuple(float(value) for value in position),
                orientation_xyzw=orientation,
                scale=scale,
                opaque=True,
            ),
        )
    if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
        diameter = 2.0 * float(size[0])
        scale = (diameter, diameter, diameter)
        shape = "sphere"
    elif geom_type == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        scale = tuple(2.0 * float(value) for value in size)
        shape = "sphere"
    elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
        diameter = 2.0 * float(size[0])
        scale = (diameter, diameter, 2.0 * float(size[1]))
        shape = "cylinder"
    elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
        scale = tuple(2.0 * float(value) for value in size)
        shape = "box"
    elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
        radius = float(size[0])
        half_length = float(size[1])
        diameter = 2.0 * radius
        centre = MarkerSpec(
            name=f"{name}_shaft",
            shape="cylinder",
            position=tuple(float(value) for value in position),
            orientation_xyzw=orientation,
            scale=(diameter, diameter, 2.0 * half_length),
        )
        caps = []
        for suffix, sign in (("negative", -1.0), ("positive", 1.0)):
            cap_position = position + rotation @ np.asarray(
                [0.0, 0.0, sign * half_length]
            )
            caps.append(
                MarkerSpec(
                    name=f"{name}_{suffix}_cap",
                    shape="sphere",
                    position=tuple(float(value) for value in cap_position),
                    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                    scale=(diameter, diameter, diameter),
                )
            )
        return (centre, *caps)
    else:
        raise ValueError(
            f"Unsupported physical collision geom type {geom_type} for {name}"
        )

    return (
        MarkerSpec(
            name=name,
            shape=shape,
            position=tuple(float(value) for value in position),
            orientation_xyzw=orientation,
            scale=scale,
        ),
    )


def physical_collision_specs(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> tuple[MarkerSpec, ...]:
    """Describe every MuJoCo geom that participates in contact matching."""
    specs = []
    for geom_id in range(model.ngeom):
        if not (
            model.geom_contype[geom_id]
            or model.geom_conaffinity[geom_id]
        ):
            continue
        specs.extend(_primitive_spec(model, data, geom_id))
    return tuple(specs)


class RosScenePublisher:
    """Publish MuJoCo visual and physical geometry as independent RViz layers."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        objects_root: Path,
        config: dict,
    ) -> None:
        try:
            import rclpy
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from rclpy.signals import SignalHandlerOptions
            from visualization_msgs.msg import Marker, MarkerArray
        except ImportError as exc:
            raise RuntimeError(
                "ROS scene publishing requires sourcing "
                "/opt/ros/humble/setup.zsh before starting d1-mujoco-sim"
            ) from exc

        self._rclpy = rclpy
        self._Marker = Marker
        self._MarkerArray = MarkerArray
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(
                args=[],
                signal_handler_options=SignalHandlerOptions.NO,
            )
        self._node = rclpy.create_node("d1_mujoco_scene")
        self._model = model
        self._data = data
        self._objects_root = Path(objects_root).resolve()
        self._frame_id = str(config.get("frame_id", "base_link"))
        self.publish_rate_hz = float(config.get("publish_rate_hz", 25.0))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mesh_publisher = self._node.create_publisher(
            MarkerArray,
            str(config.get("object_mesh_topic", "/d1_mujoco/object_meshes")),
            qos,
        )
        self._collision_publisher = self._node.create_publisher(
            MarkerArray,
            str(
                config.get(
                    "physical_collision_topic",
                    "/d1_mujoco/physical_collisions",
                )
            ),
            qos,
        )

    def _message(
        self,
        specs: tuple[MarkerSpec, ...],
        namespace: str,
        collision: bool,
    ):
        result = self._MarkerArray()
        stamp = self._node.get_clock().now().to_msg()
        marker_types = {
            "box": self._Marker.CUBE,
            "cylinder": self._Marker.CYLINDER,
            "mesh": self._Marker.MESH_RESOURCE,
            "sphere": self._Marker.SPHERE,
        }
        for marker_id, spec in enumerate(specs):
            marker = self._Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = marker_types[spec.shape]
            marker.action = self._Marker.ADD
            marker.pose.position.x = spec.position[0]
            marker.pose.position.y = spec.position[1]
            marker.pose.position.z = spec.position[2]
            marker.pose.orientation.x = spec.orientation_xyzw[0]
            marker.pose.orientation.y = spec.orientation_xyzw[1]
            marker.pose.orientation.z = spec.orientation_xyzw[2]
            marker.pose.orientation.w = spec.orientation_xyzw[3]
            marker.scale.x = spec.scale[0]
            marker.scale.y = spec.scale[1]
            marker.scale.z = spec.scale[2]
            if spec.resource is not None:
                marker.mesh_resource = spec.resource
                marker.mesh_use_embedded_materials = True
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            elif collision:
                marker.color.r = 0.05
                marker.color.g = 0.85
                marker.color.b = 1.0
                marker.color.a = 1.0 if spec.opaque else 0.55
            result.markers.append(marker)
        return result

    def publish(self) -> None:
        self._mesh_publisher.publish(
            self._message(
                object_mesh_specs(
                    self._model,
                    self._data,
                    self._objects_root,
                ),
                "object_meshes",
                False,
            )
        )
        self._collision_publisher.publish(
            self._message(
                physical_collision_specs(self._model, self._data),
                "physical_collisions",
                True,
            )
        )

    def close(self) -> None:
        self._node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()
