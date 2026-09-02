#!/usr/bin/env python3
"""Build the Blender scene used to annotate a D1 bowl grasp TCP pose.

Run with the source bowl blend already open, for example:

  blender -b simulation/objects/bowl/bowl.blend \
    --python tools/blender/create_bowl_grasp_annotation.py \
    -- simulation/objects/bowl/bowl_grasp_annotation.blend

The movable parent is GRIPPER_TCP_POSE.  Its transform is the pose of
``tcp_link``; all Link6 and finger meshes are children expressed relative to
that frame.  The fingers are placed at the URDF maximum opening (Joint6=0.03).
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

import bpy
from mathutils import Matrix


WORKSPACE = Path(__file__).resolve().parents[3]
MESH_DIR = WORKSPACE / "src" / "d1_description" / "meshes"
DEFAULT_OUTPUT = (
    WORKSPACE / "simulation" / "objects" / "bowl" / "bowl_grasp_annotation.blend"
)


def output_path() -> Path:
    if "--" not in sys.argv:
        return DEFAULT_OUTPUT
    arguments = sys.argv[sys.argv.index("--") + 1 :]
    return Path(arguments[0]).expanduser().resolve() if arguments else DEFAULT_OUTPUT


def translation(x: float, y: float, z: float) -> Matrix:
    return Matrix.Translation((x, y, z))


def urdf_rpy(roll: float, pitch: float, yaw: float) -> Matrix:
    """Return URDF fixed-axis RPY rotation Rz(yaw) Ry(pitch) Rx(roll)."""
    rx = Matrix.Rotation(roll, 4, "X")
    ry = Matrix.Rotation(pitch, 4, "Y")
    rz = Matrix.Rotation(yaw, 4, "Z")
    return rz @ ry @ rx


def urdf_origin(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> Matrix:
    return translation(*xyz) @ urdf_rpy(*rpy)


def material(name: str, rgba: tuple[float, float, float, float], metallic: float = 0.0):
    value = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    value.diffuse_color = rgba
    value.metallic = metallic
    value.roughness = 0.34
    return value


def import_stl(name: str, filename: str, collection: bpy.types.Collection):
    before = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=str(MESH_DIR / filename))
    imported = list(set(bpy.data.objects) - before)
    if len(imported) != 1:
        raise RuntimeError(f"expected one object from {filename}, got {len(imported)}")
    obj = imported[0]
    obj.name = name
    for owner in list(obj.users_collection):
        owner.objects.unlink(obj)
    collection.objects.link(obj)
    return obj


def move_to_collection(obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    for owner in list(obj.users_collection):
        owner.objects.unlink(obj)
    collection.objects.link(obj)


def main() -> None:
    bowl = bpy.data.objects.get("bowl")
    if bowl is None or bowl.type != "MESH":
        raise RuntimeError(
            "open simulation/objects/bowl/bowl.blend before running this script"
        )

    for old_name in ("GRIPPER_TCP_POSE", "LINK6_ORIGIN", "BOWL_CENTER"):
        old = bpy.data.objects.get(old_name)
        if old is not None:
            bpy.data.objects.remove(old, do_unlink=True)
    for old_name in ("D1_GRIPPER", "REFERENCE"):
        old = bpy.data.collections.get(old_name)
        if old is not None:
            bpy.data.collections.remove(old)

    reference_collection = bpy.data.collections.new("REFERENCE")
    gripper_collection = bpy.data.collections.new("D1_GRIPPER")
    bpy.context.scene.collection.children.link(reference_collection)
    bpy.context.scene.collection.children.link(gripper_collection)

    bowl.name = "BOWL_REFERENCE"
    move_to_collection(bowl, reference_collection)
    bowl.hide_select = True
    bowl["geometry"] = "diameter approximately 0.115 m; height approximately 0.050 m"

    bowl_center = bpy.data.objects.new("BOWL_CENTER", None)
    bowl_center.empty_display_type = "SPHERE"
    bowl_center.empty_display_size = 0.008
    bowl_center.location = (0.0, 0.0, 0.025)
    bowl_center.hide_select = True
    reference_collection.objects.link(bowl_center)

    tcp = bpy.data.objects.new("GRIPPER_TCP_POSE", None)
    tcp.empty_display_type = "ARROWS"
    tcp.empty_display_size = 0.045
    # Start above the bowl with Link6 +Z (the approach direction) pointing down.
    tcp.location = (0.0, 0.0, 0.12)
    tcp.rotation_mode = "XYZ"
    tcp.rotation_euler = (math.pi, 0.0, 0.0)
    tcp["pose_semantics"] = "transform of URDF tcp_link in the bowl/world frame"
    tcp["move_this_object"] = True
    tcp["joint6_m"] = 0.03
    tcp["nominal_jaw_aperture_m"] = 0.06
    gripper_collection.objects.link(tcp)

    link6 = import_stl("D1_Link6", "Link6.STL", gripper_collection)
    left = import_stl("D1_left_finger", "Link7_1.STL", gripper_collection)
    right = import_stl("D1_right_finger", "Link7_2.STL", gripper_collection)

    white = material("D1 white", (0.92, 0.94, 0.97, 1.0), metallic=0.15)
    finger_white = material("D1 finger white", (1.0, 1.0, 1.0, 1.0), metallic=0.08)
    link6.data.materials.clear()
    link6.data.materials.append(white)
    for finger in (left, right):
        finger.data.materials.clear()
        finger.data.materials.append(finger_white)

    # URDF: T_Link6_tcp.  Parenting below uses transforms from tcp_link to each
    # mesh so the parent object's transform is directly the annotated TCP pose.
    link6_from_tcp = translation(0.00038, 0.0, 0.1256)
    tcp_from_link6 = link6_from_tcp.inverted()

    link6.matrix_parent_inverse = Matrix.Identity(4)
    link6.parent = tcp
    link6.matrix_local = tcp_from_link6

    # Joint6=+0.03 along local (0,0,-1).
    link6_from_left = urdf_origin(
        (-0.0056012, -0.02100301, 0.0706),
        (-1.5714, -1.5708, 0.0),
    ) @ translation(0.0, 0.0, -0.03)
    # Mimic joint value=-0.03 along local (0,0,+1), also a -0.03 translation.
    link6_from_right = urdf_origin(
        (-0.0056388, 0.02101013, 0.0706),
        (1.5702, -1.5708, 0.0),
    ) @ translation(0.0, 0.0, -0.03)

    for obj, local_transform in (
        (left, tcp_from_link6 @ link6_from_left),
        (right, tcp_from_link6 @ link6_from_right),
    ):
        obj.matrix_parent_inverse = Matrix.Identity(4)
        obj.parent = tcp
        obj.matrix_local = local_transform

    link6_origin = bpy.data.objects.new("LINK6_ORIGIN", None)
    link6_origin.empty_display_type = "PLAIN_AXES"
    link6_origin.empty_display_size = 0.025
    gripper_collection.objects.link(link6_origin)
    link6_origin.parent = tcp
    link6_origin.matrix_parent_inverse = Matrix.Identity(4)
    link6_origin.matrix_local = tcp_from_link6
    link6_origin.hide_select = True

    instructions = bpy.data.texts.get("README_BOWL_GRASP") or bpy.data.texts.new(
        "README_BOWL_GRASP"
    )
    instructions.clear()
    instructions.write(
        "Select only GRIPPER_TCP_POSE and move/rotate it to annotate the grasp.\n"
        "Its transform is the desired tcp_link pose in the bowl/world frame.\n"
        "BOWL_REFERENCE is locked against accidental selection.\n"
        "The fingers use URDF Joint6=0.03 m (maximum opening, nominal 60 mm).\n"
        "The 115 mm bowl cannot be enclosed from both outer sides; place one finger "
        "inside and one outside the rim.\n"
        "Do not apply transforms to GRIPPER_TCP_POSE before saving.\n"
    )

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "METERS"
    scene.unit_settings.scale_length = 1.0
    scene.world.color = (0.055, 0.065, 0.085)
    scene["annotation_frame"] = "bowl/world; ground plane is z=0"
    scene["annotation_object"] = "GRIPPER_TCP_POSE"
    scene["source_urdf"] = "src/d1_description/urdf/d1_description.urdf"

    bpy.ops.object.select_all(action="DESELECT")
    tcp.hide_select = False
    tcp.select_set(True)
    bpy.context.view_layer.objects.active = tcp

    destination = output_path()
    destination.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(destination))
    print(f"SAVED_BOWL_GRASP_SCENE {destination}")


if __name__ == "__main__":
    main()
