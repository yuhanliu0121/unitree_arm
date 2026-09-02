"""Build a Blender scene with independently rooted Go2 and D1 assemblies.

Run with Blender::

    blender --background --factory-startup --python build_mount_scene.py
"""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Vector


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
GO2_ROOT = HERE / "vendor" / "go2_description"
D1_ROOT = WORKSPACE / "src" / "d1_description"
OUTPUT = HERE / "go2_d1_mount.blend"

# Official Unitree MuJoCo ``stand_down_joint_pos`` (lie-down/crouched pose).
# The SDK motor array is ordered FR, FL, RR, RL, with hip, thigh, calf within
# each leg.  The named mapping below deliberately remains independent of the
# URDF traversal order.
GO2_STAND_DOWN = {
    "FL": (0.0473455, 1.22187, -2.44375),
    "FR": (-0.0473455, 1.22187, -2.44375),
    "RL": (0.0473455, 1.22187, -2.44375),
    "RR": (-0.0473455, 1.22187, -2.44375),
}
GO2_JOINTS = {
    f"{leg}_{joint}_joint": angle
    for leg, angles in GO2_STAND_DOWN.items()
    for joint, angle in zip(("hip", "thigh", "calf"), angles)
}
D1_JOINTS = {
    "Joint0": 0.0,
    "Joint1": -1.5,
    "Joint2": 1.5,
    "Joint3": 0.0,
    "Joint4": 0.0,
    "Joint5": 0.0,
    "Joint6": 0.0,
    "Joint6_mimic": 0.0,
}


def values(text: str | None, count: int) -> list[float]:
    if not text:
        return [0.0] * count
    result = [float(item) for item in text.split()]
    if len(result) != count:
        raise ValueError(f"Expected {count} values, got {text!r}")
    return result


def origin_matrix(element: ET.Element | None) -> Matrix:
    if element is None:
        return Matrix.Identity(4)
    xyz = values(element.get("xyz"), 3)
    rpy = values(element.get("rpy"), 3)
    return Matrix.Translation(Vector(xyz)) @ Euler(rpy, "XYZ").to_matrix().to_4x4()


def joint_motion(joint: ET.Element, position: float) -> Matrix:
    joint_type = joint.get("type", "fixed")
    axis_element = joint.find("axis")
    axis = Vector(values(axis_element.get("xyz") if axis_element is not None else None, 3))
    if joint_type in ("revolute", "continuous"):
        return Matrix.Rotation(position, 4, axis)
    if joint_type == "prismatic":
        return Matrix.Translation(axis * position)
    return Matrix.Identity(4)


def link_transforms(
    robot: ET.Element,
    root_link: str,
    positions: dict[str, float],
) -> dict[str, Matrix]:
    children: dict[str, list[ET.Element]] = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent").get("link")
        children.setdefault(parent, []).append(joint)

    transforms = {root_link: Matrix.Identity(4)}
    pending = [root_link]
    while pending:
        parent = pending.pop()
        for joint in children.get(parent, []):
            child = joint.find("child").get("link")
            transforms[child] = (
                transforms[parent]
                @ origin_matrix(joint.find("origin"))
                @ joint_motion(joint, positions.get(joint.get("name"), 0.0))
            )
            pending.append(child)
    return transforms


def new_empty(name: str, collection: bpy.types.Collection) -> bpy.types.Object:
    empty = bpy.data.objects.new(name, None)
    empty.empty_display_type = "ARROWS"
    empty.empty_display_size = 0.08
    collection.objects.link(empty)
    return empty


def move_to_collection(obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    if collection not in obj.users_collection:
        collection.objects.link(obj)
    for old_collection in list(obj.users_collection):
        if old_collection != collection:
            old_collection.objects.unlink(obj)


def import_ply(
    path: Path,
    parent: bpy.types.Object,
    collection: bpy.types.Collection,
    prefix: str,
) -> None:
    before = set(bpy.data.objects)
    bpy.ops.wm.ply_import(filepath=str(path))
    imported = list(set(bpy.data.objects) - before)
    imported_set = set(imported)
    roots = [obj for obj in imported if obj.parent not in imported_set]
    for index, obj in enumerate(imported):
        obj.name = f"{prefix}_{index:02d}_{obj.name}"
        move_to_collection(obj, collection)
        obj.hide_select = True
    for obj in roots:
        obj.parent = parent


def add_robot(
    *,
    name: str,
    urdf_path: Path,
    root_link: str,
    positions: dict[str, float],
    assembly_root: bpy.types.Object,
    collection: bpy.types.Collection,
    package_name: str,
    converted_mesh_root: Path,
) -> None:
    robot = ET.parse(urdf_path).getroot()
    transforms = link_transforms(robot, root_link, positions)
    for link in robot.findall("link"):
        link_name = link.get("name")
        link_empty = new_empty(f"{name}_LINK_{link_name}", collection)
        link_empty.parent = assembly_root
        link_empty.matrix_local = transforms[link_name]
        link_empty.hide_select = True
        for visual_index, visual in enumerate(link.findall("visual")):
            mesh = visual.find("./geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename")
            expected_prefix = f"package://{package_name}/"
            if not filename.startswith(expected_prefix):
                raise ValueError(f"Unexpected mesh URI: {filename}")
            mesh_path = converted_mesh_root / f"{Path(filename).stem}.ply"
            visual_empty = new_empty(
                f"{name}_VISUAL_{link_name}_{visual_index}", collection
            )
            visual_empty.parent = link_empty
            visual_empty.matrix_local = origin_matrix(visual.find("origin"))
            visual_empty.hide_select = True
            import_ply(
                mesh_path,
                visual_empty,
                collection,
                f"{name}_{link_name}",
            )


def bake_rigid_assembly(
    *,
    name: str,
    root: bpy.types.Object,
    collection: bpy.types.Collection,
    color: tuple[float, float, float, float],
) -> None:
    """Bake all URDF visual meshes under ``root`` into one rigid child mesh."""
    mesh_objects = [obj for obj in collection.objects if obj.type == "MESH"]
    root_from_world = root.matrix_world.inverted()
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, ...]] = []
    for obj in mesh_objects:
        root_from_mesh = root_from_world @ obj.matrix_world
        offset = len(vertices)
        vertices.extend(tuple(root_from_mesh @ vertex.co) for vertex in obj.data.vertices)
        faces.extend(
            tuple(offset + index for index in polygon.vertices)
            for polygon in obj.data.polygons
        )

    baked_data = bpy.data.meshes.new(f"{name}_MESH_DATA")
    baked_data.from_pydata(vertices, [], faces)
    baked_data.update(calc_edges=True)
    baked = bpy.data.objects.new(f"{name}_MESH", baked_data)
    collection.objects.link(baked)
    baked.parent = root
    baked.matrix_local = Matrix.Identity(4)
    baked.hide_select = True

    material = bpy.data.materials.new(f"{name}_MATERIAL")
    material.diffuse_color = color
    baked.data.materials.append(material)

    for obj in list(collection.objects):
        if obj not in {root, baked}:
            bpy.data.objects.remove(obj, do_unlink=True)


def main() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "METERS"
    scene.unit_settings.scale_length = 1.0

    go2_collection = bpy.data.collections.new("GO2_OFFICIAL")
    d1_collection = bpy.data.collections.new("D1_ARM")
    scene.collection.children.link(go2_collection)
    scene.collection.children.link(d1_collection)

    go2_root = new_empty("GO2_BASE", go2_collection)
    go2_root["frame"] = "Official Go2 URDF base; +X forward, +Y left, +Z up"
    go2_root["source_commit"] = "daadf41ee9afce8f90fdc09a98506012691fa122"
    go2_root["pose"] = "Official Unitree stand_down_joint_pos"
    go2_root.lock_location = (True, True, True)
    go2_root.lock_rotation = (True, True, True)
    go2_root.lock_scale = (True, True, True)
    add_robot(
        name="GO2",
        urdf_path=GO2_ROOT / "urdf" / "go2_description.urdf",
        root_link="base",
        positions=GO2_JOINTS,
        assembly_root=go2_root,
        collection=go2_collection,
        package_name="go2_description",
        converted_mesh_root=HERE / "converted" / "go2",
    )
    bake_rigid_assembly(
        name="GO2",
        root=go2_root,
        collection=go2_collection,
        color=(0.16, 0.20, 0.24, 1.0),
    )

    d1_root = new_empty("D1_BASE_LINK", d1_collection)
    # Provisional placement above the torso only so the two assemblies are
    # visible on first open. The user must replace this transform manually.
    d1_root.location = (0.0, 0.0, 0.10)
    d1_root["frame"] = "D1 URDF base_link; edit only this object's transform"
    d1_root["pose_rad"] = "[0, -1.54, 1.55, 0, 0, 0]"
    d1_root["mount_transform_status"] = "PROVISIONAL - USER MUST ALIGN"
    add_robot(
        name="D1",
        urdf_path=D1_ROOT / "urdf" / "d1_description.urdf",
        root_link="base_link",
        positions=D1_JOINTS,
        assembly_root=d1_root,
        collection=d1_collection,
        package_name="d1_constrained_description",
        converted_mesh_root=HERE / "converted" / "d1",
    )
    bake_rigid_assembly(
        name="D1",
        root=d1_root,
        collection=d1_collection,
        color=(0.72, 0.72, 0.72, 1.0),
    )

    bpy.ops.object.select_all(action="DESELECT")
    d1_root.hide_select = False
    d1_root.select_set(True)
    bpy.context.view_layer.objects.active = d1_root
    scene["mounting_instructions"] = (
        "Move/rotate only D1_BASE_LINK. Do not apply transforms or edit child meshes."
    )
    bpy.ops.wm.save_as_mainfile(filepath=str(OUTPUT))
    print(f"Saved {OUTPUT}")


if __name__ == "__main__":
    main()
