"""Export the D435i and its mount from the mounting Blender file.

Run with Blender so its Python API is available::

    blender -b D1-550.blend -P export_d435i_mesh.py -- camera.obj mount.obj

The camera vertices are expressed in the RGB optical frame, while the mount
vertices are expressed directly in Link6.  Keeping those relationships
independent makes a camera/mount calibration mismatch visible in MuJoCo.
"""

from __future__ import annotations

import sys
from pathlib import Path

import bpy


SOURCE_OBJECT = "D435i_Official_CAD"
RGB_FRAME_OBJECT = "FRAME_RGB_OPTICAL_FACTORY"
MOUNT_OBJECT = "main_stand"
LINK6_FRAME_OBJECT = "LYH_D1_FRAME_Link6"
DECIMATE_RATIO = 0.15


def _script_args() -> list[str]:
    try:
        separator = sys.argv.index("--")
    except ValueError as exc:
        raise SystemExit("Expected '-- OUTPUT.obj'") from exc
    return sys.argv[separator + 1 :]


def _export_mesh(
    source_name: str,
    reference_name: str,
    output_path: Path,
    decimate_ratio: float,
) -> None:
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    source = bpy.data.objects.get(source_name)
    reference = bpy.data.objects.get(reference_name)
    if source is None or source.type != "MESH":
        raise RuntimeError(f"Mesh object not found: {source_name}")
    if reference is None:
        raise RuntimeError(f"Reference frame not found: {reference_name}")

    exported = source.copy()
    exported.data = source.data.copy()
    bpy.context.scene.collection.objects.link(exported)
    exported.hide_viewport = False
    exported.hide_render = False
    bpy.context.view_layer.objects.active = exported
    exported.select_set(True)

    if decimate_ratio < 1.0:
        modifier = exported.modifiers.new(
            name="MuJoCoDecimate",
            type="DECIMATE",
        )
        modifier.ratio = decimate_ratio
        bpy.ops.object.modifier_apply(modifier=modifier.name)

    reference_from_mesh = (
        reference.matrix_world.inverted_safe() @ exported.matrix_world
    )
    exported.data.transform(reference_from_mesh)
    exported.data.calc_loop_triangles()

    vertices = exported.data.vertices
    triangles = exported.data.loop_triangles
    with output_path.open("w", encoding="ascii", newline="\n") as output:
        output.write(f"# Blender source object: {source_name}\n")
        output.write(f"# coordinate frame: {reference_name}\n")
        output.write(f"# decimate ratio: {decimate_ratio}\n")
        output.write(f"o {source_name}\n")
        for vertex in vertices:
            x, y, z = vertex.co
            output.write(f"v {x:.9g} {y:.9g} {z:.9g}\n")
        for triangle in triangles:
            a, b, c = (index + 1 for index in triangle.vertices)
            output.write(f"f {a} {b} {c}\n")

    coordinates = [vertex.co for vertex in vertices]
    mins = [min(value[axis] for value in coordinates) for axis in range(3)]
    maxs = [max(value[axis] for value in coordinates) for axis in range(3)]
    print(
        "Exported",
        output_path,
        f"source={source_name} frame={reference_name}",
        f"vertices={len(vertices)} triangles={len(triangles)}",
        f"bounds_min={mins} bounds_max={maxs}",
    )


def main() -> None:
    args = _script_args()
    if len(args) != 2:
        raise SystemExit("Usage: ... -- CAMERA.obj MOUNT.obj")
    _export_mesh(
        SOURCE_OBJECT,
        RGB_FRAME_OBJECT,
        Path(args[0]),
        DECIMATE_RATIO,
    )
    _export_mesh(
        MOUNT_OBJECT,
        LINK6_FRAME_OBJECT,
        Path(args[1]),
        1.0,
    )


if __name__ == "__main__":
    main()
