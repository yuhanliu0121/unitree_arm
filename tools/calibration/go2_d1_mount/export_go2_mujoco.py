"""Export the baked lie-down Go2 visual from Blender for MuJoCo."""

from pathlib import Path

import bpy


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
OUTPUT = (
    WORKSPACE
    / "d1_mujoco_sim"
    / "src"
    / "d1_mujoco_sim"
    / "assets"
    / "go2"
    / "go2_lie_down.obj"
)


def main() -> None:
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    mesh = bpy.data.objects["GO2_MESH"]
    mesh.hide_select = False
    mesh.select_set(True)
    bpy.context.view_layer.objects.active = mesh
    bpy.ops.wm.obj_export(
        filepath=str(OUTPUT),
        export_selected_objects=True,
        export_materials=False,
        apply_modifiers=True,
        # Preserve the Blender/ROS coordinates in the OBJ vertex data:
        # +X forward, +Y left, +Z up.
        forward_axis="Y",
        up_axis="Z",
    )
    print(f"Exported {OUTPUT}")


if __name__ == "__main__":
    main()
