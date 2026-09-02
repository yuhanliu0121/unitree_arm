"""Print T_go2_base_d1_base from the manually aligned Blender scene."""

from __future__ import annotations

import bpy


go2 = bpy.data.objects["GO2_BASE"]
d1 = bpy.data.objects["D1_BASE_LINK"]
transform = go2.matrix_world.inverted_safe() @ d1.matrix_world
translation = transform.to_translation()
quaternion = transform.to_quaternion()

print("T_go2_base_d1_base =")
for row in transform:
    print("  [" + ", ".join(f"{value:.12f}" for value in row) + "]")
print(
    "translation_xyz_m = ["
    + ", ".join(f"{value:.12f}" for value in translation)
    + "]"
)
print(
    "quaternion_xyzw = ["
    + ", ".join(
        f"{value:.12f}" for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )
    + "]"
)
