# Go2–D1 manual mounting calibration

This Blender scene is intended to recover the rigid transform from Unitree's
official Go2 `base` frame to the D1 `base_link` frame.

## Source and coordinate contract

- Go2 meshes and URDF come from Unitree's official `unitree_ros` repository at
  commit `daadf41ee9afce8f90fdc09a98506012691fa122`.
- The upstream BSD-3-Clause license is preserved under `vendor/`.
- D1 meshes and kinematics come from this workspace's calibrated URDF.
- Units are metres. Both roots use the ROS convention: +X forward, +Y left,
  +Z up.
- Go2 uses Unitree's official `stand_down_joint_pos` from the
  `unitree_mujoco/example/python/stand_go2.py` sim-to-real example. This only
  determines the displayed leg pose and does not affect the mounting
  transform between the two robot base frames.
- The D1 is displayed in the configured `STOWED` pose.

## Build the scene

```zsh
source ./scripts/setup_dev_env.zsh
python tools/calibration/go2_d1_mount/convert_meshes.py
blender --background --factory-startup \
  --python tools/calibration/go2_d1_mount/build_mount_scene.py
```

Open the scene:

```zsh
blender tools/calibration/go2_d1_mount/go2_d1_mount.blend
```

The two robots are baked into rigid visual
assemblies rooted at `GO2_BASE` and `D1_BASE_LINK`. Select and transform
**only** `D1_BASE_LINK` until the D1 base is correctly seated on the Go2 back.
The Go2 root and both child meshes are locked against accidental selection.
Use `G` to translate, `R` to rotate, or the `N` panel to enter values. Do not
use “Apply Transforms” or edit mesh vertices. Save with `Ctrl+S` when done.

Save the Blender file after alignment. Extract the relationship with:

```zsh
blender --background tools/calibration/go2_d1_mount/go2_d1_mount.blend \
  --python tools/calibration/go2_d1_mount/read_mount_transform.py
```

The output is `T_go2_base_d1_base`, satisfying:

```text
p_go2_base = T_go2_base_d1_base @ p_d1_base_link
```

## Locomotion handoff

The nominal real-robot manipulation posture, SDK motor ordering, ground/base
relationship, and fields that still require loaded-robot validation are kept
in [`go2_manipulation_posture.yaml`](go2_manipulation_posture.yaml). The
Chinese handoff note is [`LOCOMOTION_HANDOFF_CN.md`](LOCOMOTION_HANDOFF_CN.md).
