# Unitree D1 arm stack

ROS 2 Humble and MuJoCo development workspace for the Unitree D1 arm. The
current stack provides a calibrated robot description, a D1 protocol-compatible
MuJoCo simulator, `ros2_control`, MoveIt configuration, RGB-D simulation for a
wrist-mounted RealSense D435i, and a fixed-cube grasp acceptance flow.

## Packages

- `d1_constrained_description_20260728`: calibrated URDF, meshes and display.
- `d1_mujoco_sim`: physics scene, D1 DDS protocol and wrist RGB-D publishers.
- `d1_ros2_control`: D1 hardware interface and trajectory controllers.
- `d1_moveit_config`: MoveIt planning and RViz configuration.
- `d1_grasp_demo`: grasp baseline and application-level fixed poses.
- `d1_manipulation`: gravity-aligned eye-in-hand observation Action and
  ordered MoveIt candidate search.

## Acceptance

Activate the `trash_collection` Conda environment and run:

```zsh
./accept_cube_grasp.zsh
```

Optional visualization modes:

```zsh
./accept_cube_grasp.zsh --rviz
./accept_cube_grasp.zsh --headless --rviz
```

Run the automatic seed-0 target-observation acceptance:

```zsh
./accept_observe_target.zsh --headless
./accept_observe_target.zsh --rviz
```

Keep the stack running and send `/arm/debug/observe_target` goals manually:

```zsh
./accept_observe_target.zsh --rviz --server-only
```

For direct MuJoCo pose inspection:

```zsh
d1-mujoco-sim --manual-control \
  --initial-arm-pose '[0, -1.5, 1.5, 0, -0.6, 0]'
```

See `doc/codex_handoff.md` for the detailed architecture, calibration values,
validation history and current limitations.
