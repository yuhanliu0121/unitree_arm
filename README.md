# Unitree D1 arm stack

ROS 2 Humble and MuJoCo stack for wrist-RGB-D pick, carry, and drop tasks on a
Unitree D1 arm. The only Go2-facing task API is:

- `/arm/tasks/pick_object`
- `/arm/tasks/drop_object`
- `/arm/task_status`

The action and status types live in `d1_interfaces`.
The external contract is documented in
[`docs/arm_manipulation_api_cn.md`](docs/arm_manipulation_api_cn.md).

## Repository layout

| Path | Purpose |
| --- | --- |
| `src/` | Production ROS 2 packages: interfaces, bringup, description, control, MoveIt, perception integration, and task logic |
| `simulation/` | MuJoCo simulator and simulation-only object assets |
| `runtime/` | Versioned perception runtime and model assets |
| `scripts/` | Supported environment, real-machine bringup, preflight, and recovery entry points |
| `tools/` | Calibration, validation, workspace analysis, and asset-generation utilities |
| `docs/` | Architecture, calibration, experiment, and handoff documentation |
| `third_party/` | Official D1 SDK files and the in-tree `unitree-d1-control` component |
| `deploy/` | Native amd64/arm64 Docker environment and D1 onboard deployment scripts |

Generated `build/`, `install/`, and `log/` trees remain at the workspace root
and are ignored by Git.

## Development environment

From a new zsh terminal:

```zsh
source ./scripts/setup_dev_env.zsh
```

The script loads ROS 2, the `trash_collection` Conda environment, and the
workspace overlay. It deliberately leaves `ROS_DOMAIN_ID` unset; launchers
select the simulation or real-machine domain explicitly.

## Build

Production build:

```zsh
colcon build --symlink-install
```

Commissioning clients are excluded by default. Enable them only for staged
development and acceptance:

```zsh
colcon build --symlink-install \
  --cmake-args -DD1_BUILD_COMMISSIONING_TOOLS=ON
```

## Validation

Examples:

```zsh
./tools/validation/scripts/accept_observe_target.zsh --headless
./tools/validation/scripts/accept_visual_cube_grasp.zsh --rviz
./tools/validation/scripts/accept_cube_pick_drop.zsh --headless
```

Calibration-only entry points are under `tools/calibration/scripts/`.

## Real-machine bringup

Run `./deploy/configure_d1_manipulation.sh`, complete and validate the generated
`src/d1_bringup/config/real_machine.local.yaml`, select the perception weights,
and fill the Go2-owned network/TF values. Then run the motionless
preflight before starting the control stack:

```zsh
./deploy/configure_d1_manipulation.sh --validate
./scripts/real_preflight.zsh
./scripts/real_bringup.zsh --rviz
```

See `src/d1_bringup/config/README.md` for configuration ownership and
`docs/codex_handoff.md` for the current architecture and known limitations.
For a clean target machine, follow the three-step procedure in
`deploy/README.md`.
