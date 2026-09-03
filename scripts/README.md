# Supported entry points

- `setup_dev_env.zsh`: source the ROS, Conda, and workspace environment.
- `real_preflight.zsh`: motionless validation of deployment configuration,
  feedback, camera, IMU, and TF prerequisites.
- `real_bringup.zsh`: start the production real-machine stack after preflight.
- `arm_stowed_control.zsh`: recover to STOWED through the active ROS controller,
  or automatically fall back to the direct SDK utility when no stack is running.
- `arm_zero_control.zsh`: stop a local production stack when necessary, then
  command the physical arm to the zero pose through the direct SDK utility.
- `all_joints_unload.zsh`: stop a local production stack when necessary, then
  unload all physical joints through the direct SDK utility.
- `arm_maintenance.zsh`: shared implementation behind the three maintenance
  entry points; normally call the operation-specific wrappers above.

Calibration and acceptance scripts intentionally live under `tools/` and are
not production APIs.

`real_bringup.zsh` first performs a one-shot SSH check of the configured D1
board. It requires the configured systemd service to be active and the
installed executable plus its deployment release manifest to exist. The
manifest must match both the expected software version and the host protocol
version. The checker never launches the board executable. Only then does the existing
motionless sensor/feedback preflight run. Password-based board access uses the
Ubuntu `sshpass` package; key-based access can be selected by leaving
`arm.onboard_control.ssh_password` empty.

## Portable environment configuration

All entry points derive the workspace root from their own location, so the
repository may be moved or embedded as `Go2_Codebase/manipulator` without
editing scripts. `setup_dev_env.zsh` uses ROS 2 Humble and the
`trash_collection` Conda environment by default, while allowing deployment
overrides:

| Variable | Purpose | Default |
| --- | --- | --- |
| `D1_ROS_SETUP` | ROS 2 shell setup | `/opt/ros/humble/setup.zsh` |
| `D1_CONDA_ENV` | Conda environment name | `trash_collection` |
| `D1_CONDA_SETUP` | Explicit `conda.sh` location | auto-discovered |
| `D1_CONDA_EXE` | Conda executable used by validation tools | auto-discovered |
| `D1_SKIP_CONDA=1` | Use the caller/container Python environment as-is | disabled |

Auto-discovery checks the active Conda installation, `PATH`, the current
user's standard Miniconda/Anaconda locations, and `/opt/conda`. An explicit
override is authoritative: an invalid configured path fails startup instead
of silently falling back to a different environment.
