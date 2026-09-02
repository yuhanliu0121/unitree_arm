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
