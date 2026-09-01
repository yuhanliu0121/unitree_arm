# D1 real-machine bringup

The repository-root entry point is:

```zsh
./real_bringup.zsh
./real_bringup.zsh --rviz
```

It reads `config/real_machine.yaml`, runs the motionless preflight, and only
then starts the physical D1 gateway, `ros2_control`, MoveIt, the wrist
RealSense, perception, and the public pick/drop Action servers. The preflight
gravity estimate is frozen for the task run so wrist acceleration during arm
motion cannot rotate the gravity reference.

When the wrist RealSense USB cable is attached to another computer, start its
ROS driver there on the configured ROS domain, then run:

```zsh
./real_bringup.zsh --camera-driver-location remote
./real_bringup.zsh --camera-driver-location remote --rviz
```

Remote mode never opens a local USB camera. It still requires fresh color and
aligned-depth CameraInfo, IMU samples, and the configured camera TF chain before
the physical controllers are activated. The remote publisher must therefore
use `ROS_DOMAIN_ID` from `real_machine.yaml` and publish the canonical
`/wrist_camera/*` topics and frames. Set `wrist_camera.driver_location` in a
site deployment YAML to make either mode persistent; the command-line option is
intended for acceptance tests and overrides YAML.

`wrist_camera.frame_rate_hz` is shared by the RGB and depth streams and accepts
only the D435i-native values `6`, `15`, or `30`. The launch files combine it
with `color_resolution` and `depth_resolution` to build the RealSense profiles.
The default is 6 Hz so two uncompressed RGB-D cameras can fit on a 1 GbE link.
Local mode applies this setting automatically. A remote camera publisher must
apply the same value when it starts `realsense2_camera`; for the default config,
use `rgb_camera.color_profile:=1280x720x6` and
`depth_module.depth_profile:=848x480x6`.

The motionless preflight accepts either disabled or already-enabled hardware
when feedback is fresh and `error_status=0`. It never sends a power or enable
command. The following ros2_control activation performs the idempotent D1
power/enable requests and requires a fresh `enable_status=1` before exposing
active controllers.

The command does not initiate a pose change or a pick/drop task. The caller
must still send `/arm/tasks/pick_object` or `/arm/tasks/drop_object` goals.

Use `--config PATH` for a site-specific deployment file and
`--arm-serial SERIAL` only when intentionally overriding the serial recorded
in that file.
`real_bringup.zsh` runs the motionless preflight before starting physical
control, MoveIt, perception, and the task Actions. A read-only readiness monitor
then waits for fresh D1 joint feedback, RGB and aligned-depth CameraInfo, MoveIt,
the arm/gripper command Actions, and `pick_object`/`drop_object`. Once every
dependency is live it prints a prominent green `D1 REAL CONTROL STACK READY`
banner. If readiness takes longer than 60 seconds it prints the missing items in
red and continues waiting; no task is sent automatically.
# Streaming STOWED recovery

With `real_bringup.zsh` already running, recover the arm without switching back
to the vendor controller:

```bash
./arm_stowed_control.zsh --confirm STOWED_MOVE
```

The command first sends Joint0--5 to the canonical STOWED pose while preserving
the current gripper target, then fully opens Joint6. It refuses to run when the
control Action servers are unavailable. Ctrl+C cancels the active Action and
asks the controller to hold the measured position.
