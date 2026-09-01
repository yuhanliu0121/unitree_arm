# D1 ros2_control

Position-command `ros2_control` hardware interface for the Unitree D1 native
DDS service. It connects to `d1_mujoco_sim` on DDS domain 42 and, only after
explicit safety validation, physical hardware on domain 0.

The Unitree SDK is deliberately isolated from `ros2_control_node` because ROS
2 Humble and the installed SDK provide ABI-incompatible Cyclone DDS libraries.
The launch file starts both parts automatically:

```text
ROS 2 controllers -> hardware plugin -> UDP loopback -> command gateway -> Unitree DDS
Unitree DDS -> feedback gateway -> UDP loopback -> hardware plugin
Unitree DDS -> status gateway -> UDP loopback -> hardware plugin
Unitree DDS <-> MuJoCo or D1
```

Command publication, joint feedback, and hardware status subscription run in
separate gateway processes. This isolates the physical D1 SDK runtime: writing
commands from the same process that owns a feedback subscriber can stop its
callback, while the arm continues publishing to independent subscribers.

The loopback ports default to `15000` for commands and `15001` for feedback.
Starting a second control stack with the same ports fails instead of allowing
two controllers to command one arm.

## Safety boundary

- The default domain is 42 and does not target a factory-default physical D1.
- Activating the hardware waits for live feedback and initializes every command
  from the measured position.
- Real-hardware activation requires live status with `error_status=0`, powers
  the D1 when necessary, requests the validated `mode=65535` full enable, and
  verifies a fresh powered/enabled status before controllers become active.
- Simulation uses the standard `joint_trajectory_controller` and
  `GripperActionController`. Physical motion uses
  `d1_joint_segment_controller`, exposed under the same Action names.
- Each physical task stage sends one coherent Joint0--6 target, one common
  arrival time and one acceleration/deceleration profile. The onboard executor
  translates it to FashionStar native interval commands and compensates UART
  dispatch skew.
- The real ros2_control hardware plugin is feedback-only. It publishes
  `/joint_states` but cannot accidentally emit a competing sampled command
  stream.
- Joint feedback remains authoritative for motion and completion. The 100 Hz
  ros2_control loop may read the most recent sample repeatedly, while freshness
  and velocity calculations use the native feedback arrival time.
- Physical deployment is paired with `unitree-d1-control`'s thin
  onboard executor, which gives one I/O thread exclusive ownership of command,
  damping and feedback serial traffic.
- Simulation hardware deactivation emits one short measured-position target.
  Real feedback-only hardware emits no command on deactivation; neither path
  is an emergency stop or a replacement for a vendor-approved hardware stop.

The native controller intentionally executes each accepted task-stage endpoint
as one joint-space segment; it does not reproduce every intermediate waypoint
of the MoveIt trajectory. Collision checking therefore applies to the planned
stage, while the physical segment still requires guarded real-machine
validation. Its native segment duration is derived from the largest joint
displacement at the configured D1 speed (15 deg/s by default).
`common_arrival` clamps it to 1.5--30 seconds; `uniform_joint_speed` preserves
short durations down to the protocol's 1 ms resolution. MoveIt's full waypoint
trajectory duration is logged for diagnostics but is not reused for this
single native segment.

The standard `FollowJointTrajectory` compatibility endpoint always uses
`uniform_joint_speed`; `native_joint_speed_deg_s` configures its default speed
(15 deg/s). Higher values require explicit real-machine validation.

Task code that needs explicit timing sends
`d1_ros2_control/action/ExecuteJointSegment` to
`/arm_controller/execute_joint_segment`. Every goal carries its own speed and
one of these profiles:

- `common_arrival` gives all moving joints one arrival time;
- `uniform_joint_speed` scales each joint duration by its displacement, so all
  moving joints use the same average angular speed and may arrive separately.

There is no process-global motion-profile state to switch or restore. The
gripper always uses its own uniform-speed segment. Simulation does not use the
native-segment controller.

`d1_gripper_goal.py` is a low-level diagnostic helper, not a normal task
interface. It bypasses the Action controller, so stop and restart the real
control stack after using it before resuming task execution.

## Build

From the repository root:

```bash
source /opt/ros/humble/setup.zsh
colcon build --symlink-install \
  --base-paths d1_constrained_description_20260728 d1_ros2_control \
    d1_moveit_config d1_manipulation \
  --packages-select d1_constrained_description d1_ros2_control \
    d1_moveit_config d1_manipulation
source install/setup.zsh
```

## Simulation

Terminal 1:

```bash
source /home/tony/miniconda3/etc/profile.d/conda.sh
conda activate trash_collection
cd /home/tony/Project/unitree_arm/dev
d1-mujoco-sim
```

Terminal 2:

```bash
cd /home/tony/Project/unitree_arm/dev
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch d1_ros2_control control.launch.py backend:=simulation
```

Inspect the loaded hardware and controllers:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic hz /joint_states
```

All three controllers should report `active`; every position command interface
should report `available` and `claimed`.

Send a small six-axis trajectory:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [Joint0, Joint1, Joint2, Joint3, Joint4, Joint5], points: [{positions: [0.17, -0.17, 0.26, 0.09, -0.09, 0.14], time_from_start: {sec: 3}}]}}"
```

Open the gripper to 50 mm total aperture (`Joint6=0.025 m` per the existing
description convention):

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.025, max_effort: 0.0}}"
```

Do not change `dds_domain_id` to 0 or provide a physical-arm interface until
the real-hardware activation procedure is reviewed at the robot.

## Known non-fatal warnings

- FIFO real-time scheduling is unavailable unless the user is granted the
  corresponding Linux limits. Functional simulation does not require it.
- KDL warns that `base_link` has inertia because the validated vendor URDF uses
  an inertial root link.
- Humble warns that directly passing `robot_description` to controller manager
  is deprecated; it remains supported in the installed Humble release.
