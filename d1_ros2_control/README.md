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
- Simulation commands are range-checked and limited to 10 Hz. Physical motion
  bypasses the ros2_control write interface and is emitted only by the
  dedicated mode=1 Action controller.
- Joint6 completion uses a 2 degree target tolerance. Contact stop is detected
  only after at least 1 degree of commanded motion, when a 0.55 second rolling
  window spans no more than 0.3 degree; it does not depend on noisy
  adjacent-frame velocity estimates or the ROS joint-state publication rate.
- Completion is evaluated from joint feedback; D1 execution ACK is not used.
- If a mode=1 absolute target is written but fresh joint feedback shows
  no physical motion for 2.0 seconds while the endpoint remains outside its
  tolerance, the controller resends the same absolute target with a fresh
  sequence number every two seconds. It stops retrying as soon as motion starts
  and aborts after four retries. This applies to both arm and gripper goals.
- Simulation hardware deactivation sends a measured-position hold. Physical
  Action cancellation sends a measured-position hold through the unique
  command owner. Neither mechanism is an emergency stop or a replacement for
  a vendor-approved hardware stop.

On physical hardware, `d1_mode1_controller` is the sole motion-command owner.
It implements the standard arm and gripper Action names expected by MoveIt,
keeps one coherent seven-joint target, and emits complete D1
`funcode=2, mode=1` snapshots. Arm goals update Joint0--5 while preserving the
commanded gripper target; gripper goals update Joint6 while preserving the arm
target. The ros2_control hardware plugin remains responsible for preparation
and feedback publication but its physical command output is disabled.

Simulation continues to use the standard ros2_control trajectory and gripper
controllers. Canceling a physical arm goal holds measured Joint0--5 while
preserving the active gripper target; canceling a gripper goal holds its
measured opening.

The physical arm Action currently sends the final MoveIt trajectory endpoint
as one vendor-smoothed joint goal. It does not replay every planned waypoint;
real collision avoidance therefore remains a staged-validation boundary until
the D1 interface exposes trajectory timing or sparse waypoint execution is
added and validated.

For physical-command diagnosis, the controller logs the initial and requested
Joint0--5 angles, preserved Joint6 target, local UDP sequence, first observed
motion, final feedback, and timeout error. The command gateway logs every
native JSON payload and its local/native sequence pair. The status gateway logs
matching D1 receive and execution acknowledgements from `rt/arm_Feedback`.
`DDS Write accepted` only means the message entered the DDS writer; joint
feedback remains the authority for motion start and endpoint completion.

`d1_mode1_gripper_goal.py` is a low-level diagnostic helper, not a normal task
interface. It bypasses the Action controller, so stop and restart the real
control stack after using it before resuming task execution.

## Build

From the repository root:

```bash
source /opt/ros/humble/setup.zsh
colcon build --symlink-install \
  --base-paths d1_constrained_description_20260728 d1_ros2_control \
    d1_moveit_config d1_grasp_demo \
  --packages-select d1_constrained_description d1_ros2_control \
    d1_moveit_config d1_grasp_demo
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
