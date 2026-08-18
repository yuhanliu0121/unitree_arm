# D1 ros2_control

Position-command `ros2_control` hardware interface for the Unitree D1 native
DDS service. It connects to `d1_mujoco_sim` on DDS domain 42 and, only after
explicit safety validation, physical hardware on domain 0.

The Unitree SDK is deliberately isolated from `ros2_control_node` because ROS
2 Humble and the installed SDK provide ABI-incompatible Cyclone DDS libraries.
The launch file starts both parts automatically:

```text
ROS 2 controllers -> hardware plugin -> UDP loopback -> d1_dds_gateway
                                                      -> Unitree DDS
                                                      -> MuJoCo or D1
```

The loopback ports default to `15000` for commands and `15001` for feedback.
Starting a second control stack with the same ports fails instead of allowing
two controllers to command one arm.

## Safety boundary

- The default domain is 42 and does not target a factory-default physical D1.
- Activating the hardware waits for live feedback and initializes every command
  from the measured position.
- Commands are range-checked and limited to 10 Hz.
- `Joint6/velocity` is estimated from adjacent feedback frames for the standard
  gripper action controller.
- Completion is evaluated from joint feedback by the standard trajectory
  controller; D1 execution ACK is not used.
- Software deactivation sends a measured-position hold. It is not an emergency
  stop and does not replace a vendor-approved hardware stop.

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
