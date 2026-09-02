# ROS 2 Quick Start

The following commands create a fresh workspace without modifying an existing
ROS 2 workspace. The launch starts RViz, `robot_state_publisher`, and the joint
state GUI only; it does not connect to or command the physical arm.

## Zsh

```zsh
mkdir -p ~/d1_description_ws/src
cd ~/d1_description_ws/src
unzip /path/to/d1_constrained_description_20260728.zip

cd ~/d1_description_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select d1_constrained_description
source install/setup.zsh
ros2 launch d1_constrained_description display.launch.py
```

## Bash

Use the same commands, replacing:

```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Move `Joint0` through `Joint5` in the GUI and verify that each mesh remains
connected throughout its allowed range. `Joint6` controls the gripper opening.

To verify package discovery after building:

```bash
ros2 pkg prefix d1_constrained_description
```

## Live Read-Only D1 Visualization

Connect the D1 Ethernet interface and launch:

```bash
ros2 launch d1_constrained_description live_display.launch.py interface:=eno1
```

This replaces the GUI publisher with the included feedback-only bridge. It
subscribes to the Unitree `current_servo_angle` DDS topic, converts arm degrees
to radians and gripper motor angle to 0–0.03 m, and publishes `/joint_states`.
It does not publish a D1 command topic.

In another sourced terminal, verify reception:

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link tcp_link
```

The `tcp_link` origin is the nominal midpoint of the two finger-tip faces.
