# Unitree D1 Autonomous Visual Picking and Dropping

[简体中文](README_CN.md) | English

This project uses a **Unitree D1-550 arm, an Intel RealSense D435i wrist camera, and ROS 2 Humble** to perform autonomous visual recognition, picking, carrying, and dropping. The client supplies only an approximate object location or a drop target. The arm module handles perception, motion planning, execution, and failure recovery, without requiring the client to specify grasp poses or joint trajectories.

The project can run independently on a development PC or on a Unitree Go2 onboard computer, providing arm manipulation capabilities to a higher-level application. Quadruped locomotion, navigation, and overall task orchestration are outside the scope of this project.

## Features and Implementation

- **Visual perception**: Uses wrist-mounted RealSense RGB-D images and IMU data, together with YOLO recognition, ground estimation, and object geometry, to compute grasp poses.
- **Autonomous picking**: Performs coarse observation, fine observation, pregrasp positioning, visual fine-tuning, descent, gripper closure, lifting, and carrying, then checks that the object is still held in the carry pose.
- **Motion planning and control**: Uses MoveIt 2 for inverse kinematics and collision checking, and executes physical arm movements through the project's in-tree `unitree-d1-control` onboard control service.
- **Autonomous dropping and failure handling**: Searches for release poses, checks outbound and return paths, releases the object, and stows the arm. A state machine handles recoverable task failures and returns information to guide the client's next decision.
- **Simulation and deployment**: Provides MuJoCo simulation validation, RViz visualization, and Docker deployment scripts for amd64 development PCs and arm64 Go2 onboard computers.

The project currently supports yellow cubes, elongated objects such as zucchini, and bowls. **It is not a general-purpose solution for grasping arbitrary objects.** See the [object guide](docs/api/objects/README.md) for specifications and images. Changes to the objects, arm, or camera mounting may require adjustments to the perception model and calibration parameters for the new setup.

## Deployment Tutorial

The [deployment tutorial](deploy/README.md) covers hardware connections, Docker environment setup, updates to the D1 onboard control service, parameter configuration, and functional validation in simulation and on hardware.

It provides two deployment options:

- **Development PC (amd64)**: For independent development, debugging, and acceptance testing of the arm service.
- **Unitree Go2 onboard computer (arm64)**: For integrating the arm service into the quadruped system. Validation on a development PC is recommended first.

Printable files and assembly instructions for the wrist-camera mount are available in the [camera mount guide](hardware/wrist_camera_mount/README.md).

## API Reference

The [API reference](docs/api/README.md) is intended for clients of the arm module. It explains how to call the interfaces, their request parameters, execution feedback, final results, and state transitions, with command-line, Python, and C++ examples.

| Interface | Kind | Purpose |
| --- | --- | --- |
| `/arm/tasks/pick_object` | ROS 2 Action | Autonomously pick an object using its approximate location and move to the carry pose |
| `/arm/tasks/drop_object` | ROS 2 Action | Release an object at the specified drop target and return to the stowed pose |
| `/arm/task_status` | ROS 2 Topic | Report the arm's current task state, payload state, and fault information |

Clients use `outcome` in the Action Result to decide whether to proceed, reposition, specify a new drop target, or arrange for manual troubleshooting. The status Topic is used to monitor the arm's state.
