# Unitree D1 Autonomous Vision-Guided Pick-and-Place: Deployment Guide
[中文](README_CN.md) | English

## Hardware and Software Requirements
| Device | Purpose | Required |
| --- | --- | --- |
| [Unitree D1-550](https://www.unitree.com/D1-T) | Performs pick-and-place operations | Yes |
| [Intel RealSense D435i](https://realsenseai.com/products/depth-camera-d435i/) | Provides wrist-mounted RGB-D sensing and gravity measurements | Yes |
| [Camera mount](../hardware/wrist_camera_mount/README.md) | Maintains a fixed geometric relationship between the camera and the arm | Yes |
| Ubuntu 22.04 LTS / 20.04 LTS | Used for development, simulation, and standalone hardware testing | Yes |
| [Unitree Go2 EDU](https://www.unitree.com/go2) | Serves as an autonomous mobile platform that carries the arm for navigation and pick-and-place operations | No. The project can run independently of Go2 or be integrated with it. |




## Deployment Overview
+ Choose **Deployment Option 1** to develop, debug, and independently validate the manipulation service.
+ To integrate the project with a Unitree Go2, first complete Deployment Option 1 and verify that the arm and RealSense camera operate as expected. Then proceed to Deployment Option 2.

## Deployment Option 1: Development Workstation
+ This option is intended for developing, debugging, and independently validating the manipulation service.
+ Use this option when modifying code, validating the pick-and-place workflow, or troubleshooting hardware.
+ If you intend to integrate the project with a Unitree Go2, complete this option before proceeding to Deployment Option 2.
+ In this setup, the arm and wrist-mounted RealSense camera connect directly to the development workstation, which runs the complete perception, planning, and control stack. This setup provides good compatibility with development tools and makes it easier to diagnose issues using RViz, logs, and other visualization tools.
+ The workstation must use the `x86_64` architecture (`uname -m` must return `x86_64`) and run Ubuntu 22.04 LTS or 20.04 LTS.
+ The Docker deployment requires at least 20 GB of free disk space.

### 1.1 Install the Required Tools
```bash
sudo apt update
sudo apt install -y git git-lfs python3 python3-yaml openssh-client sshpass tar usbutils
git lfs install
```

### 1.2 Clone the Repository and Download Large Files
```bash
mkdir -p ~/Project && cd ~/Project
git clone https://github.com/yuhanliu0121/unitree_arm.git
cd unitree_arm
git lfs pull
```

### 1.3 Build the Docker Environment
This project uses Docker to package ROS 2, MoveIt, the RealSense driver, the perception stack, and other runtime dependencies, minimizing differences between development environments. A script is provided to build the Docker environment automatically.

1. Run the following commands:

```bash
xhost +
cd ~/Project/unitree_arm
./deploy/install_d1_runtime_env.sh
```

If the build succeeds, the final output should look similar to the following. Test counts and runtimes may vary by code version and hardware. The final summaries must report no failures, and the message `D1 runtime environment is ready` must appear:

```bash
============================== 27 passed in 0.08s ==============================
Finished <<< d1_bringup [0.57s]

Summary: 8 packages finished [4.94s]
  1 package had stderr output: d1_camera_visualization
Summary: 87 tests, 0 errors, 0 failures, 0 skipped

D1 runtime environment is ready.
  architecture: amd64
  image:        unitree-d1-manipulation:amd64
  workspace:    /home/<your_user_name>/Project/unitree_arm
  dev container: unitree-d1-manipulation-dev-amd64 (running)

Enter the development environment with:
  docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
```

2. Verify the build:

```bash
test -f install/setup.bash && echo "ROS workspace: OK"
sudo docker image inspect unitree-d1-manipulation:amd64 >/dev/null && echo "Docker image: OK"
sudo docker ps --format '{{.Names}}' | grep '^unitree-d1-manipulation-dev-amd64$'
```

Expected output:

```text
ROS workspace: OK
Docker image: OK
unitree-d1-manipulation-dev-amd64
```

If the build or any test fails, inspect the first error reported in the terminal and the contents of `log/`. Do not proceed to simulation or hardware testing.

3. Once the build is complete, enter the development container:

```bash
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
```

Unless a step is explicitly marked as a host operation, run all subsequent commands in Deployment Option 1 inside this container. `/workspace` is the mount point for the host's `unitree_arm/` directory, so changes to the source code and the `build/`, `install/`, and `log/` directories persist on the host.

### 1.4 Validate the System in MuJoCo
First, validate the complete perception, motion-planning, picking, carrying, and dropping workflow in the fixed MuJoCo scene. This step uses only the simulated controller. It does not send commands to a physical D1 and does not require a D1 or RealSense camera to be connected.

1. Run the interactive MuJoCo and RViz acceptance test inside the container:

```bash
cd /workspace
./tools/validation/scripts/accept_cube_pick_drop.zsh --rviz
```

The script automatically starts MuJoCo, RViz, the arm control stack, and the perception nodes.

<table>
  <thead>
    <tr>
      <th>MuJoCo Window</th>
      <th>RViz Window</th>
      <th>Terminal</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><img src="images/mujoco_window.png" alt="MuJoCo simulation window" width="360"></td>
      <td><img src="images/rviz_simulation.png" alt="RViz simulation window" width="360"></td>
      <td><img src="images/simulation_terminal_ready.png" alt="Simulation acceptance terminal" width="360"></td>
    </tr>
  </tbody>
</table>


When the system is ready, the terminal displays `Press Enter to pick yellow_cube and move to CARRY...`. Press Enter to begin the simulated task.

If the test succeeds, the terminal displays `ACCEPTANCE PASSED: yellow_cube picked, carried, released into bin, and arm stowed.`

![MuJoCo simulation acceptance passed](images/simulation_acceptance_passed.png)

If this message does not appear, use the terminal error and the simulation logs under `log/` to diagnose the problem. Do not proceed to hardware deployment.



NOTE: Running MuJoCo, RViz, and the control stack together can consume substantial system resources. On a lower-performance workstation, this may cause the desktop to lag or the simulation to fail. In that case, run the acceptance test in headless mode:

```bash
./tools/validation/scripts/accept_cube_pick_drop.zsh --headless
```

### 1.5 Connect the Hardware
1. Power the D1 using its DC power supply, then connect it to the development workstation with an Ethernet cable.
2. Connect the RealSense D435i to the workstation using a USB 3.0 cable.
3. Run `lsusb | grep -i realsense` inside the container. The output should resemble the following:

![RealSense detected inside the container](images/realsense_lsusb.png)

### 1.6 Configure the Workstation Network
1. Open **Wired Settings** from the network/Wi-Fi menu in the upper-right corner of the Ubuntu desktop.
2. Click the gear icon next to the wired connection, then select the **IPv4** tab.
3. Set **Method** to **Manual**.
4. Set **Address** to `192.168.123.162`.
5. Set **Netmask** to `255.255.255.0`.

![Ubuntu wired IPv4 configuration](images/ubuntu_wired_ipv4.png)

6. Click **Apply**, then disable and re-enable the wired connection for the changes to take effect.

### 1.7 Verify Network Connectivity to the D1
Run `ping -c 3 192.168.123.100` inside the container. Replies similar to those shown below confirm that the connection is working.

![D1 network connectivity check](images/d1_ping.png)

### 1.8 Update the D1 Onboard Control Service
The D1-550's internal controller runs an onboard control service that receives joint commands from the development workstation or Go2, drives the motors, and reports execution status. The vendor-provided service offers limited multi-joint control and feedback, making it difficult for ROS 2/MoveIt pick-and-place tasks to determine reliably whether a command was executed, whether the joints reached their targets, or where a failure occurred.

This project therefore extends the onboard service while retaining the vendor protocol. The enhanced service provides more robust multi-joint command handling, execution-result feedback, diagnostics, and version queries, and is designed to work with the project's host-side control node.

When deploying to a D1 for the first time, replace the vendor-provided onboard service with the version included in this project. It does not need to be reinstalled during routine operation.

From the project root, run:

```bash
cd /workspace
./deploy/update_d1_onboard_sdk.sh --confirm UPDATE_D1_ONBOARD_SDK
```

The script uploads the control-service source code to the D1, builds it and runs the protocol test on the onboard computer, backs up the existing service, and then installs and starts the new service. The script does not intentionally send any joint-motion commands. However, the arm may temporarily lose holding torque while the services are being switched. Support the arm and clear its surrounding workspace before proceeding. A successful update produces output that includes:

```bash
100% tests passed, 0 tests failed out of 1
#blablabla......
D1 onboard SDK update passed. The d1-control.service is enabled and active.
```

![D1 onboard service update succeeded](images/onboard_update.png)

You can then inspect the service over SSH:

```bash
# try 123 if password is required
ssh ubuntu@192.168.123.100 'systemctl is-active d1-control.service && cat /home/ubuntu/marm_code/build/d1-control-release.env'
```

Expected output:

```bash
active
D1_CONTROL_VERSION=0.1.0
D1_CONTROL_PROTOCOL=2
```

![D1 onboard service state and protocol version](images/onboard_service_status.png)

This output confirms that the update succeeded.

### 1.9 Configure the Runtime Parameters
The manipulation service requires several parameters, including the D1 network interface, RealSense serial number, perception model, and device extrinsics. To simplify deployment, the configuration script detects the current platform, the network interface connected to the D1, and the wrist-mounted RealSense camera, then supplies validated defaults.

1. Connect the D1 and RealSense D435i to the workstation. Confirm that `lsusb | grep -i realsense` detects the camera and that `ping -c 3 192.168.123.100` receives replies.
2. Run the configuration script:

```bash
cd /workspace
./deploy/configure_d1_manipulation.sh
```

The script detects the available network and hardware connections and generates `src/d1_bringup/config/real_machine.local.yaml`. The parameters are described below:

| Parameter | Purpose | Manual configuration required | When to change it |
| --- | --- | --- | --- |
| `deployment.platform` | Identifies the current deployment platform and prevents a configuration from being loaded on the wrong architecture | No; detected automatically | Regenerate the configuration only if automatic detection fails after the platform has been confirmed |
| `deployment.ros_domain_id` | Domain ID used for ROS 2 node discovery and communication | No; the script uses the default value `31` | Change it when communicating with other ROS 2 nodes that use a different Domain ID; valid range: `0–232` |
| `perception.model_path` | Path to the YOLO object-detection model | Usually yes; defaults to `runtime/perception_runtime_v1/weights/mcislab_trash_collect.pt` | The default model is intended only for the objects and environments validated by this project. For other deployments, use a model trained and validated with data from the actual objects and operating environment |
| `arm.serial_no` | Selects arm-specific URDF corrections, currently used mainly for joint-limit overrides | Usually not; left blank by default | Set this only if the arm requires unit-specific joint-limit or other URDF corrections and a corresponding entry exists in `hardware_profiles.yaml` |
| `arm.network_interface` | Wired network interface used by the host to exchange native control and feedback data with the D1 | Usually not; enter manually if no interface or multiple candidates are detected | If automatic detection fails or is ambiguous, select the interface using `ip -br address` |
| `wrist_camera.driver_location` | Specifies whether the RealSense driver runs on this host or another ROS 2 host | Usually not; change it for a remote camera | Set it to `remote` if the camera is connected to another computer that publishes its data |
| `wrist_camera.serial_no` | Identifies the RealSense used as the wrist camera | Usually not; enter manually if no camera or multiple candidates are detected | Set it manually if automatic detection fails or more than one RealSense camera is connected |
| `wrist_camera.frame_rate_hz` | Frame rate for the RGB, depth, and aligned-depth streams | Usually not; default: 15 Hz | Set it to `6`, `15`, or `30` Hz according to available compute resources and network bandwidth |
| `wrist_camera.link6_to_camera_link.translation_xyz_m` | Translation from the `Link6` origin to the camera-frame origin, in meters | Usually not; the calibration for the supplied camera mount is used by default | Recalibrate and update this value if the camera, mount, or relative installation position changes |
| `wrist_camera.link6_to_camera_link.quaternion_xyzw` | Rotation from `Link6` to the camera frame, represented as a quaternion in `x,y,z,w` order | Usually not; the calibration for the supplied camera mount is used by default | Recalibrate and update this value if the camera, mount, or installation orientation changes |
| `site.go2_base_to_arm_base.translation_xyz_m` | Mounting translation from the Go2 base frame to the arm base frame, in meters | Usually not; defaults to `[0,0,0]` for standalone development | Keep the default for Deployment Option 1. After installing the arm on a Go2, replace it with the measured value |
| `site.go2_base_to_arm_base.quaternion_xyzw` | Mounting rotation from the Go2 base frame to the arm base frame, represented as a quaternion in `x,y,z,w` order | Usually not; defaults to the identity quaternion for standalone development | Keep the default for Deployment Option 1. After installing the arm on a Go2, replace it with the measured value |


3. Validate the generated configuration:

```bash
./deploy/configure_d1_manipulation.sh --validate
```

Proceed only after the terminal displays `Configuration: OK`. If validation fails, correct the parameters reported by the script.

### 1.10 Start the Arm Control Stack
1. Connect the RealSense D435i to the workstation over USB 3.0 and the D1 over Ethernet. Inside the container, verify connectivity to the D1 with `ping -c 3 192.168.123.100` and confirm that the camera is detected with `lsusb | grep -i realsense`.
2. Manually place the arm in the STOWED pose shown below, with the upper arm pointing in the same direction as the Ethernet port.

![D1 STOWED pose](images/stowed_pose.png)

3. Run the following commands:

```bash
cd /workspace
./scripts/real_bringup.zsh --rviz
```

During startup, the system checks the onboard service version, joint feedback, RealSense camera, IMU, TF tree, MoveIt, and task interfaces. It does not initiate any pick-and-place motion automatically. The service is ready only when the following banner appears:

+ Note: The control stack checks the arm's current pose during startup. Startup will fail if the pose differs significantly from the one shown above.

```bash
************************************************************
* D1 REAL CONTROL STACK READY                              *
* pick_object and drop_object are ready to accept commands *
************************************************************
```

![Physical control stack ready](images/real_stack_ready.png)

Do not call any task interfaces unless this banner appears. In Deployment Option 1, the control stack runs inside the persistent development container, and runtime logs are stored in `/tmp/d1_ros_logs/` within that container.

Also verify the following in RViz:

1. The complete arm model is visible, and its joint configuration approximately matches the physical arm.
2. The wrist-camera RGB image updates continuously.
3. The pseudo-colored aligned-depth image updates continuously and is spatially aligned with the RGB image.
4. Do not proceed to physical pick-and-place testing if the RViz window, robot model, or either camera stream is abnormal. It is normal for the **Wrist Camera YOLO** panel to display **No Image** immediately after startup; the panel begins displaying images when the visual-estimation stage invokes YOLO.

![RViz physical model and wrist camera](images/rviz_real_system.png)

### 1.11 Validate Pick-and-Place Operation
Validation consists of a basic deployment check followed by a physical pick-and-place test. Keep the control-stack terminal running, open another Bash terminal on the host, and enter the same development container:

```bash
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
export ROS_DOMAIN_ID=$(python3 -c 'import yaml; print(yaml.safe_load(open("/workspace/src/d1_bringup/config/real_machine.local.yaml"))["deployment"]["ros_domain_id"])')
```

#### 1.11.1 Basic Deployment Check
First, verify the arm's public interfaces:

```bash
ros2 action list | grep '^/arm/tasks/'
ros2 topic list | grep '^/arm/task_status$'
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

The output must include:

```text
/arm/tasks/drop_object
/arm/tasks/pick_object
/arm/task_status
```

When the arm starts successfully in the STOWED pose, the key fields in `/arm/task_status` should be:

```yaml
state: 1
payload_state: 0
canonical_pose: 0
```

![READY_STOWED status](images/task_status_ready_stowed.png)

#### 1.11.2 Physical Pick-and-Place Test
1. Place a 5 cm × 5 cm × 5 cm yellow cube, which may be made from cardboard, 0.35 m along the positive Y-axis of the arm's base frame. The base frame and cube placement are shown below.

![Physical cube acceptance layout](images/cube_test_layout.png)

2. Make sure that no people or obstacles are within the arm's workspace, then run the following command:

```bash
ros2 action send_goal /arm/tasks/pick_object d1_interfaces/action/PickObject "{target: {header: {frame_id: base_link}, point: {x: 0.0, y: 0.35, z: 0.0}}, stop_after: 4}" --feedback
```

If the pick succeeds, the arm physically lifts the object and moves to the CARRY pose. The action result contains `success: true`, `outcome: 0`, `final_task_state: 2`, and `payload_state: 1`.

3. You may continue to the drop test after either a successful pick that ends in `READY_CARRY` or a failed pick that safely returns to `READY_STOWED`. If the system enters `FAULTED`, an unrecoverable fault has occurred and must be diagnosed before continuing.
4. To test dropping, assume that the target in the arm's base frame remains `{x: 0.0, y: 0.35, z: 0.0}}`, then run:

```bash
ros2 action send_goal /arm/tasks/drop_object d1_interfaces/action/DropObject "{target: {header: {frame_id: base_link}, point: {x: 0.00, y: 0.35, z: 0.0}}}" --feedback
```

> Note: Pressing `Ctrl+C` during execution requests that the arm stop and hold its current pose. After confirming that the arm is within a safe recovery range and that its workspace is clear, run `./scripts/arm_stowed_control.zsh --confirm STOWED_MOVE` from `/workspace` to request a return to STOWED. This utility cannot guarantee a safe recovery from every possible pose.
>

If the drop succeeds, the object is physically released and the arm returns to the STOWED pose. The action result contains `success: true`, `outcome: 0`, `returned_to_stowed: true`, `final_task_state: 1`, and `payload_state: 0`. Finally, run:

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

Confirm that the arm has returned to `READY_STOWED`.

5. Press `Ctrl+C` once in the manipulation-service terminal and wait for the control stack to finish shutting down. This stops the service but does not remove the development container.

After validation, exit all container terminals and stop the development container from the host:

```bash
exit
sudo docker stop unitree-d1-manipulation-dev-amd64
```

To run another pick-and-place session later, open a terminal on the host and run:

```bash
# start and enter the container
sudo docker start unitree-d1-manipulation-dev-amd64
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash

# cd to workspace
cd /workspace

# start control stack and wait until the banner appears
./scripts/real_bringup.zsh --rviz
```

Then open a second terminal on the host and run:

```bash
# enter the container
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash

# cd to workspace
cd /workspace

# set ROS Domain ID
export ROS_DOMAIN_ID=$(python3 -c 'import yaml; print(yaml.safe_load(open("/workspace/src/d1_bringup/config/real_machine.local.yaml"))["deployment"]["ros_domain_id"])')

# pick an object
ros2 action send_goal /arm/tasks/pick_object d1_interfaces/action/PickObject "{target: {header: {frame_id: base_link}, point: {x: 0.0, y: 0.35, z: 0.0}}, stop_after: 4}" --feedback

# drop an object
ros2 action send_goal /arm/tasks/drop_object d1_interfaces/action/DropObject "{target: {header: {frame_id: base_link}, point: {x: 0.00, y: 0.35, z: 0.0}}}" --feedback
```



## Deployment Option 2: Unitree Go2
+ This option is intended for full-system integration with Go2 navigation, locomotion control, and task management, as well as final deployment in the target environment.
+ Before following this procedure, complete Deployment Option 1 and run at least one full pick-and-place cycle on an `x86_64` workstation. This verifies the arm, wrist camera, perception model, and public interfaces before Go2 integration.
+ This option requires a Unitree Go2 EDU equipped with an expansion computing unit. Running `uname -m` on the Go2 must return `aarch64`. The Go2 onboard computer ultimately runs the arm's perception, planning, and control services without relying on an external workstation.
+ The AMD64 Docker image and the `build/`, `install/`, and `log/` artifacts produced in Deployment Option 1 cannot be reused on the Go2. Rebuild the ARM64 image and workspace on the Go2.
+ Reserve at least 20 GB of free disk space on the Go2 onboard computer.

### 2.1 Connect the Hardware
1. Securely mount the D1-550 on the Go2, then connect the D1 to the Go2 expansion dock over Ethernet.
2. Connect the wrist-mounted RealSense D435i to a USB 3.0 port on the expansion dock.
3. Power on the D1 and Go2, clear the arm's workspace, and physically support the arm.
4. Connect the Go2 to the development workstation over Ethernet.

### 2.2 Log In to the Go2 Expansion Computing Unit
1. Configure the development workstation's network as described in Deployment Option 1, then confirm that `ping 192.168.123.18` receives replies.
2. Run the following command in a terminal on the development workstation:

```bash
# execute in workstation
ssh unitree@192.168.123.18 # the default password is 123
```

### 2.3 Install the Required Tools
After logging in, run the following commands on the Go2:

```bash
# execute in a Go2 terminal
sudo apt update
sudo apt install -y git git-lfs python3 python3-yaml openssh-client sshpass tar usbutils
git lfs install
```

The deployment script installed in a later step provides Docker, ROS 2, MoveIt, Conda, and the required Python perception packages. You do not need to install these dependencies manually on the Go2 host.

### 2.4 Verify Connections from the Go2
1. Verify that the Go2 can reach the D1:

```bash
# execute in a Go2 terminal
ping -c 3 192.168.123.100
```

   You should receive three replies.

2. Verify that the Go2 can detect the RealSense camera:

```bash
# execute in a Go2 terminal
lsusb | grep -i realsense
```

   An Intel RealSense device should be listed. If there is no output, check the USB 3.0 port, cable, expansion-dock power, and camera connection.

3. Do not proceed if either check fails.

### 2.5 Clone the Repository
Deploy the project repository directly on the Go2 expansion computing unit. Run:

```bash
mkdir -p ~/Project && cd ~/Project
git clone https://github.com/yuhanliu0121/unitree_arm.git
cd ~/Project/unitree_arm
git lfs pull
```

### 2.6 Build the ARM64 Docker Environment
This project uses Docker to package ROS 2, MoveIt, the RealSense driver, the perception stack, and other runtime dependencies. The Go2 deployment uses an ARM64 image and runs perception on the CPU; Python packages are installed through Miniforge/conda-forge. Build the image before configuring the runtime parameters so that the configuration script can use the container's librealsense tools to detect the wrist camera when those tools are not installed on the Go2 host.

Run the following commands on the Go2:

```bash
# execute in a Go2 terminal
cd ~/Project/unitree_arm
./deploy/install_d1_runtime_env.sh
```

The script installs Docker if necessary, builds the ARM64 image based on Ubuntu 22.04 and ROS 2 Humble, mounts the repository at `/workspace` inside the container, and runs `colcon build` and the test suite. The source code and the `build/`, `install/`, and `log/` directories remain under `unitree_arm/` on the Go2 host.

Successful output includes:

```text
D1 runtime environment is ready.
  architecture: arm64
  image:        unitree-d1-manipulation:arm64
```

Verify the build:

```bash
# execute in a Go2 terminal
test -f install/setup.bash && echo "ROS workspace: OK"
sudo docker image inspect unitree-d1-manipulation:arm64 >/dev/null && echo "Docker image: OK"
```

Expected output:

```text
ROS workspace: OK
Docker image: OK
```

If the build or any test fails, inspect the first error reported in the terminal and the contents of `log/`. Do not proceed to full-system testing.

### 2.7 Verify the D1 Onboard Control Service
You may skip this step only if you are using the same D1 that was updated in Deployment Option 1. Otherwise, verify the service status and protocol version from the Go2:

```bash
# execute in a Go2 terminal
# try 123 is a password is needed
ssh ubuntu@192.168.123.100 'systemctl is-active d1-control.service && cat /home/ubuntu/marm_code/build/d1-control-release.env'
```

Expected output:

```text
active
D1_CONTROL_VERSION=0.1.0
D1_CONTROL_PROTOCOL=2
```

If the service is inactive or the version does not match, support the arm, clear its surrounding workspace, and run:

```bash
cd ~/Project/unitree_arm
./deploy/update_d1_onboard_sdk.sh --confirm UPDATE_D1_ONBOARD_SDK
```

A successful update produces:

```text
D1 onboard SDK update passed. The d1-control.service is enabled and active.
```

### 2.8 Configure the Runtime Parameters
The manipulation service requires several parameters, including the D1 network interface, RealSense serial number, ROS Domain ID, perception model, and device extrinsics. The configuration script automatically detects the Go2 platform, the network interface connected to the D1, and the wrist-mounted RealSense camera, then supplies validated defaults.

1. Connect the D1 and RealSense D435i to the Go2 expansion dock. Confirm that `lsusb | grep -i realsense` detects the camera and that `ping -c 3 192.168.123.100` receives replies.
2. Run the configuration script:

```bash
cd ~/Project/unitree_arm
./deploy/configure_d1_manipulation.sh
```

The script generates `src/d1_bringup/config/real_machine.local.yaml` and fills in all values that can be detected automatically, along with the project's defaults. The table below describes every parameter and identifies those that require manual review or modification.

| Parameter | Purpose | Manual configuration required | Go2 deployment requirement |
| --- | --- | --- | --- |
| `deployment.platform` | Identifies the deployment platform and validates the host architecture | No | The script must detect and set this to `go2` |
| `deployment.ros_domain_id` | Domain ID used for ROS 2 node discovery and communication | Yes | Must match the Domain ID used by the Go2 navigation, locomotion-control, and task nodes; valid range: `0–232` |
| `perception.model_path` | Path to the YOLO object-detection model | Usually yes | The default model is intended only for the objects and environments validated by this project. For other deployments, use a model trained and validated with data from the actual objects and operating environment |
| `arm.serial_no` | Selects arm-specific URDF corrections, currently used mainly for joint-limit overrides | Usually not | Set this only if the arm requires unit-specific joint-limit or other URDF corrections and a corresponding entry exists in `hardware_profiles.yaml` |
| `arm.network_interface` | Wired network interface used by the Go2 to exchange native control and feedback data with the D1 | Usually not | The script fills this automatically when exactly one candidate is found. If none or multiple candidates are detected, select the interface using `ip -br address` |
| `wrist_camera.driver_location` | Specifies whether the RealSense driver runs locally or on another ROS 2 host | Usually not | Keep this set to `local` when the D435i is connected to the Go2 |
| `wrist_camera.serial_no` | Identifies the RealSense used as the wrist camera | Usually not | The script fills this automatically when exactly one RealSense is found. If multiple cameras are connected, enter the wrist-camera serial number manually |
| `wrist_camera.frame_rate_hz` | Frame rate for the RGB, depth, and aligned-depth streams | Usually not | Default: `15` Hz. Set it to `6`, `15`, or `30` Hz according to the Go2's available compute resources and network bandwidth |
| `wrist_camera.link6_to_camera_link.translation_xyz_m` | Translation from the `Link6` origin to the wrist-camera frame origin, in meters | Usually not | Keep the default if the camera and mount are installed exactly as in Deployment Option 1. Recalibrate if the mounting position changes |
| `wrist_camera.link6_to_camera_link.quaternion_xyzw` | Rotation from `Link6` to the wrist-camera frame, represented as a quaternion in `x,y,z,w` order | Usually not | Keep the default if the camera and mount orientation matches Deployment Option 1. Recalibrate if the mounting orientation changes |
| `site.go2_base_to_arm_base.translation_xyz_m` | Mounting translation from the Go2 base frame to the arm base frame, in meters | Yes | Enter the measured value; the default zero translation is not permitted for a Go2 deployment |
| `site.go2_base_to_arm_base.quaternion_xyzw` | Mounting rotation from the Go2 base frame to the arm base frame, represented as a quaternion in `x,y,z,w` order | Yes | Enter the measured value; the default identity quaternion is not permitted for a Go2 deployment |


3. Validate the generated configuration:

```bash
./deploy/configure_d1_manipulation.sh --validate
```

Proceed only after the terminal displays `Configuration: OK`. If validation fails, correct the parameters reported by the script.

### 2.9 Start the Manipulation Service
1. Confirm that both the D1 and RealSense camera are connected to the Go2.
2. Manually place the arm in the STOWED pose shown in Deployment Option 1 under **Start the Arm Control Stack**.
3. Run:

```bash
# execute in a Go2 terminal
cd ~/Project/unitree_arm
./deploy/start_d1_manipulation.sh
```

RViz is disabled by default for production deployment on the Go2. During startup, the system checks the onboard service version, joint feedback, RealSense camera, IMU, TF tree, MoveIt, and task interfaces. It does not initiate any pick-and-place motion automatically. The manipulation service is ready only when the following banner appears:

```text
************************************************************
* D1 REAL CONTROL STACK READY                              *
* pick_object and drop_object are ready to accept commands *
************************************************************
```

Do not call any task interfaces unless this banner appears. Runtime logs are stored in `unitree_arm/log/runtime/` on the Go2 host.

### 2.10 Validate the Manipulation Service
Validation consists of a basic deployment check followed by a physical pick-and-place test. Keep the manipulation-service terminal running, open another terminal on the Go2, and enter the container:

```bash
# execute in a new Go2 terminal
sudo docker exec -it unitree-d1-manipulation-arm64 /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
export ROS_DOMAIN_ID=$(python3 -c 'import yaml; print(yaml.safe_load(open("/workspace/src/d1_bringup/config/real_machine.local.yaml"))["deployment"]["ros_domain_id"])')
```

#### 2.10.1 Basic Deployment Check
Verify the public interfaces:

```bash
# execute in a Go2 terminal
ros2 action list | grep '^/arm/tasks/'
ros2 topic list | grep '^/arm/task_status$'
```

The output must include at least:

```text
/arm/tasks/drop_object
/arm/tasks/pick_object
/arm/task_status
```

Read the current arm status:

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

When the arm starts successfully in the STOWED pose, the key state fields should be:

```yaml
state: 1
payload_state: 0
canonical_pose: 0
```

These values correspond to:

```text
READY_STOWED
PAYLOAD_EMPTY
POSE_STOWED
```

Do not send pick-and-place requests if the system remains in `INITIALIZING` or enters `FAULTED`. Diagnose the problem using the `failure_code` and `detail` fields in `/arm/task_status` and the logs under `log/runtime/`.

Next, verify that the wrist-camera RGB, aligned-depth, and IMU data are being published continuously:

```bash
ros2 topic hz /wrist_camera/color/image_raw
ros2 topic hz /wrist_camera/aligned_depth_to_color/image_raw
ros2 topic echo /wrist_camera/imu --once
```

After each `ros2 topic hz` reading stabilizes, press `Ctrl+C` before running the next command. The RGB and aligned-depth rates should be close to the configured value, which is 15 Hz by default, and the IMU command should return current data. To inspect the images directly in a graphical Go2 session, restart the service with `./deploy/start_d1_manipulation.sh --rviz` and apply the RViz checks described in Deployment Option 1.

#### 2.10.2 Physical Pick-and-Place Test
1. Keep the Go2 stable and close to the ground, and clear the workspace around both the Go2 and the arm.
2. Place a yellow cube 0.35 m along the positive Y-axis of the arm's base frame. Before running the command, replace the example coordinates and `frame_id` with the values actually estimated by the Go2.

```bash
ros2 action send_goal /arm/tasks/pick_object d1_interfaces/action/PickObject "{target: {header: {frame_id: base_link}, point: {x: 0.0, y: 0.35, z: -0.20}}, stop_after: 4}" --feedback
```

> Note: Pressing `Ctrl+C` during execution requests that the arm stop and hold its current pose. After confirming that the arm is within a safe recovery range and that its workspace is clear, run `./scripts/arm_stowed_control.zsh --confirm STOWED_MOVE` from `/workspace` to request a return to STOWED. This utility cannot guarantee a safe recovery from every possible pose.
>

If the pick succeeds, the arm physically lifts the object and moves to the CARRY pose. The action result contains `success: true`, `outcome: 0`, `final_task_state: 2`, and `payload_state: 1`.

3. After confirming that the pick succeeded, send a drop request:

```bash
ros2 action send_goal /arm/tasks/drop_object d1_interfaces/action/DropObject "{target: {header: {frame_id: base_link}, point: {x: 0.00, y: 0.40, z: -0.20}}}" --feedback
```

If the drop succeeds, the object is physically released and the arm returns to the STOWED pose. The action result contains `success: true`, `outcome: 0`, `returned_to_stowed: true`, `final_task_state: 1`, and `payload_state: 0`. Finally, run:

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

Confirm that the arm has returned to `READY_STOWED`.

4. Press `Ctrl+C` once in the manipulation-service terminal. The launcher stops all child ROS processes and removes the runtime container.
5. Confirm that the container has stopped:

```bash
sudo docker ps --format '{{.Names}}' \
  | grep '^unitree-d1-manipulation-arm64$'
```

No output means that the service has stopped. If a container with the same name is still running, diagnose the previous shutdown failure before starting another instance of the manipulation service.

### 2.11 Verify Integration with the Go2 Task Node
The Go2 task node must use the same `ROS_DOMAIN_ID` as the manipulation service and depend on the message and action definitions provided by `d1_interfaces`. The manipulation module exposes only the following interfaces to the Go2 application layer:

```text
/arm/tasks/pick_object
/arm/tasks/drop_object
/arm/task_status
```

For request fields, result fields, state enumerations, and failure-handling behavior, see the [D1 Manipulation API documentation](../docs/api/README_CN.md).



## Troubleshooting

### Onboard Service or Protocol Version Mismatch
Run the D1 onboard control-service update again. Do not bypass the version check or automatically fall back to an unknown onboard executable.

### RealSense Preflight Check Fails
Confirm that the D435i is connected to a USB 3.0 port on the Go2 expansion dock, then check the configured camera serial number, driver location, and frame rate. If another ROS 2 computer publishes the camera data, set `wrist_camera.driver_location` to `remote` and use the same `ROS_DOMAIN_ID` on both computers.

### The System Enters `FAULTED`
Read `failure_code` and `detail` from `/arm/task_status`, and inspect `unitree_arm/log/runtime/` on the Go2 host. Before retrying the task, verify the arm's physical pose, joint feedback, collision state, and onboard control service.

### The Go2 Task Node Cannot Discover the Arm Interfaces
Confirm that the manipulation container and Go2 task node use the same `ROS_DOMAIN_ID`, and verify that the calling workspace has built and sourced `d1_interfaces`. Docker uses host networking, so ROS 2 ports do not normally require additional mapping.
