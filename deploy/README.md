# D1 manipulation deployment

[中文文档](README_CN.md) | English

Deployment has three explicit steps. The Docker image contains only the
runtime/toolchain; this repository is bind-mounted at `/workspace`. Therefore
`colcon` creates `build/`, `install/`, and `log/` directly in the same host
directory as the business source.

## 1. Update the D1 onboard SDK

Connect the deployment computer directly to the D1 board and manually put its
wired interface on `192.168.123.0/24`. Support the arm and clear its workspace,
then run:

```bash
./deploy/update_d1_onboard_sdk.sh --confirm UPDATE_D1_ONBOARD_SDK
```

The script prefers SSH keys and falls back to the documented `ubuntu`/`123`
credentials. It builds natively on the board, runs the protocol test, backs up
the existing deployment, atomically installs the executable, manifest and
systemd unit, and verifies that `d1-control.service` is active. It does not
change the host network and does not command arm motion.

## 2. Detect and validate machine-local configuration

Run this once on each deployment computer:

```bash
./deploy/configure_d1_manipulation.sh
```

The helper identifies amd64 or the arm64 Go2 platform, and fills unambiguous
D1-interface and RealSense-serial candidates. Complete the `REQUIRED` fields in
`src/d1_bringup/config/real_machine.local.yaml`. The final-object perception
model and current hand-eye calibration are supplied as defaults. Arm serial is
optional and selects a matching per-arm URDF profile when one has been added.
Then run:

```bash
./deploy/configure_d1_manipulation.sh --validate
```

Do not edit the tracked defaults or the generated effective file.

## 3. Install the host runtime environment

Run this independently on the amd64 development PC and the arm64 Go2 computer:

```bash
./deploy/install_d1_runtime_env.sh
```

The script detects the native architecture and builds either
`unitree-d1-manipulation:amd64` or `unitree-d1-manipulation:arm64`. On amd64 it
creates the persistent `unitree-d1-manipulation-dev-amd64` development
container and builds the workspace, including commissioning tools, inside it.
On arm64 it keeps the production-oriented ephemeral build-container workflow.
Docker is installed from Ubuntu's `docker.io` package only when it is absent.
Both architectures use Ubuntu 22.04, ROS 2 Humble and the CPU perception stack.
The amd64 target additionally includes the MuJoCo development dependencies;
the arm64 target remains focused on the physical-arm runtime.

After changing business code on amd64, rebuild directly inside the persistent
development container. Re-run the installer only when the Docker environment
changes; stop the existing development container before doing so. Host-side
`build/`, `install/`, and `log/` keep subsequent builds incremental.

## 4. Start the manipulation service

On amd64, enter the persistent development container and start the stack
directly:

```bash
docker exec -it unitree-d1-manipulation-dev-amd64 \
  /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
./scripts/real_bringup.zsh --rviz
```

On arm64, use the production service-container launcher:

```bash
./deploy/start_d1_manipulation.sh
./deploy/start_d1_manipulation.sh --rviz
```

All options after the script name are forwarded to `real_bringup.zsh`, so
commissioning-only Joint6 bypass remains explicit:

```bash
./deploy/start_d1_manipulation.sh --rviz --joint6-bypass
```

The container uses host networking, host IPC and `--privileged` on both
architectures. It runs in the foreground, rejects duplicate container names,
and forwards Ctrl+C into the existing coordinated ROS shutdown path. The
bringup itself performs the one-shot board service/protocol check before it
activates any controller.

## Files that remain deployment-specific

Edit only `src/d1_bringup/config/real_machine.local.yaml` for a deployment,
especially:

- the D1 network interface and board address;
- the Go2-base to arm-base mounting transform;
- the wrist-camera serial number, location and frame rate;
- the perception weight path;
- the final hand-eye transform and arm serial profile.

Model files, configuration and source remain on the host through the workspace
bind mount. Runtime ROS logs persist under `log/runtime/`. Rebuilding the
Docker image is unnecessary when only those files change.
