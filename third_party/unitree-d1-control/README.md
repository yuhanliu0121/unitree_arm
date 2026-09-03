# Unitree D1 Robotic Arm Control

In-tree, unofficial control and diagnostics layer for the Unitree D1 robotic arm.
This project is not affiliated with or endorsed by Unitree Robotics.

It provides timed seven-joint commands, measured joint feedback and guarded
maintenance tools while keeping the physical serial bus under one owner. It
does **not** reimplement motion planning or ROS 2 Actions: applications should
continue using mature components such as MoveIt for those functions.

## Why this exists

The official D1 high-level interface can send a single joint target or a
seven-joint pose, but it does not expose a general multi-joint trajectory
executor with configurable timing. Sending seven low-level servo messages
directly also lacks group completeness and serial-bus ownership guarantees.

This project adds:

- native long-segment commands with a common arrival time and configurable
  acceleration/deceleration intervals;
- complete seven-joint targets identified by one sequence number;
- latest-target semantics instead of an accumulating waypoint queue;
- a single D1-computer thread that owns command, damping and feedback serial I/O;
- bounded command blocking by spreading feedback queries across the cycle;
- a host gateway that isolates the vendor Cyclone DDS ABI from ROS 2 processes;
- a stable loopback packet consumed by integrations such as `d1_ros2_control`.

See the detailed [capability matrix](docs/CAPABILITY_MATRIX_CN.md) and
[architecture](docs/ARCHITECTURE.md).

## Repository layout

```text
unitree-d1-control/
├── host/src/                 # Host UDP-to-Unitree-DDS gateway
├── onboard/src/              # D1-computer grouped serial dispatcher
├── include/d1_control/
├── tests/                    # Dependency-free protocol checks
├── docs/
├── CMakeLists.txt
├── CHANGELOG.md
├── CONTRIBUTING.md
├── SECURITY.md
└── NOTICE.md
```

## Dependencies

- C++17 and CMake 3.16 or newer;
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) and its
  matching Cyclone DDS C/C++ libraries;
- the [official Unitree D1 SDK](https://unitree-firmware.oss-cn-hangzhou.aliyuncs.com/tool/d1_sdk.zip)
  source tree for generated DDS message definitions;
- FashionStar UART-servo and CSerialPort libraries for the onboard target.

Vendor source remains separate from this component. In the arm workspace,
the official `D1_SDK` is stored beside this directory; standalone builds may
instead pass `D1_OFFICIAL_SDK_ROOT` explicitly.

## Host build

```bash
cmake -S . -B build/host -DD1_OFFICIAL_SDK_ROOT=/path/to/D1_SDK -DD1_CONTROL_BUILD_HOST=ON -DD1_CONTROL_BUILD_ONBOARD=OFF
cmake --build build/host -j2
cmake --build build/host --target test
```

## D1-computer build

```bash
cmake -S . -B build/onboard -DD1_OFFICIAL_SDK_ROOT=/path/to/D1_SDK -DD1_CONTROL_BUILD_HOST=OFF -DD1_CONTROL_BUILD_ONBOARD=ON
cmake --build build/onboard -j2
cmake --build build/onboard --target test
```

## Runtime topology

```text
MoveIt / application (host)
        |
        v
joint-segment Action adapter
        |
        v
D1 adapter -- loopback UDP --> d1_control_gateway
                                      |
                                      v
                         one complete DDS segment
                                      |
                                      v
                     d1_control_node
                                      |
                                      v
                native interval commands, one serial owner
```

For normal motion, the host sends seven targets and a native timing profile.
The onboard component calls the servo library's `setRawAngle()` command once
per joint. `common_arrival` compensates UART dispatch skew and makes all joints
finish together. `uniform_joint_speed` derives one configurable angular speed,
converts each joint displacement into its own delay, and therefore matches the
primitive and timing rule used by Unitree's `funcode=2, mode=1` path. A
high-frequency waypoint replay API is intentionally outside this repository's
supported control surface.

## Maintenance tools

The host build also provides three guarded tools. They talk directly to the
already-running onboard executor and do not start a second serial owner:

```bash
d1_all_joints_unload --interface enp3s0 --confirm UNLOAD_ALL
d1_move_stowed --interface enp3s0 --confirm STOWED_MOVE
d1_move_zero --interface enp3s0 --confirm ZERO_MOVE
```

`d1_move_stowed` commands `[0, -1.54, 1.55, 0, 0, 0]` radians for Joint0–5;
`d1_move_zero` commands zero for Joint0–5. Both leave Joint6 fully open at
60 degrees, choose a common-arrival duration from the largest joint motion at
15 degrees/second, and verify all seven joints within 2 degrees. Use
`--duration-ms` to override the automatically chosen duration. The unload tool
sends damping power zero to Joint0–6 and cannot verify torque release from
angle feedback, so physical confirmation remains mandatory.

Compatibility names are also built for the utilities previously shipped in
the local official-SDK build directory:

| Executable | Enhanced behavior |
| --- | --- |
| `all_joints_unload` | Alias of `d1_all_joints_unload` |
| `arm_stowed_control` | Alias of `d1_move_stowed` |
| `arm_zero_control` | Alias of `d1_move_zero` |
| `joint_angle_control` | Replaces one joint in a fresh seven-joint snapshot and sends one native segment |
| `multiple_joint_angle_control` | Sends a validated seven-joint native segment |
| `joint_delta_control` | Applies a bounded relative change to one joint while preserving the other six |
| `joint_enable_control` | Unloads all joints, or re-enables them by holding fresh feedback |
| `get_arm_joint_angle` | Reads the enhanced executor's measured feedback |
| `d1_interface_probe` | Read-only feedback/rate monitor for the enhanced interface |

The old `d1_interface_probe` protocol-fuzzing commands intentionally are not
carried forward: they characterize the bypassed vendor high-level controller,
not this executor.

## Safety

Never run the vendor `marm_controller_node` and this project's onboard process
at the same time: both access `/dev/ttyS4`. Read
[DEPLOYMENT.md](docs/DEPLOYMENT.md) and test rollback before commanding motion.

## License

This project is licensed under the [MIT License](LICENSE). Vendor SDK headers,
generated messages and runtime libraries are external dependencies and remain
subject to their respective terms.
