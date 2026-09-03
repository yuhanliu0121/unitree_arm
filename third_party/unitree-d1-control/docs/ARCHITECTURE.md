# Architecture

## Responsibility split

| Layer | Responsibility |
|---|---|
| MoveIt/application | Generate task and geometric trajectories |
| Joint-segment Action adapter | Select the stage endpoint, motion profile and completion policy |
| D1 host adapter | Validate and transmit one complete Joint0–6 segment |
| `d1_control_gateway` | Isolate vendor DDS ABI and publish one sequenced segment |
| Onboard executor | Validate the segment, compensate serial skew and serialize motor-bus access |
| D1 servos | Execute native timed interpolation and drive motors |

## Command semantics

Every normal motion command is a complete Joint0–6 endpoint with a requested
maximum duration, an acceleration/deceleration profile and a named timing
profile. Arm-only and gripper-only operations are merged by the upper
controller before transport; the onboard process therefore never guesses
omitted joint targets.

Both timing profiles use `setRawAngle()`, the primitive exercised by Unitree's
`funcode=2, mode=1` implementation. With `common_arrival`, every servo receives
nearly the same delay, so joints move at different effective speeds and finish
together; UART command-start skew is compensated. With
`uniform_joint_speed`, the onboard executor derives the requested speed from
the largest displacement and maximum duration, then maps every joint
displacement to its own delay. Normal position control never uses
`setRawAngleByInterval()`.

Every `uniform_joint_speed` command still dispatches all seven joint targets.
Each derived interval is clamped by `D1_MIN_JOINT_INTERVAL_MS` (10 ms by
default), preventing near-zero joint deltas from becoming pathological 1 ms
servo commands. The clamp does not change `common_arrival`.

## Serial ownership

Only the onboard I/O thread calls the servo library. DDS callbacks validate and
store data but never touch `/dev/ttyS4`. Feedback reads are performed one joint
per scheduling opportunity, so a command waits for at most one query rather
than a complete seven-joint sweep.

## Deliberate non-goals

- motion planning and collision checking;
- continuous multi-waypoint trajectory interpolation or retiming;
- replacing official enable, damping and zeroing firmware functions;
- velocity, torque or impedance control;
- safe coexistence with the vendor serial controller.
