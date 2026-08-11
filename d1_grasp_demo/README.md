# D1 fixed cube grasp

Fixed application poses are stored in `config/fixed_poses.yaml`. Each pose
contains six arm joint targets in radians (`Joint0` through `Joint5`); gripper
open/close state remains a separate state-machine action. `home` and
`place_ready` intentionally remain separate keys even while their current
joint values are identical.

This package is a simulation-only manipulation baseline. It receives MuJoCo
object truth over UDP loopback, plans to a pose above the yellow cube, descends
along a Cartesian path, closes the gripper, attaches the cube in MoveIt's
planning scene, lifts 0.10 m, and verifies that the physical MuJoCo cube stays
raised for 12 seconds.

The MoveIt planning scene also contains the same single Go2 bounding box used
by MuJoCo (`0.753442 x 0.338254 x 0.255121 m`). It is rigidly attached to
`base_link`, so it follows the complete platform during later GT navigation.
Only the unavoidable mounting overlap with `base_link` is allowed; moving arm
links must avoid the platform box.

It deliberately does not use camera data yet. The UDP scene-state protocol is
bound to `127.0.0.1:15002` and is not part of the physical D1 interface.

Before lift, the node compares the MuJoCo TCP with the measured cube centre,
reports both finger collision centres, and rejects the grasp unless both
fingers contact the cube. It also reports simulated Joint6 position and summed
normal force per finger. Final success is based only on the physical MuJoCo
body pose; MoveIt's attached collision object cannot make the check pass.

The acceptance trajectory defaults to 10% velocity and acceleration scaling.
This keeps the measured 8.97 Hz D1 feedback path inside the existing 0.15 rad
controller tolerance when MuJoCo and RViz are rendering simultaneously.
