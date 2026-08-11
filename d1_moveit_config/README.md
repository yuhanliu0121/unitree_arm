# D1 MoveIt 2 configuration

The `arm` planning group is the six-joint chain from `base_link` to
`tcp_link`. `Joint6` is a separate gripper group and `Joint6_mimic` remains a
URDF mimic joint. KDL provides IK and OMPL `RRTConnect` provides the initial
collision-aware approach plan.

Start the MuJoCo service first, then launch the ROS control and planning stack:

```bash
source /opt/ros/humble/setup.zsh
source /home/tony/Project/unitree_arm/dev/install/setup.zsh
ros2 launch d1_moveit_config move_group.launch.py
```

Run the fixed-cube grasp after all controllers and `move_group` are ready:

```bash
ros2 launch d1_moveit_config pick_cube.launch.py
```

The demo is intentionally simulation-only. Object truth uses UDP loopback port
15002; it is not exposed by or accepted from the physical D1 control path.

The shared vendor URDF retains its STL collision geometry so MuJoCo can import
the original link-frame transforms correctly. At launch, this package creates
a planning-only robot description with conservative primitive collisions for
KDL/MoveIt. The two descriptions share joints and TF but are intentionally not
the same collision representation.

The seed-0 acceptance sequence is:

1. plan an OMPL approach 0.12 m above the cube;
2. descend vertically with a Cartesian path;
3. close the gripper and require contacts on both fingers;
4. attach the cube only in MoveIt's planning scene;
5. lift the TCP vertically by 0.10 m;
6. require the physical MuJoCo cube to remain at least 0.05 m above its initial
   height after a two-second hold.

The validated seed-0 run lifted the physical cube by 0.065 m after settling.

For routine acceptance from the workspace root, one command starts MuJoCo,
the complete ROS 2/MoveIt stack and the grasp node, then shuts everything down:

```bash
./accept_cube_grasp.zsh
```

Use `--headless` in a terminal-only session, or `--rviz` when the MoveIt scene
also needs to be inspected. Logs from each run are retained under
`/tmp/d1_cube_accept.*`.

The RViz configuration uses `world` as its fixed frame and keeps the live
robot, Go2 mesh and MuJoCo-driven textured object meshes visible by default.
`MuJoCo Physical Collisions` and `MoveIt Planning Collisions` are separate,
disabled-by-default displays. The object mesh layer continues to follow MuJoCo
while the target is removed from or attached to the MoveIt planning scene.
Interactive `--rviz` acceptance pauses for Enter before the grasp and after
success; non-interactive runs remain automatic.

The RViz configuration also contains an `Observation Debug` MarkerArray layer
for the selected gravity-aligned camera axes, optical ray, target, and up
direction. Start the observation Action and its complete stack with:

```bash
./accept_observe_target.zsh --rviz
```
