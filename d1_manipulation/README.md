# D1 manipulation

The first implemented task primitive is the internal eye-in-hand observation
step exposed for development as:

```text
/arm/debug/observe_target
```

It accepts a stamped coarse target centre, constructs the configured ordered
camera-pose candidates in a gravity-aligned frame, and executes the first
candidate for which MoveIt finds a complete collision-free plan.

Run the automatic seed-0 cube acceptance:

```zsh
./accept_observe_target.zsh --rviz
```

Keep the stack running for manual goals:

```zsh
./accept_observe_target.zsh --rviz --server-only
```

Then send a goal from another sourced terminal:

```zsh
ros2 action send_goal /arm/debug/observe_target \
  d1_manipulation/action/ObserveTarget \
  "{target: {header: {frame_id: go2_base}, point: {x: 0.45, y: 0.0, z: -0.14}}}" \
  --feedback
```

The automatic seed-0 regression selected candidate 56 (`beta=0 deg`,
`alpha=65 deg`, `distance=0.60 m`) and measured the executed target projection
0.3 pixels from the calibrated RGB image centre.

## Wrist perception adapter

`detect_target_server` is the ROS boundary around the frozen
`perception_runtime_v1` package. It subscribes to:

- `/wrist_camera/color/image_raw` (`rgb8` or `bgr8`);
- `/wrist_camera/aligned_depth_to_color/image_raw` (`16UC1` millimetres or
  `32FC1` metres);
- `/wrist_camera/aligned_depth_to_color/camera_info`.

The `/arm/perception/detect_target` service accepts a coarse
`geometry_msgs/PointStamped` target hint. Each request clears cached frames,
waits for a new timestamp-matched RGB-D pair, runs the unchanged arm-profile
perception runtime, and chooses the depth-verified mask nearest the projected
hint. Its response contains the class, confidence, mask centre, and the robust
visible-surface point-cloud centre in both the color optical frame and
`base_link`.

The reported 3-D point is deliberately named an observation centre: a single
camera view cannot recover the hidden half of an object, so it is not the full
object's geometric centre. Cube, bowl, and zucchini grasp estimators must apply
their own geometry after this adapter.

The node publishes its latest annotated image on
`/arm/perception/debug/overlay`; the standard RViz configuration enables this
panel. The adapter owns no arm motion and does not change the released model,
thresholds, or depth filter.

## Visual yellow-cube grasp

The formal staged development Action is `/arm/tasks/pick_object`. For the
seed-0 simulation scene, run one of:

```zsh
./accept_visual_cube_grasp.zsh --headless --stage compute
./accept_visual_cube_grasp.zsh --rviz --stage pregrasp
./accept_visual_cube_grasp.zsh --rviz --stage descend
./accept_visual_cube_grasp.zsh --rviz --stage lift
./accept_visual_cube_grasp.zsh --rviz --stage carry
```

The pipeline performs an oblique RGB-D observation, gravity-constrained ground
RANSAC, a strict top-down RGB observation, metric top-contour projection,
minimum-area square fitting, ordered symmetric grasp planning, and the selected
execution stage. RViz exposes `Cube Grasp Geometry`; the latest coarse/fine
images and JSON estimates are copied into the acceptance log directory.

MoveIt deliberately does not receive MuJoCo ground-truth collision geometry
for the cube, bowl, or zucchini. Its world contains the arm, the Go2 proxy and
the perception-fitted ground plane; MuJoCo retains the complete physical object
scene, and RViz retains the independent object-mesh visualization. A grasped
object will be added from perception as an attached collision object rather
than copied from simulation truth.

The `carry` stage attaches the visually estimated 50 mm cube to `tcp_link` for
MoveIt collision checking, executes the configured CARRY joint pose, and then
checks three fresh wrist RGB frames. Yellow HSV connected components are
measured only inside an ROI projected from the expected held-object position;
MuJoCo object truth is not used by this verification.
