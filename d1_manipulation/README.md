# D1 manipulation

The first implemented task primitive is the internal eye-in-hand observation
step exposed for development as:

```text
/arm/internal/observe_target
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
ros2 action send_goal /arm/internal/observe_target \
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
perception runtime, filters candidates to the three configured task classes,
`depth_verified` status, and the configured minimum confidence, then chooses
the mask nearest the RGB optical centre. The hint remains a 3-D diagnostic but
does not rank candidates because real-arm URDF error can corrupt its live-TF
reprojection. Its response contains the class, confidence, mask centre, and
the robust visible-surface point-cloud centre in both the color optical frame
and `base_link`.

The reported 3-D point is deliberately named an observation centre: a single
camera view cannot recover the hidden half of an object, so it is not the full
object's geometric centre. Cube, bowl, and zucchini grasp estimators must apply
their own geometry after this adapter.

The node publishes its latest annotated image on
`/arm/perception/debug/overlay`; the standard RViz configuration enables this
panel. The adapter owns no arm motion and does not change the released model,
thresholds, or depth filter.

## Visual yellow-cube grasp

The public `d1_interfaces/action/PickObject` Action is served at
`/arm/tasks/pick_object`. For the
seed-0 simulation scene, run one of:

```zsh
./accept_visual_cube_grasp.zsh --headless --stage compute
./accept_visual_cube_grasp.zsh --rviz --stage pregrasp
./accept_visual_cube_grasp.zsh --rviz --stage descend
./accept_visual_cube_grasp.zsh --rviz --stage lift
./accept_visual_cube_grasp.zsh --rviz --stage carry
```

`pick_object_server` is split into a category-independent task orchestrator
and registered object strategies. The orchestrator owns observation,
classification, staged execution, failure recovery, CARRY transition and
verification dispatch. `YellowCubePickStrategy` owns cube perception,
top-view refinement, pregrasp/grasp search, gripper settings, lift parameters,
debug geometry and attached collision geometry. A detected class without a
registered strategy is rejected at `SELECT_STRATEGY`; bowl and zucchini are
intentionally not implemented by the cube strategy.

The pipeline performs an oblique RGB-D observation, gravity-constrained ground
RANSAC, a strict top-down RGB observation, metric top-contour projection,
minimum-area square fitting, ordered symmetric grasp planning, and the selected
execution stage. By default, the cube strategy additionally performs
`FINETUNE_GRASP` after PREGRASP: it refits the
ground, isolates the depth-consistent top face, checks its camera-frame 3-D
centre against the physically calibrated safe-descent prism, and makes at most
three bounded stop-and-look corrections. The zucchini strategy performs the
same staged correction using a physically calibrated closing-direction safe
slab: a point already inside the slab is accepted immediately; an outside
point uses the slab centre as its correction target. Finger-length and gravity
coordinates remain unconstrained. Bowl retains its established path. RViz
exposes the corresponding grasp geometry;
the latest coarse/fine/finetune images and JSON estimates are written to the
cube debug directory.

The finetune flow is enabled for both backends. Real arms sharing the same D1,
wrist-camera mount and URDF use the common physically calibrated prism;
`gripper_simulation.yaml` overrides its closing-axis offset because ideal
MuJoCo kinematics intentionally omit the stable physical TCP/URDF error.
The same capture infrastructure supports two-side zucchini safe-slab
calibration through `capture_zucchini_safe_region.zsh`. Both boundaries must
pass an open-gripper DESCEND physical validation before their sorted
`closing_min`/`closing_max` coordinates are enabled at runtime.

MoveIt deliberately does not receive MuJoCo ground-truth collision geometry
for the cube, bowl, or zucchini. Its world contains the arm, the Go2 proxy and
the perception-fitted ground plane; MuJoCo retains the complete physical object
scene, and RViz retains the independent object-mesh visualization. A grasped
object will be added from perception as an attached collision object rather
than copied from simulation truth.

The `carry` stage attaches the perception-estimated object to `tcp_link` for
MoveIt collision checking, executes the configured CARRY joint pose, and then
checks a fresh window of `Joint6` feedback. Its median position and the ratio
of samples above the selected object's retention threshold determine whether
the object still blocks the fingers from closing. MuJoCo object truth and
wrist-camera appearance are not used by this verification.

Gripper targets and retention thresholds are backend calibration, selected by
the required launch profile. Simulation acceptance commands `-30 deg` for all
objects and uses simulation thresholds; physical D1 operation uses the measured
object-specific targets (`cube=35`, `zucchini=0`, `bowl=-30 deg`) and matching
thresholds (`37.5`, `3`, `-28 deg`):

```zsh
ros2 launch d1_manipulation observe_target.launch.py backend:=simulation
ros2 launch d1_manipulation observe_target.launch.py backend:=real
```

`backend` is required and cannot fall back automatically. One
`arm_task_state_manager` process is the authority for both public Actions, so
`PickObject` and `DropObject` cannot overlap even though their strategy servers
remain separate processes. The latched `/arm/task_status` topic exposes the
exact state, payload knowledge, canonical-pose knowledge, active phase and last
failure. Full tasks follow this contract:

| Current state | Accepted task | Success state | Recoverable business failure |
|---|---|---|---|
| `READY_STOWED` | `PickObject` | `READY_CARRY` | collision-checked recovery to `READY_STOWED` |
| `READY_CARRY` | `DropObject` | `READY_STOWED` | recovery to `READY_CARRY` with payload retained |

Startup remains `INITIALIZING` until fresh Joint0--Joint5 feedback verifies the
canonical STOWED pose. If the arm starts within the configured 45-degree
per-joint near-STOWED envelope, the manager commands one bounded recovery target
and verifies the resulting feedback; larger deviations enter `FAULTED` without
motion. Stale joint feedback, cancellation, controller/TF/camera
failure, ambiguous payload state, or failed recovery enters terminal
`FAULTED`. A fault stops/cancels the active command and does not enqueue a
measured-position hold or another motion target. New pick/drop goals are then
rejected. `PickObject` results reduce the caller decision to `SUCCESS`,
`REPOSITION_REQUIRED`, or `ARM_FAULTED`; `DropObject` uses the same outcomes.
Partial `stop_after` goals and the debug continuation services are commissioning
tools and intentionally bypass the production task-state contract. They are
rejected/absent unless launch explicitly sets `enable_commissioning_api:=true`.
The corresponding seed and calibration executables are installed only when
building with `-DD1_BUILD_COMMISSIONING_TOOLS=ON`.

Inspect the externally visible state with:

```zsh
ros2 topic echo /arm/task_status
```

Every `PickObject` starts with `ENSURE_STOWED`. If the current pose is within
45 degrees per joint of `[0, -1.54, 1.55, 0, 0, 0]`, the server sends that
complete endpoint directly to `arm_controller`. The real backend executes it
as one D1 `funcode=2, mode=1` target while retaining a single command owner.
This initial near-STOWED recovery deliberately bypasses MoveIt's conservative
collision check. Larger deviations are held for manual recovery with the
standalone native-DDS `arm_stowed_control` utility.

The bundled pick/drop command-line clients convert `Ctrl+C` into an Action
cancel request before shutting down. This is a software stop and does not
replace the physical emergency stop during real-machine tests.

## Gravity-aligned object release

The external `d1_interfaces/action/DropObject` Action is served at
`/arm/tasks/drop_object`. Its stamped target is
the trash-bin bottom centre estimated by Go2. The production state contract
accepts it only from `READY_CARRY`; the legacy empty-gripper path remains an
internal commissioning behavior rather than a public precondition. When one
held MoveIt object exists it is detached after release. More than one attached
object is treated as an invalid planning-scene state. For reliable reachability and Go2 body clearance,
navigation should place the bin centre approximately 0.35--0.45 m
horizontally from `base_link`; this is a recommendation rather than an Action
precondition. The server searches the configured release candidates and only
requests repositioning when no collision-free IK/trajectory plan is available.

The trash bin is deliberately not added to MoveIt's planning scene: its target
point is a release reference, while the real low-profile bin is outside the
collision model used by this task. MuJoCo may still render and physically
simulate the bin independently. The server fixes `tcp_link` +Z along gravity
and searches the configured candidates in strict order. Height
offsets relative to the `base_link` gravity height are searched from the
nominal height outwards as `0, -10, +10, ..., -60, +60, -80, +80 mm`.
At each height, the target is additionally searched along `base_link` Y using
`0, +3, -3, ..., +15, -15 mm`; every height/Y pair uses yaw offsets
`0, +15, -15, ..., +90, -90 deg` from the projected CARRY TCP x-axis. Every
candidate is logged as an IK failure, planning failure, or success. The first
release pose with a complete plan is executed. The server confirms that the gripper reached its fully open target,
detaches the held object from MoveIt's gripper model, holds the physical gripper
fully open for 0.5 seconds, and then plans directly to STOWED.

Run the deterministic physical acceptance with:

```zsh
./accept_cube_pick_drop.zsh --headless
./accept_cube_pick_drop.zsh --rviz
```

The acceptance performs the complete visual cube pick and CARRY sequence,
sends the simulated bin truth only as the Action input, and verifies from three
fresh MuJoCo truth frames that the released cube is stationary inside the bin.
Normal `d1-mujoco-sim` startup remains seeded-random; the acceptance command
selects the separate fixed layout for reproducibility.
