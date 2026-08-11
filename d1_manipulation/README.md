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
