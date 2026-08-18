# D1 Workspace Analysis

Offline workspace-scanning tools for the D1 manipulation stack. This package is
not part of the robot's runtime pick/drop path.

The scanner evaluates one object type over an XY grid using the current MoveIt
model, fixed Go2/D1 assembly, ground collision model, observation search, grasp
search, lift, and return-to-carry checks. It writes one row per XY/yaw trial to
CSV.

## Build

```bash
colcon build --packages-select d1_manipulation d1_workspace_analysis --symlink-install
source install/setup.zsh
```

## Scan

```bash
ros2 launch d1_workspace_analysis grasp_workspace_scan.launch.py object_type:=yellow_cube output_csv:=/tmp/d1_grasp_workspace/yellow_cube_workspace.csv
```

`object_type` accepts `yellow_cube`, `zucchini`, or `bowl`. The launch arguments
`x_min_m`, `x_max_m`, `y_min_m`, `y_max_m`, and `xy_step_m` control the sampled
region in the D1 `base_link` frame.

## Render an artifact

```bash
python3 tools/workspace/generate_grasp_sweet_zone.py /tmp/d1_grasp_workspace/yellow_cube_workspace.csv --object-type yellow_cube --output-dir artifacts/cube_sweet_zone
```

The repository-level concise handoff is generated separately by
`tools/workspace/generate_grasp_sweet_zones_handoff.py`.
