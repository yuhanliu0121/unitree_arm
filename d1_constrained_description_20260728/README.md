# D1 Mechanically Constrained Description

This ROS 2 package is the distributable 2026-07-28 Unitree D1 arm-with-gripper
description. It retains the mechanically continuous model after constrained
AprilTag calibration failed to justify a nonzero kinematic correction on an
independent dataset.

## Model Conventions

- `Joint0` through `Joint5` correspond to SDK `angle0` through `angle5`.
- SDK arm angles are degrees; URDF joint positions are radians.
- Joint origins, link translations, and mesh placement remain fixed to the
  mechanically coherent source model.
- Joint directions and limits include the earlier real-hardware corrections.
- `Joint6` and `Joint6_mimic` describe finger travel in metres. Convert the
  SDK gripper motor angle using `config/gripper_ratio.yaml`.
- The vendor finger origins are corrected so `Joint6=0` is nominally closed
  and `Joint6=0.03 m` gives a nominal 60 mm jaw aperture.
- `tcp_link` is fixed at the midpoint of the two finger-tip faces:
  `[0.00038, 0, 0.1256] m` in `Link6`. Its `+Z` axis points from the wrist
  toward the grasp point.
- The visual robot model includes the `main_stand` wrist-camera mount in
  `Link6` coordinates and the D435i CAD at the calibrated RGB optical pose.
  Each accessory uses its mesh bounding box for collision checking.
- The distributable URDF retains the vendor STL geometry. The MuJoCo builder
  and `d1_moveit_config` independently substitute the same validated
  box/cylinder collision approximation, avoiding detailed-mesh assembly
  overlap without changing the shared kinematic frames.

## Build and Inspect

Place this directory in a ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select d1_constrained_description
source install/setup.bash
ros2 launch d1_constrained_description display.launch.py
```

The display launch does not connect to or command D1 hardware.

With either launch running, inspect the live flange and grasp-point FK:

```bash
ros2 run tf2_ros tf2_echo base_link Link6
ros2 run tf2_ros tf2_echo base_link tcp_link
```

For live, read-only visualization from a D1 connected on `eno1`:

```bash
ros2 launch d1_constrained_description live_display.launch.py
```

The included bridge only subscribes to Unitree DDS joint feedback and
publishes ROS 2 `/joint_states`; it contains no DDS command publisher.
Override the interface when necessary:

```bash
ros2 launch d1_constrained_description live_display.launch.py interface:=eno2
```

Building the live bridge requires `unitree_sdk2` and Cyclone DDS/C++ installed
under the normal system include and library locations.

## Calibration Decision

Four models were compared on 25 fit poses, six untouched holdout poses, three
fresh holdout poses, and a separate 42-pose session. Zero offsets and bounded
axis tilts improved the original camera's pixel residuals but failed to improve
the independent session consistently. No correction was therefore promoted.

See `calibration/constrained_calibration_summary.json` for the numerical
decision record. This is the safest current development model, not a metrology
certificate. Absolute FK accuracy remains uncertain at the millimetre and
degree level, and the camera hand-eye transform is not embedded.

Mass, centre-of-mass, inertia, effort, and velocity values originate from
manufacturer files or documentation and have not been independently verified.

See `USAGE_CN.md` for the complete Chinese build and two-mode validation
tutorial, `QUICKSTART.md` for compact copy-paste commands, and
`KNOWN_ISSUES.md` for the measured error envelope and remaining limitations.
