# Known Issues and Accuracy

## Observed FK Residual

In the six-pose independent holdout set, the difference between URDF FK and
the external-camera/AprilTag Link6 observation was:

- translation RMS: 8.673 mm;
- maximum observed translation: 15.758 mm;
- rotation RMS: 2.956 degrees;
- maximum observed rotation: 5.010 degrees.

These are combined measurement-chain residuals, not certified robot-only
errors or guaranteed global bounds. They include camera, tag detection,
mounting, backlash, joint feedback, and URDF errors and apply only to the
sampled workspace. For planning, allow approximately 20 mm TCP tolerance
unless visual or contact-based final correction is available.

## Calibration Decision

Zero-offset and tightly bounded axis-tilt candidates improved reprojection in
the fit camera but did not improve translation and rotation consistently in a
separate 42-pose session. They were rejected. This release intentionally keeps
the mechanically continuous nominal joint centres and axes, avoiding the
Joint4/Joint5 mesh separation seen in the invalid unconstrained model.

## Remaining Limitations

- The package contains a robot description, RViz launch, and a read-only D1
  feedback bridge. It does not contain MoveIt, an IK solver, trajectory
  execution, or any D1 command publisher.
- Building the bridge requires system installations of `unitree_sdk2` and
  Cyclone DDS/C++; these third-party libraries are not bundled.
- The camera hand-eye transform is device- and mount-specific and is not
  included.
- Dynamics, effort, and velocity values have not been independently verified.
- Joint limits and directions were hardware-tested, but limits are not
  certified safety limits.
- The gripper requires the SDK-angle mapping in
  `config/gripper_ratio.yaml`; SDK J6 is not a URDF prismatic displacement.
  The corrected nominal visualization is 0 mm aperture at `Joint6=0` and
  60 mm aperture at `Joint6=0.03 m`; this aperture was derived from the
  supplied meshes rather than independently measured with metrology.
- `tcp_link` is the nominal centre of the two STL finger-tip faces. Its
  location has not been independently measured on the physical fingertips.
