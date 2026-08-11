# Simulation perception domain-gap validation

This is an offline diagnostic. It does not modify or participate in the grasp
runtime. It renders the current MuJoCo scene from six controlled wrist-camera
views, obtains exact masks from MuJoCo segmentation, runs the frozen
`perception_runtime_v1` YOLO segmenter, and compares its result with scene
truth.

The six cases are the Cartesian product of:

- `yellow_cube`, `bowl`, and `zucchini`
- an oblique view (65 degrees above the ground, 0.60 m away) and a top-down
  view (0.45 m away)

Run from the repository root:

```zsh
./validate_sim_perception.zsh
```

Results are written to `validation/sim_perception_domain/artifacts/latest/`:

- `images/*_rgb.png`: undistorted MuJoCo RGB passed through the calibrated
  D435i distortion model
- `images/*_gt.png`: exact target mask from MuJoCo segmentation
- `images/*_prediction.png`: the matched model mask, if detected
- `images/*_overlay.png`: cyan GT boundary and magenta prediction boundary
- `report.json`: machine-readable detections and metrics
- `report.md`: compact human-readable summary

The first pass intentionally keeps the released model, confidence/NMS/mask
thresholds, object materials, and scene lighting unchanged. No pass/fail
threshold is imposed: the report exposes class detection, confidence, mask IoU,
and mask-centroid error for review.

The camera is placed independently of arm IK and robot visual geometry is
excluded for this asset-domain test. The ground, lighting, outlines, object
meshes, materials, and object poses remain unchanged. The test therefore does
not validate arm reachability or occlusion along a planned observation
trajectory.
