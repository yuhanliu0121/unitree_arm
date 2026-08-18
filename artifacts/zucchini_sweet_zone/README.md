# Zucchini sweet-zone coarse scan

- XY grid: `-0.70..0.70 m` x `-0.70..0.70 m`, `0.05 m` step.
- Orientation samples: `0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170 deg`.
- Conservative points: `75/841`.
- Recommended coarse-grid point: `(0.10, -0.30) m`.
- Grid-derived XY robustness radius: `0.10 m`.

This is an offline candidate map using calibrated camera extrinsics, observation endpoint IK/collision checks, the current MoveIt collision model, ground, Go2 proxy, object-specific pregrasp/descent/lift geometry and carried-object OMPL planning to CARRY. Observation trajectories are OMPL-validated only at the later physical spot checks. It does not by itself prove RGB/depth perception or MuJoCo contact success at every grid point.

Failure counts by stage:

- `PLAN_TOP_OBSERVE`: 7488
- `PREGRASP_DESCENT_OR_CARRY`: 3221
- `TARGET_OVERLAPS_GO2_FOOTPRINT`: 2916
