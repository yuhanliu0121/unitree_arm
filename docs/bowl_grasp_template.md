# Bowl grasp template

The accepted Blender annotation is stored in:

```text
simulation/objects/bowl/bowl_grasp_annotation.blend
```

Select and move only `GRIPPER_TCP_POSE`. Its world transform is the annotated
URDF `tcp_link` pose. The bowl object remains at the model origin and the
fingers remain at `Joint6=0.03 m` (nominal 60 mm opening).

The runtime source of truth is:

```text
d1_manipulation/config/grasp_templates.yaml
```

Runtime perception constructs `world_T_rim` from the fitted bowl-rim centre
and plane normal. The configured canonical pose is expanded as:

```text
world_T_tcp = world_T_rim
            * Rz_rim(theta)
            * rim_T_tcp_canonical
            * Rz_tcp(flip)
```

`theta` moves the complete grasp around the bowl rim. `flip=180 deg` leaves
the TCP position and approach direction unchanged while exchanging the inner
and outer physical fingers. The configured ordered search contains 24 rim
locations and two finger assignments, producing 48 candidates.

Generate and inspect the exact matrices with:

```bash
python3 tools/grasp_templates/generate_bowl_candidates.py
python3 tools/grasp_templates/generate_bowl_candidates.py --json
```

Pass a detected rim pose as `x y z qx qy qz qw` with:

```bash
python3 tools/grasp_templates/generate_bowl_candidates.py --world-from-rim 0.4 0.1 0.05 0 0 0 1 --json
```

The utility is a reference implementation. Production code should load the
same YAML and preserve its multiplication order and candidate ordering.
