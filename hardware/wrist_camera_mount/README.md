# D1-550 Wrist-Mounted RealSense D435i Bracket

[中文说明](README_CN.md)

This directory contains printable parts, assembly guidance, and the hand-eye calibration associated with the shown RealSense D435i installation on a Unitree D1-550. All STL dimensions are in millimetres.

## Design and assembly

<table>
  <thead>
    <tr>
      <th>Item</th>
      <th>Model</th>
      <th>Physical installation</th>
      <th>Files or notes</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Bracket design</td>
      <td><img src="images/design_model.png" alt="Camera bracket model" width="360"></td>
      <td><img src="images/design_real.png" alt="Printed camera bracket" width="360"></td>
      <td><a href="stl/D1-550_main_stand.stl">Main stand</a><br><a href="stl/D1-550_removable_piece_1.stl">Removable clamp 1</a><br><a href="stl/D1-550_removable_piece_2.stl">Removable clamp 2</a></td>
    </tr>
    <tr>
      <td>Assembly</td>
      <td><img src="images/assembly_model.png" alt="Bracket assembly model" width="360"></td>
      <td><img src="images/assembly_real.png" alt="Physical bracket assembly" width="360"></td>
      <td>Recalibrate the hand-eye transform whenever the camera, bracket, or their relative mounting position changes.</td>
    </tr>
    <tr>
      <td>RealSense D435i field of view</td>
      <td><img src="images/fov_model.png" alt="Simulated camera field of view" width="360"></td>
      <td><img src="images/fov_real.png" alt="Physical RGB-D camera view" width="360"></td>
      <td>The working area must remain visible with minimal occlusion from the gripper.</td>
    </tr>
  </tbody>
</table>

## 3D printing

Print all three parts:

- [`D1-550_main_stand.stl`](stl/D1-550_main_stand.stl)
- [`D1-550_removable_piece_1.stl`](stl/D1-550_removable_piece_1.stl)
- [`D1-550_removable_piece_2.stl`](stl/D1-550_removable_piece_2.stl)

The following settings were used for the validated print:

| Parameter | Recommended value |
| --- | --- |
| Material | Standard PLA; PA-CF, PETG-CF, or PETG may also be used |
| Layer height | 0.20 mm |
| Nozzle | 0.4 mm or 0.6 mm |
| Walls | At least 6 |
| Top/bottom layers | 6 |
| Infill | 50%–65%, Gyroid or Cubic |

## Standard fasteners

<table>
  <thead>
    <tr>
      <th>Specification</th>
      <th>Quantity</th>
      <th>Purpose</th>
      <th>Illustration</th>
      <th>Notes</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>M3 hex nut</td>
      <td>4</td>
      <td>Insert into the slots from the top of the main bracket cover</td>
      <td><img src="images/fasteners/m3_nut.png" alt="M3 hex nut locations" width="280"></td>
      <td>—</td>
    </tr>
    <tr>
      <td>M3×10 socket-head cap screw</td>
      <td>4</td>
      <td>Pass through the side clamps and engage the nuts in the main bracket</td>
      <td><img src="images/fasteners/m3x10.png" alt="M3×10 screw locations" width="280"></td>
      <td>—</td>
    </tr>
    <tr>
      <td>M3×8 socket-head or pan-head screw</td>
      <td>2</td>
      <td>Attach the D435i to the 6 mm camera plate</td>
      <td><img src="images/fasteners/m3x8.png" alt="M3×8 screw locations" width="280"></td>
      <td>Use this method or one 1/4-20 UNC camera screw</td>
    </tr>
    <tr>
      <td>1/4-20 UNC camera screw</td>
      <td>1 (optional)</td>
      <td>Attach a D435i or another tripod-mount camera through the 7 mm base opening</td>
      <td><img src="images/fasteners/quarter20.png" alt="1/4-20 UNC camera screw location" width="280"></td>
      <td>Use this method or two M3×8 screws</td>
    </tr>
  </tbody>
</table>

## D435i RGB optical frame to Link6 extrinsic

The following calibration applies to the camera, bracket, and installation shown above and was obtained using the corrected D1-550 URDF in this repository. The RGB optical frame uses `+X` right, `+Y` down, and `+Z` forward.

```python
# p_link6 = T_link6_rgb @ p_rgb
# Translation unit: metre
T_link6_rgb = np.array([
    [ 0.007802511223,  0.969491460227,  0.245000876253, -0.113433495391],
    [-0.999491412298, -0.000014772168,  0.031889128609,  0.033016808957],
    [ 0.030919857055, -0.245125087105,  0.968998273534, -0.051772117970],
    [ 0.000000000000,  0.000000000000,  0.000000000000,  1.000000000000],
], dtype=np.float64)

quaternion_link6_rgb_xyzw = np.array([
    -0.098512702045,
     0.076132192723,
    -0.700216133504,
     0.702991111713,
], dtype=np.float64)
```

The RGB optical centre expressed in the Link6 frame is:

- `X = -113.433 mm`
- `Y = +33.017 mm`
- `Z = -51.772 mm`

> **Important:** This section gives the `RGB optical frame → Link6` point transform. The runtime field `wrist_camera.link6_to_camera_link` refers to the RealSense `wrist_camera_link`, not its RGB optical frame. The RealSense driver publishes an additional internal static transform between those frames. Do not paste this quaternion directly over the runtime setting. Recalibrate and update the actual TF chain whenever the camera or bracket is reinstalled.
