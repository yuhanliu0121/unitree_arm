# D1-550 腕部 RealSense D435i 相机支架

[English](README.md)

本目录提供 Unitree D1-550 腕部 RealSense D435i 相机支架的打印文件、装配说明和当前安装对应的手眼外参。STL 的长度单位均为毫米。

## 设计与装配效果

<table>
  <thead>
    <tr>
      <th>项目</th>
      <th>模型</th>
      <th>实物</th>
      <th>相关文件</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>支架设计</td>
      <td><img src="images/design_model.png" alt="相机支架模型" width="360"></td>
      <td><img src="images/design_real.png" alt="相机支架实物" width="360"></td>
      <td><a href="stl/D1-550_main_stand.stl">主支架</a><br><a href="stl/D1-550_removable_piece_1.stl">可拆卸夹板 1</a><br><a href="stl/D1-550_removable_piece_2.stl">可拆卸夹板 2</a></td>
    </tr>
    <tr>
      <td>装配效果</td>
      <td><img src="images/assembly_model.png" alt="支架装配模型" width="360"></td>
      <td><img src="images/assembly_real.png" alt="支架装配实物" width="360"></td>
      <td>安装后若相机、支架或相对安装位置发生变化，必须重新进行手眼标定。</td>
    </tr>
    <tr>
      <td>RealSense D435i 视野</td>
      <td><img src="images/fov_model.png" alt="仿真相机视野" width="360"></td>
      <td><img src="images/fov_real.png" alt="真实相机 RGB-D 视野" width="360"></td>
      <td>支架应保证工作区域可见，并尽量减少夹爪对有效视野的遮挡。</td>
    </tr>
  </tbody>
</table>

## 3D 打印

打印以下三个文件：

- [`D1-550_main_stand.stl`](stl/D1-550_main_stand.stl)
- [`D1-550_removable_piece_1.stl`](stl/D1-550_removable_piece_1.stl)
- [`D1-550_removable_piece_2.stl`](stl/D1-550_removable_piece_2.stl)

已使用的打印参数如下：

| 参数 | 建议值 |
| --- | --- |
| 材料 | 普通 PLA；也可使用 PA-CF、PETG-CF 或 PETG |
| 层高 | 0.20 mm |
| 喷嘴 | 0.4 mm 或 0.6 mm |
| 壁数 | 至少 6 层 |
| 顶层/底层 | 6 层 |
| 填充 | 50%～65%，Gyroid 或 Cubic |

## 标准紧固件

<table>
  <thead>
    <tr>
      <th>规格</th>
      <th>数量</th>
      <th>用途</th>
      <th>图示</th>
      <th>备注</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>M3 六角螺母</td>
      <td>4</td>
      <td>从主支架盖板顶部装入预留槽</td>
      <td><img src="images/fasteners/m3_nut.png" alt="M3 六角螺母安装位置" width="280"></td>
      <td>—</td>
    </tr>
    <tr>
      <td>M3×10 内六角圆柱头螺钉</td>
      <td>4</td>
      <td>从夹板侧面穿过夹板，与主支架中的螺母连接</td>
      <td><img src="images/fasteners/m3x10.png" alt="M3×10 螺钉安装位置" width="280"></td>
      <td>—</td>
    </tr>
    <tr>
      <td>M3×8 内六角或盘头螺钉</td>
      <td>2</td>
      <td>将 D435i 固定到 6 mm 相机板</td>
      <td><img src="images/fasteners/m3x8.png" alt="M3×8 螺钉安装位置" width="280"></td>
      <td>与 1/4-20 UNC 相机螺钉二选一</td>
    </tr>
    <tr>
      <td>1/4-20 UNC 相机螺钉</td>
      <td>1（可选）</td>
      <td>通过底托 7 mm 通孔固定 D435i 或其他带三脚架孔的相机</td>
      <td><img src="images/fasteners/quarter20.png" alt="1/4-20 UNC 相机螺钉安装位置" width="280"></td>
      <td>与两颗 M3×8 螺钉二选一</td>
    </tr>
  </tbody>
</table>

## D435i RGB 光学坐标系到 Link6 的外参

以下外参对应本文所示的相机、支架和安装位置，并基于本仓库中校验后的 D1-550 URDF 标定。RGB optical frame 定义为 `+X` 向右、`+Y` 向下、`+Z` 向前。

```python
# p_link6 = T_link6_rgb @ p_rgb
# 平移单位：米
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

RGB 光学中心在 Link6 坐标系中的位置为：

- `X = -113.433 mm`
- `Y = +33.017 mm`
- `Z = -51.772 mm`

> **注意：**这里给出的是 `RGB optical frame → Link6` 的点变换。运行配置中的 `wrist_camera.link6_to_camera_link` 使用的是 RealSense 的 `wrist_camera_link`，不是 RGB optical frame。二者之间还包含 RealSense 驱动发布的内部静态 TF，因此不要直接复制本节四元数覆盖运行配置。相机或支架重新安装后，应重新标定并按实际 TF 链更新配置。
