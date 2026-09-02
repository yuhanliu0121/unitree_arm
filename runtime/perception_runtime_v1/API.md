# 感知接口约定

## 主接口

```python
objects = runtime.process(color_bgr, aligned_depth_m, intrinsics)
```

每个 `Detection3D` 是一个独立实例，常用字段如下：

| 字段 | 类型 | 说明 |
|---|---|---|
| `instance_id` | `int` | 当前帧中的实例编号 |
| `class_name` | `str` | `bowl` / `yellow_cube` / `zucchini` |
| `confidence` | `float` | RGB 分割置信度 |
| `accepted` | `bool` | 是否通过当前 profile 的感知门控 |
| `status` | `str` | `depth_verified` / `rgb_only_far` / `rgb_only_near` / `rejected` |
| `reject_reasons` | `tuple[str]` | 被过滤原因 |
| `mask` | `H×W bool` | 实例分割遮罩 |
| `center_pixel` | `(u,v)` | 深度核心区域中位像素 |
| `depth_m` | `float?` | mask 核心区域的稳健中位深度 |
| `position_camera_m` | `(X,Y,Z)?` | 相机光学坐标下三维中心 |
| `dimensions_m` | `(long,mid,short)?` | 点云 PCA 稳健尺寸 |
| `longest_size_m` | `float?` | 原始目标点云 PCA 最大边，仅供诊断/后续姿态计算 |
| `apparent_size_m` | `float?` | mask 在中位深度处的投影最大边；当前尺寸过滤使用此值 |
| `bearing_deg` | `float?` | 目标相对相机前方的水平角，右正左负 |
| `point_cloud_camera_m` | `N×3?` | 已清理和下采样的目标点云 |

调用方只需要导航发现目标时，可使用 `accepted`。要进入机械臂抓取位姿计算，建议进一步要求：

```python
obj.status == "depth_verified" and obj.position_camera_m is not None
```

## 被过滤原因

- `mask_too_small`：mask 面积低于当前模式阈值。
- `outside_work_range`：目标深度不在当前 dog/arm 工作范围。
- `size_out_of_range`：点云物理尺寸与该类别明显不符。
- `too_large_for_distance`：深度稀疏，但图像表观尺寸已经不可能属于该目标。
- `no_valid_depth_and_small_mask`：机械臂模式下小目标没有有效深度。
- `depth_quality_too_low`：有少量深度，但不足以确认三维尺寸。

## 坐标变换责任

本包输出 `camera_optical` 坐标，不内置任何一台机械臂的手眼标定结果。集成人负责提供时间同步的外参 `T_base_camera`：

```text
p_base = T_base_camera × p_camera
```

只有完成这一变换、IK 可达性和碰撞检查后，才可以把候选交给机械臂控制器。
