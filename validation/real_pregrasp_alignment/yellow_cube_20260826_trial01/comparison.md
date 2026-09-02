# Yellow Cube PREGRASP 视觉对准实验报告（Trial 01）

## 1. 实验结论

人工调整立方体后，本次 DESCEND 物理验收通过：

- 两根指尖分别位于立方体两侧；
- 下降过程中没有接触或推动立方体；
- 如果在最终位置闭合夹爪，可以可靠夹住立方体；
- 程序成功运动到配置的 `-40 mm` 抓取深度，并保持夹爪完全张开。

数据表明，本轮失败位置与成功位置之间的主要区别是 **PREGRASP RGB
图像中的二维横向偏差**，不是物体高度、朝向或机械臂漂移。因此，当前最值得
尝试的最小方案是在 PREGRASP 阶段增加一次小范围、走走停停式的图像平面纠偏。

## 2. 三个关键时刻的 RGB 图像

### 2.1 人工调整前的 PREGRASP

洋红色十字为通过颜色分割离线计算出的立方体像素中心。

![人工调整前的 PREGRASP](before_adjustment/extracted/rgb_annotated.png)

### 2.2 人工调整后的正确 PREGRASP

机械臂保持不动，仅调整立方体位置。该位置随后通过了真实 DESCEND 验收。

![人工调整后的正确 PREGRASP](after_adjustment/extracted/rgb_annotated.png)

### 2.3 DESCEND 完成、夹爪保持张开

此时两根手指已分别位于立方体两侧，未接触或推动立方体。

![DESCEND 完成](after_descend/extracted/rgb_annotated.png)

## 3. PREGRASP 调整前后量化对比

| 指标 | 调整前 | 成功位置 | 变化量 |
|---|---:|---:|---:|
| RGB mask 中心 `u` | 786.31 px | 740.41 px | -45.90 px |
| RGB mask 中心 `v` | 448.61 px | 440.16 px | -8.45 px |
| mask 面积 | 34,307 px | 33,030 px | -3.72% |
| 最小外接矩形角度 | 2.60° | 3.37° | +0.76° |
| 对齐深度中位数 | 273 mm | 272 mm | -1 mm |
| TCP 模型坐标 `x` | 215.218 mm | 215.360 mm | +0.142 mm |
| TCP 模型坐标 `y` | 125.510 mm | 125.985 mm | +0.475 mm |
| TCP 模型坐标 `z` | 136.328 mm | 136.335 mm | +0.007 mm |

相机内参为 `fx=908.86`、`fy=908.51`。按照 PREGRASP 深度 `z=272 mm`
近似换算，本轮人工调整对应 RGB 相机图像平面内约：

- 水平方向：`-13.7 mm`；
- 竖直方向：`-2.5 mm`。

这个结果只是基于针孔模型的相机平面估计，不是真实 `base_link` 坐标系下的
位移测量。

两次 PREGRASP 记录中，机械臂反馈基本没有变化：只有 Joint4 出现约 `-0.1°`
的量化级差异，TCP 模型坐标变化不超过 `0.5 mm`。因此可以认为对比期间机械臂
保持静止，图像变化来自人工移动立方体。

## 4. 深度图对比

深度伪色固定显示范围为 `0.20–2.00 m`。

| 调整前 PREGRASP | 成功位置 PREGRASP | DESCEND 后 |
|---|---|---|
| ![调整前深度](before_adjustment/extracted/depth_aligned_plasma.png) | ![成功位置深度](after_adjustment/extracted/depth_aligned_plasma.png) | ![DESCEND 后深度](after_descend/extracted/depth_aligned_plasma.png) |

PREGRASP 阶段，立方体区域的对齐深度中位数约为 `272–273 mm`，深度数据稳定。
DESCEND 后深度中位数降至约 `205 mm`，已经接近配置的深度有效下界，而且有效
深度像素数量明显减少。因此，精细视觉纠偏应在 PREGRASP 完成，不宜依赖
DESCEND 阶段的深度图继续修正。

## 5. 当前最小可行纠偏方案

本轮成功 PREGRASP 中，立方体 RGB mask 中心约为 `(740, 440) px`。可以先将其
作为实验参考值：

1. 机械臂运动到名义 PREGRASP；
2. 重新获取 RGB mask 与对齐深度；
3. 计算当前 mask 中心相对实验参考像素的偏差；
4. 利用当前深度和相机内参，将像素偏差换算为一次小幅相机平面位移；
5. 执行修正、等待机械臂稳定并重新观测；
6. 误差满足阈值后，使用 `common_arrival` 执行 DESCEND；
7. 若一到两次修正后仍不满足阈值，则中止并要求 Go2 调整位置。

这套方案暂时不需要指尖识别、在线视觉 Jacobian 估计或连续流式视觉伺服。
不过本报告只有一个成功样本，不能直接把 `(740, 440) px` 作为最终标定结果。
建议再采集 2–3 组人工对准且 DESCEND 验收成功的数据，估计目标像素的均值和
波动范围后，再确定纠偏目标与通过阈值。

## 6. 数据位置

- 调整前：`before_adjustment/`
- 人工调整后：`after_adjustment/`
- DESCEND 后：`after_descend/`
- 每组目录均保留压缩 ROS bag、提取后的 RGB/深度 PNG、关节反馈、TF、IMU
  和 `summary.json`。
