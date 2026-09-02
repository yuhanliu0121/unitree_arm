# 立方体安全下降区域：四边界标定

本工具在机械臂已经停在 PREGRASP 后，采集立方体顶面中心相对于腕部 RGB
光学坐标系的位置。它只读取传感器和状态、调用现有精观测服务，不会向机械臂
下发任何运动命令。

## 四次摆放

保持机械臂姿态不变，只移动立方体：

1. 左侧无余量，抓指下半部分有效贴合；
2. 左侧无余量，抓指上半部分有效贴合；
3. 右侧无余量，抓指下半部分有效贴合；
4. 右侧无余量，抓指上半部分有效贴合。

“临界位置”采用已经确认的物理判据：张开手指下降时不推动立方体，且相关抓指
至少约一半能够贴到立方体侧面。这四点对应安全柱体横截面的四个物理角点；同一
边界的两次测量会先取平均，再生成矩形上下界。

## 使用方法

先启动真机控制栈并让机械臂停在本轮标定使用的 PREGRASP。每次摆好并人工确认
临界位置后，在新终端执行对应的一条命令：

```bash
./capture_cube_safe_region.zsh 1
./capture_cube_safe_region.zsh 2
./capture_cube_safe_region.zsh 3
./capture_cube_safe_region.zsh 4
```

如果上次存在未完成的会话，而本次确实要废弃它并重新开始：

```bash
./capture_cube_safe_region.zsh 1 --new
```

工具拒绝以下样本：RGB-D、IMU、TF 或关节反馈缺失，RGB-D 与几何估计时间戳
不匹配，精观测没有得到四个顶面角点，或最近一秒机械臂运动范围超过 0.5°。

## 输出

数据保存在：

```text
validation/real_pregrasp_alignment/safe_region_calibration/
  cube_safe_region_YYYYMMDD_HHMMSS/
    1_left_lower/
    2_left_upper/
    3_right_lower/
    4_right_upper/
    session.json
    safe_region.json
```

每个边界目录包括：

- `rgb.png`；
- `aligned_depth.png`（原始 16UC1 毫米深度）；
- `aligned_depth_plasma.png`；
- 可用时的 `raw_depth.png`；
- YOLO/立方体几何调试图；
- `sample.json`，含顶面中心、四角点、重力、夹爪三轴、关节、IMU、相机内参和
  时间同步诊断。

第四次采集后生成 `safe_region.json`。其中：

- `raw_bounds_m` 是四次实测得到的原始边界；
- `safe_bounds_m` 是向内收缩后的边界；
- `safe_centre_line` 是运行时优先对准的中心线；
- 柱体沿相机坐标系中的重力方向延伸。

当前标定使用四个已通过张开夹爪 DESCEND 验收的物理边界点，
因此两个方向的内缩量均为 0 mm，结果标记为 `physically_validated`。
运行时默认启用，但仍要求相机坐标系与标定一致；同一套 D1、腕部相机安装
几何和 URDF 可共用该标定。这一区域表示
已验证的 PREGRASP 静态对准边界，不代替对 DESCEND 随机横向扫动的安全余量评估。

## Closing 边界重复标定

当原四点标定得到的 closing 范围过窄时，使用独立的重复标定流程。它保留当前
运行时 closing 轴以及已有 finger 方向安全范围，只重新测量 closing 两侧边界。

每次均先在同一个 PREGRASP 姿态摆放立方体并采集，然后执行张爪 DESCEND 做
物理验收。编号 1～3 是同一侧无余量边界的三次有效重复，编号 4～6 是另一侧：

```bash
./capture_cube_closing_recalibration.zsh 1 --new
./capture_cube_closing_recalibration.zsh 2
./capture_cube_closing_recalibration.zsh 3
./capture_cube_closing_recalibration.zsh 4
./capture_cube_closing_recalibration.zsh 5
./capture_cube_closing_recalibration.zsh 6
```

若某次 DESCEND 碰到或推动立方体，该样本无效。恢复 PREGRASP、重新摆放后，
替换相同编号，例如：

```bash
./capture_cube_closing_recalibration.zsh 3 --replace
```

固定 PREGRASP 已从最近一次真机日志中提取，并用首次标定采集时的静止反馈更新，
Joint0～5 为 `[-97.5°, 67.1°, -10.0°, 10.8°, 35.6°, -106.8°]`。因此开始
标定以及每次张爪 DESCEND 验收结束后，都可以在保持夹爪完全张开的前提下直接
进入该姿态，无需先回 STOWED：

本轮第一个 DESCEND 仍需由一次 `pick_object(stop_after=1)` 生成并缓存，因为
历史上已成功 DESCEND 的日志属于另一个 yaw/关节构型，不能安全地拼接到这里。
第一个 DESCEND 验收成功并记录终点后，余下五轮只在固定 PREGRASP 与固定
DESCEND 之间往返，不再经过 STOWED。

```bash
./return_cube_closing_calibration_pregrasp.zsh \
  --record-current-as-descend \
  --confirm CUBE_CALIBRATION_MOVE
```

`--record-current-as-descend` 只在第一个样本确认下降有效后使用一次。它先记录这次
实际下降终点，再返回 PREGRASP。后续每个样本用固定终点下降：

```bash
./descend_cube_closing_calibration.zsh \
  --confirm CUBE_CALIBRATION_DESCEND
```

下降验收完成后再用不带记录参数的返回命令：

```bash
./return_cube_closing_calibration_pregrasp.zsh \
  --confirm CUBE_CALIBRATION_MOVE
```

该命令只允许从参考姿态 30° 范围内返回，且要求夹爪处于安全张开状态；它不会
经过 STOWED，也不会重新计算另一个 PREGRASP，因此六次采集的相机和关节姿态
保持一致。

第六个有效样本完成后生成 `closing_recalibration.json`。两侧边界分别取三次
投影坐标的中位数，并默认各向内部收缩 1 mm。文件会列出六个原始坐标、两侧
极差、新的安全宽度，以及建议写入 `observe_target.yaml` 的参数；结果不会自动
覆盖运行时配置，必须在最后一次 DESCEND 也通过后再人工启用。

2026-09-01 的重复标定得到 closing 原始跨度 8.87 mm；每侧内缩 1 mm 后，
运行安全范围为 `[-43.86, -36.99] mm`，宽度 6.87 mm。六个参与计算的样本均
已通过固定 DESCEND 物理验收。
