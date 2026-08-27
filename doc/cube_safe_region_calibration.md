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
