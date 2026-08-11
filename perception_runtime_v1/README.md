# Go2 RGB-D 感知运行包 V1

这是机械狗/机械臂项目的冻结感知运行包。它不是单独的一份 YOLO 权重，而是完整执行下面链路：

`D435i RGB + 对齐深度 → YOLO11m-seg 实例分割 → mask 内部深度取样 → 三维尺寸/工作距离过滤 → 可视化和结构化结果`

模型仍然只使用 RGB 图像推理；深度图作为模型后的几何验证，不参与神经网络输入。这样既保留现有分割模型，也能过滤白墙、桌腿、线圈等尺寸明显不合理的误检。

## 最快启动（Windows）

1. 把 D435i 直接连接到 USB 3.x 接口。
2. 关闭 `realsense-viewer`、ROS 和其他占用相机的程序。
3. 双击 `run_visualization.cmd`。
4. 输入 `1` 选择机械狗模式，或输入 `2` 选择机械臂模式。

默认不固定相机序列号，会自动使用当前连接的 RealSense；只有同时连接多台相机时，才需要在 `configs/runtime.yaml` 中填写 `camera.serial`。

默认界面左侧是 RGB 分割结果，右侧是已经对齐到 RGB 的深度图。

快捷键：

- `Q` / `Esc`：退出
- `S`：保存当前 RGB、深度、可视化和 JSON 结果到 `captures/`
- `D`：切换“左右分屏 / RGB / 深度”
- `R`：显示或隐藏被过滤的候选
- `Space`：暂停/继续

## 结果颜色和状态

| 状态 | 含义 | 下游建议 |
|---|---|---|
| `depth_verified` | 类别、距离和物理尺寸均通过 | 可交给导航/抓取候选模块 |
| `rgb_only_far` | 远处小目标，RGB 识别到但深度不足 | 机械狗可保留，靠近后复核 |
| `rgb_only_near` | 近距大目标，可能进入 D435i 近距盲区 | 仅作为视觉提示，不直接生成精确抓取位姿 |
| `rejected` | 距离、尺寸或深度质量不合理 | 不交给动作模块；可在界面用红色查看原因 |

机械狗模式重视远距离召回，允许暂时保留远处无可靠深度的小目标；机械臂模式更保守，需要近距离的可靠几何信息。两套模式使用同一份模型权重，只改变后处理策略。

## 安装

本机已配置的启动脚本使用：

```text
C:\Users\breez\Documents\New project\.venv-realsense\Scripts\python.exe
```

在其他电脑上安装时：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

如果 Python 路径不同，修改 `run_visualization.cmd` 顶部的 `PYTHON_EXE`。

Linux 可执行 `chmod +x run_visualization.sh` 后运行：

```bash
./run_visualization.sh dog
./run_visualization.sh arm
```

## 团队调用接口

稳定入口是 `PerceptionRuntime.process(...)`：

```python
from perception_runtime import CameraIntrinsics, PerceptionRuntime

runtime = PerceptionRuntime("configs/runtime.yaml", profile="arm")
objects = runtime.process(color_bgr, aligned_depth_m, intrinsics)

usable = [obj for obj in objects if obj.accepted]
for obj in usable:
    print(obj.class_name, obj.status, obj.position_camera_m, obj.bearing_deg)
```

输入要求：

- `color_bgr`：OpenCV BGR 图像，`uint8`。
- `aligned_depth_m`：已经对齐到 RGB 的二维米制深度，`float32`。
- `intrinsics`：对齐后 RGB 图像对应的内参。

输出坐标是 RealSense 光学坐标系：`+X` 向图像右侧，`+Y` 向下，`+Z` 向前，单位为米。`position_camera_m` 尚未乘手眼标定矩阵，动作模块必须再把它变换到狗体/机械臂基坐标系。

详细字段见 [API.md](API.md)。

## 当前安全边界

- 本包只做感知与可视化，不发送机械狗或机械臂控制指令。
- 不做重复实例合并；两个堆叠的西葫芦会保留为两个对象。
- 不使用固定世界地面高度，因此相机装在狗头或机械臂上都可以运行。
- 不用固定“深度中值带”截点云；这避免斜放西葫芦被错误截短。
- 尺寸门控使用 mask 在稳健中位深度处的投影尺寸；点云 PCA 尺寸保留为诊断量，不让地面边缘飞点误杀真实目标。
- D435i 过近时深度可能失效。`rgb_only_near` 不能当作精确抓取位姿。
- 当前物理尺寸先验只适用于已训练的白碗、5 cm 黄方块和约 15 cm 西葫芦。

## 目录

```text
configs/                 相机、模型、类别尺寸和 dog/arm 配置
perception_runtime/      可复用 Python 包
tests/                   深度过滤回归测试
weights/best.pt          冻结发布权重
captures/                按 S 后生成的快照（运行时创建）
run_visualization.cmd    Windows 可视化入口
run_visualization.sh     Linux 可视化入口
```
