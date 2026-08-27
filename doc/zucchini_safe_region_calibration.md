# 西葫芦安全下降夹层：双边界标定

本工具在机械臂停于 zucchini PREGRASP 后，采集西葫芦轴线中点相对于腕部
RGB 光学坐标系的三维位置。工具只读取 RGB-D、IMU、TF 和关节反馈，不会驱动
机械臂。

## 标定对象与坐标

正常抓取姿态下，两指连线垂直于西葫芦长轴：

- `closing`：两指闭合方向；
- `finger_length`：沿两根手指长度方向，也近似平行于西葫芦长轴；
- 安全区域沿重力方向延伸，因此 PREGRASP 高度变化不会改变横向边界定义。

标定所跟踪的点是感知得到的西葫芦中轴线中点，并投影到配置的名义中心高度
平面。长轴本身用于构造抓取 yaw，不用其正负号定义安全区。

## 两个物理边界

保持同一次 PREGRASP 姿态，只沿夹爪闭合方向移动西葫芦，依次测量：

1. 闭合方向 A 侧临界；
2. 闭合方向 B 侧临界。

这里的 A/B 只表示两个物理侧面。闭合轴的正负号取决于相机坐标约定，生成安全
夹层时会把两个投影坐标排序为 `closing_min`/`closing_max`，不依赖采集顺序。

每个“临界”位置都必须满足：张开夹爪 DESCEND 时不接触或推动物体，并且若
随后闭合，两指能够夹住直径足够、不会从端部滑脱的实体部分。西葫芦端部若明显
变细，应把临界位置放在仍能可靠夹持的位置，而不是几何 mask 的最远端。

## 调用

每次摆好一个边界并确认机械臂完全静止后执行：

```bash
./capture_zucchini_safe_region.zsh 1
./capture_zucchini_safe_region.zsh 2
```

首轮 PREGRASP 的实测六轴反馈已记录在
`d1_manipulation/config/zucchini_calibration_pregrasp.yaml`。一次张爪 DESCEND
通过物理验收后，无须返回 STOWED；保持夹爪完全张开、移走手和其他障碍物，执行：

```bash
./return_zucchini_calibration_pregrasp.zsh --confirm ZUCCHINI_CALIBRATION_MOVE
```

该命令只用于从本次标定的 DESCEND 附近返回固定 PREGRASP。若任一机械臂关节
距离记录姿态超过 30°，或夹爪开口不足 25 mm，命令会拒绝执行；它不能替代通用
的 STOWED 恢复工具。

废弃未完成会话并重做第一个点时使用：

```bash
./capture_zucchini_safe_region.zsh 1 --new
```

数据输出到：

```text
validation/real_pregrasp_alignment/zucchini_safe_region_calibration/
  zucchini_safe_region_YYYYMMDD_HHMMSS/
```

第二点采集后生成 `safe_region.json`。区域默认不额外内缩；两个样本必须分别经
过张开夹爪 DESCEND 物理验收，失败样本应拒绝并重新采集。
