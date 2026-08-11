# Go2 机械臂作业趴姿交接

## 交接目标

Go2 到达垃圾附近后，运控先进入并稳定保持本文件描述的趴姿。达到双方约定
的稳定判据后，才调用机械臂的 `pick_object` 或 `drop_object`。机械臂任务执行
期间，Go2 不主动改变机身姿态或平面位置。

机器可读参数见
[`go2_manipulation_posture.yaml`](go2_manipulation_posture.yaml)。

## 名义关节姿态

该姿态直接取自 Unitree 官方 `unitree_mujoco` 的 sim-to-real 低层控制示例
`stand_down_joint_pos`，单位为弧度。

SDK `LowCmd.motor_cmd[0:12]` 的顺序为：

```text
FR_hip, FR_thigh, FR_calf,
FL_hip, FL_thigh, FL_calf,
RR_hip, RR_thigh, RR_calf,
RL_hip, RL_thigh, RL_calf
```

对应目标值：

```text
[ 0.0473455, 1.22187, -2.44375,
 -0.0473455, 1.22187, -2.44375,
  0.0473455, 1.22187, -2.44375,
 -0.0473455, 1.22187, -2.44375 ]
```

这些数值均位于当前保存的官方 Go2 URDF 关节限位内。来源固定在
`unitree_mujoco` 提交 `ae6a8403e272733e9996ef59990880330496177f`。

## 名义几何关系

坐标轴采用 ROS 约定：`+X` 向前、`+Y` 向左、`+Z` 向上。

当前仿真基准为：

```text
ground -> go2_base:
  xyz = [0, 0, 0.166287] m
  rpy = [0, 0, 0]

go2_base -> D1 base_link:
  xyz = [0, 0, 0.057961769402] m
  rpy = [0, 0, 0]

ground -> D1 base_link:
  xyz = [0, 0, 0.224248769402] m
  rpy = [0, 0, 0]
```

Blender 中趴姿可视 mesh 的最低点为 `z=-0.166287392 m`；最低点上方
`1 mm` 范围内的顶点覆盖四足区域，说明该可视姿态不是靠机身单点悬空。

## 必须由运控真机确认的内容

官方 `stand_down_joint_pos` 只能作为名义关节目标，不能直接证明安装 D1 后
的真机一定稳定。运控需要在安装机械臂及实际负载后确认：

- 姿态进入和退出轨迹；
- 足端或其他允许部位的接触状态；
- 机身高度、roll、pitch 的允许误差；
- 机身线速度、角速度的稳定阈值；
- 稳定状态必须连续保持的时间；
- 抓取期间抵抗机械臂运动反作用力的能力。

这些阈值确认后，填写配置文件中的 `real_robot_acceptance`。机械臂侧把该
稳定信号作为任务前置条件，同时继续读取实际 IMU 重力方向修正残余倾斜，
不会假定真机姿态与名义值完全一致。

## 仿真模型边界

当前 MuJoCo 没有模拟 Go2 的 12 关节动力学。Go2 被表示为：

- 官方趴姿的完整可视 mesh；
- 覆盖整个 mesh 的单个 axis-aligned bounding box；
- 可在 `x/y` 平移、姿态固定的刚体平台。

因此当前仿真可以验证机械臂安装高度、视野、工作空间和平台避碰，但不能
验证 Go2 趴姿控制器、足端受力分配或机械臂运动引起的机身稳定性。
