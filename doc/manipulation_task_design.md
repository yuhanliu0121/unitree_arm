# D1 机械臂 Pick / Drop 任务设计（草案）

> 状态：黄色立方体视觉抓取、CARRY 运动及腕部 RGB 复检已实现并通过自动验收
>
> 更新时间：2026-08-11
> 本文描述 Go2 与 D1 机械臂子系统之间的业务接口、任务状态、失败分类、
> 异常恢复策略，以及下一阶段的实现与验收内容。

## 1. 系统边界

Go2 只调用机械臂子系统的高层任务接口，不直接控制机械臂关节、夹爪、
相机或 MoveIt。

机械臂子系统内部负责：

- 目标坐标转换；
- eye-in-hand 相机主动观测；
- RGB-D 目标检测和位姿估计；
- IK、碰撞检测和轨迹规划；
- ros2_control 轨迹执行；
- 夹爪控制；
- 抓取、携带、投放和异常恢复状态管理。

Go2 在 Pick 或 Drop 执行期间保持静止。GT `x/y` 导航不属于本任务接口。

调用任务前，Go2 运控进入并稳定保持统一的机械臂作业趴姿。名义关节角、
基座高度、安装关系和待真机确认的稳定判据记录在
`calibration/go2_d1_mount/go2_manipulation_posture.yaml`。机械臂侧要求收到
运控的姿态稳定状态后才接受任务，并继续使用实时 IMU 重力方向补偿名义
趴姿的残余 roll/pitch 误差。

## 2. 正式对外接口

机械臂向 Go2 正式暴露两个 ROS 2 Action：

```text
/arm/tasks/pick_object
/arm/tasks/drop_object
```

两个接口的目标位置都使用：

```text
geometry_msgs/PointStamped target
```

`frame_id` 必须指向 TF 树中可转换到机械臂规划坐标系的坐标系。仿真阶段
使用 `go2_base`；真机阶段允许使用 Go2 机身相机坐标系，但必须先具备对应
相机外参和 TF。

观测几何不假定 `base_link` 的 Z 轴竖直，也不假定真机天然发布名为
`gravity_frame`、`odom` 或 `map` 的 TF。仿真从 `world` 获取重力方向；真机
从 Go2 IMU 姿态计算重力方向。若真机导航系统另行提供经过验证的重力对齐
TF，可通过配置切换使用该 TF。

真机自研运控关闭官方 `sport_mode` 后，不依赖 `SportModeState`。重力数据
来自 Go2 底层 `LowState.imu_state`（SDK2 DDS 话题 `rt/lowstate`；ROS 2
桥接后的具体话题名由部署配置确定）。使用接收时刻做本地时间戳和超时
检查，并在真机倾斜测试中确认 IMU 四元数方向、分量顺序以及 IMU frame 到
`go2_base` 的固定变换。优先使用板载姿态解算的四元数；加速度计低通结果
只作为静止或低动态条件下的校验/降级数据源。

### 2.1 PickObject

输入语义：目标物体的粗略位置。

任务语义：

```text
STOWED
→ 主动观测
→ 目标检测
→ 精确位姿估计
→ 抓取
→ 抬升验证
→ CARRY
```

成功条件：物体已稳定抓取，机械臂到达 `CARRY`。

接受条件：机械臂未持有物体，且没有其他机械臂任务正在执行。

### 2.2 DropObject

输入语义：垃圾桶中心的粗略位置。

任务语义：

```text
CARRY
→ PLACE_READY
→ 确定投放位置
→ 移动到垃圾桶上方
→ 打开夹爪
→ 释放验证
→ STOWED
```

成功条件：物体已释放，机械臂返回 `STOWED`。

接受条件：机械臂处于 `CARRY`，并且内部状态确认当前持有物体。

### 2.3 Action 公共反馈和结果

两个 Action 使用一致的反馈语义：

```text
current_state
progress
detail
```

需要观测调试时可追加：

```text
pixel_error
candidate_count
```

两个 Action 使用一致的结果语义：

```text
outcome
failure_category
failed_state
detail
returned_to_stowed
```

`outcome` 至少包含：

```text
SUCCEEDED
FAILED
CANCELED
INTERNAL_ERROR
```

## 3. 内部接口

主动观测是 PickObject 内部模块，不是 Go2 的正式业务接口。

开发阶段提供一个调试 Action：

```text
/arm/internal/observe_target
```

该接口只执行：

```text
STOWED
→ CHECK_PRECONDITIONS
→ PLAN_OBSERVE
→ MOVE_OBSERVE
→ OBSERVING
```

完成 PickObject 后，Pick 状态机直接调用同一套观测模块代码，不通过调试
Action 再转发一次。

## 4. 固定姿态

ROS 任务运行时固定姿态由 `d1_manipulation/config/fixed_poses.yaml` 维护。夹爪状态单独控制，
不写入六关节姿态数组。

```yaml
STOWED:      [0.0, -1.54, 1.55, 0.0,  0.0, 0.0]
HOME:        [0.0, -1.0, 1.047, 0.0,  0.0, 0.0]
CARRY:       [0.0, -1.54, 1.55, 0.0, -0.6, 1.57]
PLACE_READY: [0.0, -1.0, 1.047, 0.0,  0.0, 0.0]
```

- `STOWED`：低重心收纳和默认待机姿态；
- `HOME`：通用展开参考姿态，仅用于备用规划和调试；
- `CARRY`：携带物体时使腕部抬高；
- `PLACE_READY`：投放前的展开姿态。当前数值与 `HOME` 相同，但语义独立。

正常观测流程优先直接从 `STOWED` 规划到观测姿态，不强制经过 `HOME`。
直接规划失败时允许尝试：

```text
当前位置 → HOME → 观测姿态
```

## 5. PickObject 完整状态

```text
CURRENT_POSE
→ ENSURE_STOWED
→ STOWED
→ CHECK_PRECONDITIONS
→ PLAN_OBSERVE
→ MOVE_OBSERVE
→ OBSERVING
→ DETECT_OBJECT
→ ESTIMATE_POSE
→ PLAN_PREGRASP
→ MOVE_PREGRASP
→ FINETUNE_GRASP
→ DESCEND
→ GRASP
→ VERIFY_GRASP
→ LIFT
→ CARRY
```

状态语义：

- `ENSURE_STOWED`：若当前各关节距统一 STOWED 不超过 45°，绕过 MoveIt
  初始碰撞检查并通过 `arm_controller` 直接下发完整 STOWED 关节目标；
- `CHECK_PRECONDITIONS`：检查任务输入、TF、关节状态、控制器和相机；
- `PLAN_OBSERVE`：搜索并规划相机观测姿态；
- `MOVE_OBSERVE`：执行观测轨迹；
- `OBSERVING`：相机对准目标并等待画面稳定；
- `DETECT_OBJECT`：从 RGB-D 中检出目标；
- `ESTIMATE_POSE`：计算目标精确三维位姿；
- `PLAN_PREGRASP`：计算并规划预抓取姿态；
- `MOVE_PREGRASP`：移动到物体上方；
- `FINETUNE_GRASP`：yellow_cube 在当前 RGB-D 中重新提取顶面中心，
  与真机标定的安全下落柱体比较，最多三次“小步移动—停止—重新观测”；
- `DESCEND`：沿受约束路径下降；
- `GRASP`：闭合夹爪；
- `VERIFY_GRASP`：判断物体是否被可靠夹持；
- `LIFT`：垂直抬升并验证物体随动；
- `CARRY`：进入携带姿态并结束 PickObject。

## 6. DropObject 完整状态

```text
CARRY
→ CHECK_PRECONDITIONS
→ MOVE_PLACE_READY
→ PLAN_DROP
→ MOVE_DROP
→ RELEASE
→ VERIFY_RELEASE
→ RETURNING_STOWED
→ STOWED
```

状态语义：

- `MOVE_PLACE_READY`：从携带姿态展开到投放准备姿态；
- `PLAN_DROP`：根据垃圾桶粗略位置计算并规划投放姿态；
- `MOVE_DROP`：移动到垃圾桶上方；
- `RELEASE`：打开夹爪；
- `VERIFY_RELEASE`：确认物体已经离开夹爪；
- `RETURNING_STOWED`：安全返回收纳姿态。

## 7. 三类任务失败

任务失败只分为三类。底层具体原因写入 `detail` 和日志，不扩展为大量状态。

### 7.1 PRECONDITION_ERROR

完成任务所需的信息或系统前提不完备，例如：

- 目标坐标缺失、非法或过期；
- `frame_id` 无效；
- TF 缺失或过期；
- 重力方向或 IMU 姿态不可用；
- 关节状态缺失；
- 控制器或相机未就绪；
- 到达观测姿态后没有检出目标；
- 目标区域没有有效深度；
- 无法获得目标精确位姿。

### 7.2 INFEASIBLE

已有信息完整，但计算表明任务理论上不可完成，例如：

- 目标位于任务工作空间之外；
- 没有 IK 解；
- 所有候选状态均发生碰撞；
- 无法生成无碰撞轨迹；
- 抓取姿态或投放姿态不可达。

### 7.3 EXECUTION_FAILED

已经生成理论可行方案，但实际执行异常，例如：

- 控制器拒绝或中止轨迹；
- 轨迹执行超时；
- 跟踪误差超限；
- 实际未到达目标姿态；
- 夹爪未按指令开合；
- 物体未抓住、抬升后掉落或未成功释放。

`CANCELED` 是用户主动取消，不归入三类任务失败。`INTERNAL_ERROR` 表示程序
异常或状态机不一致，也不归入任务失败类别。

## 8. 统一异常恢复

机械臂在任何非 `STOWED` 状态发生异常或收到取消请求时，执行：

```text
停止当前任务
→ 刷新机器人状态和规划场景
→ 规划返回 STOWED
→ 执行收纳轨迹
```

- 收纳成功：保留原始失败类别，设置 `returned_to_stowed=true`；
- 收纳失败：保持当前位置，进入 `RECOVERY_FAILED`，设置
  `returned_to_stowed=false`；
- 异常发生时已经处于 `STOWED`：直接结束任务，不再运动。

以下情况不盲目发送收纳轨迹，直接进入 `RECOVERY_FAILED`：

- 控制链路失联；
- 急停；
- 当前关节状态不可用；
- 无法生成无碰撞的收纳轨迹。

## 9. 第一阶段：目标引导观测

第一阶段只实现到 `OBSERVING`，不实现检测、位姿估计和实际抓取。

### 9.1 观测姿态候选

先把目标点变换到 `base_link`，并取得同一坐标系下的竖直向上单位向量
`z_up_base`。仿真由 `world` 变换得到该向量；真机由 Go2 IMU 姿态得到。
底座指向目标的水平单位向量通过向重力法平面投影计算：

```text
delta = p_target_base - [0, 0, 0]
delta_horizontal = delta - dot(delta, z_up_base) * z_up_base
h = normalize(delta_horizontal)
```

`-h` 是目标水平指向基座的方向。所有观测距离、抬升角、水平偏角和相机
朝向均使用 `-h` 与 `z_up_base`
在 `base_link` 中生成，随后直接用于 IK 和 MoveIt 规划。

候选按配置中的顺序生成，不做加权评分：

```text
beta:     [0, -5, +5, -10, +10, -15, +15] deg
alpha:    [45, 40, 50, 35, 55, 30, 60, 65, 70] deg
distance: [0.35, 0.30, 0.40, 0.45, 0.50, 0.55, 0.60] m
```

循环顺序固定为 `beta → alpha → distance`，共 441 个候选：

```text
s = RotateAroundAxis(-h, z_up_base, beta)
r = cos(alpha) * s + sin(alpha) * z_up_base
p_camera = p_target + distance * r
camera_forward = -r
```

RGB optical `+Z` 使用 `camera_forward`；图像上方向使用 `z_up_base` 在光轴
法平面上的投影。候选通过相机外参转换为 `Link6` 位姿，依次执行 IK、关节
限位、终点碰撞和 MoveIt 完整路径规划。第一个全部成功的候选立即返回，
不再搜索其余候选。

Go2 IMU 只能提供重力方向，不能单独确定地面高度或局部地面是否为斜坡。
真机地面碰撞体还需要独立的地面平面估计，至少包含平面法向和相对
`base_link` 的距离。Go2 倾斜时，不允许继续使用固定在 `base_link` XY
平面上的水平地面近似。

### 9.2 OBSERVING 到位条件

第一版已实现的条件：

- RGB 图像和 CameraInfo 有效；
- 粗略目标位于相机前方；
- 执行后等待 `0.5 s`；
- 使用实时 TF、CameraInfo 和 `plumb_bob` 畸变参数重新投影粗略目标；
- 到图像中心的像素距离不超过 `30 px`；

像素抖动、连续多帧稳定性和关节速度门限留到接入真实目标检测后实现。
所有已实现阈值写入配置文件。

## 10. 状态与调试可视化

发布以下调试信息：

- 当前任务和状态；
- 输入的粗略目标点；
- 最终选中的相机姿态；
- RGB 相机光轴；
- 真实竖直向上方向。

在现有 RViz 配置中增加独立开关，不改变 Go2 mesh、RobotModel、物体 mesh、
MuJoCo 碰撞体和 MoveIt 碰撞体的现有开关。

## 11. 下一阶段实施清单

- [x] 新建 `d1_manipulation` ROS 2 包；
- [x] 定义调试 Action 的结果、反馈和失败类别；
- [x] 定义调试用 `ObserveTarget.action`；
- [ ] 预留正式 `PickObject.action` 和 `DropObject.action`；
- [ ] 将固定姿态迁移到唯一配置文件；
- [x] 实现到 `OBSERVING` 的第一阶段状态；
- [x] 实现目标 `PointStamped` 的 TF 转换和检查；
- [x] 实现有序候选生成、IK、碰撞检查和完整规划；
- [x] 实现轨迹执行、取消处理和执行失败恢复；
- [x] 实现异常返回 STOWED；
- [x] 实现状态反馈和最终候选 Marker 可视化；
- [x] 更新现有 RViz 配置；
- [x] 编写一键观测验收脚本；
- [x] 编写候选几何自动化测试；
- [x] 更新项目 README 和 `doc/codex_handoff.md`；
- [ ] 接入真机 `LowState` 重力方向适配器；
- [x] 接入腕部 RGB、对齐深度和 CameraInfo 的 ROS 感知适配层；
- [x] 保证感知请求只使用调用后到达的新同步 RGB-D 帧；
- [x] 使用粗略目标投影匹配 YOLO 实例并输出相机系/base_link 观测中心；
- [x] 将黄色立方体感知服务纳入正式 `PickObject.action` 状态机；
- [x] 实现重力约束地面 RANSAC、粗中心和严格俯视二次观测；
- [x] 实现顶面轮廓射线投影、正方形拟合和四个对称抓取候选；
- [x] 实现 compute/pregrasp/descend/lift 四档验收；
- [ ] 真机接入地面平面估计并替换仿真固定地面碰撞体；
- [ ] 自动测试取消、不可达目标和恢复路径。

## 12. 第一阶段验收

一键验收流程：

1. 启动 MuJoCo；
2. 启动 ros2_control、MoveIt 和 `d1_manipulation`；
3. 以 `go2_base` 坐标发送黄色立方体的粗略位置；
4. 从 `STOWED` 直接规划到观测姿态；
5. 验证目标投影进入 RGB 中心区域；
6. 保持 `OBSERVING` 供 MuJoCo 和 RViz 人工检查；
7. 验证取消任务后返回 `STOWED`；
8. 验证 `PRECONDITION_ERROR`、`INFEASIBLE` 和
   `EXECUTION_FAILED` 的结果分类；
9. 验证恢复结果 `returned_to_stowed`；
10. 验证现有固定方块抓取基线未被破坏。

## 13. 本阶段不做

- Go2 导航、步态、站立或 yaw 控制；
- Go2 机身相机标定；
- 目标检测模型；
- RGB-D 精确位姿估计；
- PickObject 的真实抓取后半段；
- DropObject 的真实投放流程；
- 真机控制参数调优。
