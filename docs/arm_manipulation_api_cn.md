# D1 机械臂抓放服务 API 接口文档

## 1. 文档信息

| 项目 | 内容 |
| --- | --- |
| 接口包 | `d1_interfaces` |
| 通信框架 | ROS 2 Humble / DDS |
| 接口版本 | `0.1.0` |
| 坐标单位 | 米（m） |
| 角度单位 | 结果中的 yaw/tilt 为度（°），ROS Pose 姿态使用四元数 |
| 对外接口 | 2 个 ROS 2 Action，1 个 ROS 2 Topic |

本文是 Go2 任务节点调用 D1 机械臂模块的外部契约。感知、MoveIt、调试和内部状态迁移接口不属于公共 API。

## 2. 接口概览

| 名称 | 类型 | ROS 2 类型 | 用途 |
| --- | --- | --- | --- |
| `/arm/tasks/pick_object` | Action | `d1_interfaces/action/PickObject` | 根据物体粗略位置完成识别、抓取、抬升和携带 |
| `/arm/tasks/drop_object` | Action | `d1_interfaces/action/DropObject` | 根据垃圾桶底部中心完成投放并返回 STOWED |
| `/arm/task_status` | Topic | `d1_interfaces/msg/ArmTaskStatus` | 发布机械臂当前业务状态、载荷状态、执行阶段和故障信息 |

调用方只需要依赖 `d1_interfaces`。调用节点必须与机械臂服务使用相同的 `ROS_DOMAIN_ID`，并能通过 TF 将请求坐标系转换到机械臂规划坐标系。

## 3. 通用约定

### 3.1 调用前检查

在发送任务前，应确认三个接口均已发现：

```bash
ros2 action list | grep '^/arm/tasks/'
ros2 topic list | grep '^/arm/task_status$'
```

正常输出：

```text
/arm/tasks/drop_object
/arm/tasks/pick_object
/arm/task_status
```

调用方应持续订阅 `/arm/task_status`，并且只在允许的状态下发送任务。不要仅根据 Action Server 可发现就判断机械臂已经就绪。

### 3.2 坐标约定

所有目标点均使用 `geometry_msgs/PointStamped`：

```text
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
```

约束如下：

- `frame_id` 必填，不能为空；
- `x/y/z` 单位为米，且必须是有限数值；
- 推荐使用 Go2 基座坐标系或机械臂 `base_link`；使用其他坐标系时必须存在可用 TF；
- `stamp=0` 表示使用最新 TF；`pick_object` 的观测阶段支持按非零时间戳查询 TF，要求该时刻仍在 TF 缓存中；当前 `drop_object` 始终按最新 TF 变换投放点；
- `pick_object.target` 和 `drop_object.target` 的业务含义不同，不能混用。

### 3.3 Action 结果判定

ROS 2 Action 的终态与业务结果是两个层次：

| ROS 2 Action 终态 | 含义 |
| --- | --- |
| `SUCCEEDED` | 本次业务成功，`result.success=true` |
| `ABORTED` | 本次业务未完成；可能已经安全恢复，也可能进入故障，必须读取 `outcome` |
| `CANCELED` | 调用方取消了任务；当前实现停止活动命令并进入 `FAULTED`，不自动执行后续恢复动作 |
| Goal rejected | 请求格式不合法，或服务端在接收阶段拒绝了并发/状态冲突；不会得到完整 Action Result，应立即查询 `/arm/task_status` |

调用方必须以 `outcome` 作为下一步决策依据。`failure_category`、`failed_state` 和 `detail` 用于诊断，不应替代 `outcome`。

当前 `pick_object` 对生产任务的状态冲突会接受 Goal 后返回 `ABORTED`，并给出 `failed_state=REQUEST` 的完整 Result；`drop_object` 的并发或状态冲突可能直接拒绝 Goal。调用端必须同时兼容这两种表现。

### 3.4 并发、超时与重试

- 同一时间只允许执行一个 `pick_object` 或 `drop_object`；
- 任务执行期间不得并发发送另一个抓放请求；
- 客户端等待超时不代表机械臂没有执行，超时后应先查询 `/arm/task_status`，禁止直接重发；
- 收到 `REPOSITION_REQUIRED` 或 `NEW_TARGET_REQUIRED` 后，应等待状态回到对应稳定状态再发送新请求；
- 收到 `ARM_FAULTED` 或状态变为 `FAULTED` 后，不得继续调用抓放接口；
- 软件取消不能替代物理急停或断电措施。

## 4. 抓取接口 `pick_object`

### 4.1 基本信息

| 项目 | 内容 |
| --- | --- |
| Action 名称 | `/arm/tasks/pick_object` |
| Action 类型 | `d1_interfaces/action/PickObject` |
| 生产调用前置状态 | `READY_STOWED` |
| 成功后状态 | `READY_CARRY` |
| 成功后载荷状态 | `PAYLOAD_HELD` |
| 支持类别 | `yellow_cube`、`zucchini`、`bowl` |

调用方不传物体类别。`target` 是物体的大致位置提示，机械臂移动到观测姿态后会在允许类别中自动识别目标，并重新估计精确抓取位姿。

### 4.2 请求参数

```text
uint8 COMPUTE_ONLY=0
uint8 MOVE_PREGRASP=1
uint8 DESCEND=2
uint8 GRASP_AND_LIFT=3
uint8 GRASP_AND_CARRY=4

geometry_msgs/PointStamped target
uint8 stop_after
```

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `target` | `geometry_msgs/PointStamped` | 是 | Go2 估计的目标物体粗略位置，用于规划观测动作和关联视觉目标 |
| `target.header.frame_id` | `string` | 是 | 目标点所属坐标系；必须存在到机械臂规划坐标系的 TF |
| `target.header.stamp` | `Time` | 否 | 零值使用最新 TF；非零值供观测阶段查询对应时刻的 TF |
| `target.point` | `Point` | 是 | 目标位置，单位为米 |
| `stop_after` | `uint8` | 是 | 生产环境必须为 `GRASP_AND_CARRY(4)` |

`COMPUTE_ONLY(0)` 至 `GRASP_AND_LIFT(3)` 是分阶段调试能力，默认生产启动不会开放。生产环境传入非 `4` 值时 Goal 会被拒绝，且不会执行机械臂动作。

### 4.3 命令行调用

以下坐标仅用于展示请求格式，不能直接作为其他现场的安全抓取点：

```bash
ros2 action send_goal /arm/tasks/pick_object \
  d1_interfaces/action/PickObject \
  "{target: {header: {frame_id: base_link}, point: {x: 0.20, y: 0.35, z: -0.15}}, stop_after: 4}" \
  --feedback
```

使用 Go2 坐标系时，将 `frame_id` 改为实际 Go2 frame，并传入该坐标系下的目标点。机械臂服务必须能够查询该 frame 到 `base_link` 的 TF。

### 4.4 执行反馈

```text
string current_state
float32 progress
string detail
```

| 字段 | 说明 |
| --- | --- |
| `current_state` | 当前抓取阶段 |
| `progress` | 名义流程进度，范围 `0.0~1.0`；不表示剩余时间 |
| `detail` | 当前阶段的人类可读说明 |

生产抓取可能依次出现以下阶段：

| `current_state` | 含义 |
| --- | --- |
| `ENSURE_STOWED` | 检查或恢复到标准 STOWED 姿态 |
| `PLAN_OBSERVE` / `MOVE_OBSERVE` | 规划并执行粗观测动作 |
| `CLASSIFY_TARGET` | 从支持类别中识别目标 |
| `PREPARE_GRASP` | 精观测并计算物体专用抓取方案 |
| `MOVE_PREGRASP` | 移动到预抓取姿态 |
| `FINETUNE_GRASP` | 使用腕部 RGB-D 将目标调整到标定安全区 |
| `PRECHECK_ESCAPE` | 在闭合夹爪前验证携物 LIFT 和 CARRY 退路 |
| `DESCEND` | 保持物体专用姿态下降 |
| `GRASP` | 闭合夹爪 |
| `ATTACH_OBJECT` | 将感知估计的物体加入 MoveIt 附着碰撞模型 |
| `LIFT` | 从物体附近抬升 |
| `CARRY` | 移动到携带姿态 |
| `VERIFY_GRASP` | 根据 Joint6 反馈验证物体是否仍被夹持 |
| `RECOVERING_TO_STOWED` | 抓取未保持或业务失败后返回 STOWED |

### 4.5 返回参数

```text
bool success
uint8 outcome
uint8 failure_category
string failed_state
string detail
bool returned_to_stowed
uint8 final_task_state
uint8 payload_state
string class_name
geometry_msgs/PointStamped estimated_center
geometry_msgs/PoseStamped grasp_pose
geometry_msgs/PoseStamped pregrasp_pose
float64 pregrasp_distance_m
float64 grasp_distance_m
float64 grasp_yaw_degrees
float64 approach_tilt_degrees
```

| 字段 | 说明 |
| --- | --- |
| `success` | 是否完成完整抓取、进入 CARRY 并通过夹持复检 |
| `outcome` | 面向调用方的最终决策码，必须优先处理 |
| `failure_category` | 失败技术分类；成功时为 `FAILURE_NONE(0)` |
| `failed_state` | 首次确定失败的内部阶段；仅用于诊断 |
| `detail` | 详细原因、恢复结果和建议；文本可能随实现更新 |
| `returned_to_stowed` | 返回结果时是否已确认处于 STOWED |
| `final_task_state` | 返回结果时的任务状态，枚举与 `/arm/task_status.state` 相同 |
| `payload_state` | 返回结果时的载荷状态，枚举与 `/arm/task_status.payload_state` 相同 |
| `class_name` | 自动识别出的类别名称 |
| `estimated_center` | 精观测或微调后的物体中心估计 |
| `grasp_pose` | 最终选中的 TCP 抓取位姿 |
| `pregrasp_pose` | 最终选中的 TCP 预抓取位姿 |
| `pregrasp_distance_m` | 物体策略选中的预抓取距离或离地间隙，用于诊断 |
| `grasp_distance_m` | 物体策略选中的抓取深度或离地间隙，用于诊断 |
| `grasp_yaw_degrees` | 最终抓取 yaw |
| `approach_tilt_degrees` | 最终 approach 倾角；当前生产策略通常为 `0` |

位姿、距离和识别字段只应在 `success=true` 时作为可靠结果使用。调用方不应根据这些诊断字段自行复现机械臂轨迹。

### 4.6 `outcome` 枚举与处理方式

| 值 | 常量 | 含义 | Go2 处理方式 |
| --- | --- | --- | --- |
| `0` | `OUTCOME_SUCCESS` | 抓取成功，物体已在 CARRY 状态下通过复检 | 继续导航至投放区域，随后调用 `drop_object` |
| `1` | `OUTCOME_REPOSITION_REQUIRED` | 当前视角、可达性或夹持结果不满足要求，机械臂已安全返回 STOWED | 调整 Go2 位置或视角，等待 `READY_STOWED` 后重新抓取 |
| `2` | `OUTCOME_ARM_FAULTED` | 执行、通信、恢复或状态同步发生故障 | 停止自动任务，读取 `/arm/task_status` 并人工处理 |

### 4.7 `failure_category` 枚举

| 值 | 常量 | 含义 |
| --- | --- | --- |
| `0` | `FAILURE_NONE` | 无失败 |
| `1` | `FAILURE_INCOMPLETE_INFORMATION` | 视觉、深度、TF、重力或当前状态信息不足 |
| `2` | `FAILURE_THEORETICALLY_INFEASIBLE` | 没有对应抓取策略或理论候选不可行 |
| `3` | `FAILURE_EXECUTION_ERROR` | 规划执行、控制器、通信、取消、恢复或内部运行错误 |
| `4` | `FAILURE_REPOSITION_REQUIRED` | 当前 Go2/机械臂相对位置下无安全可达方案 |
| `5` | `FAILURE_GRASP_NOT_SECURED` | 已执行抓取，但 CARRY 阶段复检确认物体没有可靠保持 |

同一 `failure_category` 最终可能因恢复是否成功而得到不同 `outcome`，因此调用方不能只根据本字段决定是否重试。

### 4.8 返回示例

成功时的核心字段：

```yaml
success: true
outcome: 0                 # OUTCOME_SUCCESS
failure_category: 0        # FAILURE_NONE
failed_state: ""
returned_to_stowed: false
final_task_state: 2        # READY_CARRY
payload_state: 1           # PAYLOAD_HELD
class_name: yellow_cube
detail: yellow_cube grasped, carried and mechanically verified: ...
```

需要调整 Go2 时：

```yaml
success: false
outcome: 1                 # OUTCOME_REPOSITION_REQUIRED
failure_category: 4        # FAILURE_REPOSITION_REQUIRED
failed_state: PLAN_PREGRASP
returned_to_stowed: true
final_task_state: 1        # READY_STOWED
payload_state: 0           # PAYLOAD_EMPTY
detail: no cube pregrasp has a feasible downstream grasp; ...
```

## 5. 投放接口 `drop_object`

### 5.1 基本信息

| 项目 | 内容 |
| --- | --- |
| Action 名称 | `/arm/tasks/drop_object` |
| Action 类型 | `d1_interfaces/action/DropObject` |
| 正常任务前置状态 | `READY_CARRY` |
| 空载验收前置状态 | `READY_STOWED` |
| 成功后状态 | `READY_STOWED` |
| 成功后载荷状态 | `PAYLOAD_EMPTY` |

`READY_STOWED` 下允许空载调用，用于部署验收；实际捡垃圾任务应在抓取成功并进入 `READY_CARRY` 后调用。

### 5.2 请求参数

```text
geometry_msgs/PointStamped target
```

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `target` | `geometry_msgs/PointStamped` | 是 | Go2 估计的垃圾桶底部中心，不是桶沿、物体中心或 TCP 释放位置 |
| `target.header.frame_id` | `string` | 是 | 投放点所属坐标系；必须存在到规划坐标系的 TF |
| `target.header.stamp` | `Time` | 否 | 当前实现使用最新 TF，建议传零值 |
| `target.point` | `Point` | 是 | 垃圾桶底部中心，单位为米 |

服务会根据垃圾桶底部中心和重力方向搜索 TCP 释放高度及 yaw，并预先验证去程和完全松开后的 STOWED 回程。垃圾桶中心与机械臂 `base_link` 的推荐水平距离为 `0.35~0.45 m`，该范围是导航建议，不是请求格式约束。

Go2 平台 XY 包络及其外扩安全区属于禁投区。目标点落入禁投区时机械臂不会运动，并返回 `OUTCOME_NEW_TARGET_REQUIRED`。

### 5.3 命令行调用

以下坐标只展示格式，调用前必须替换为现场估计的垃圾桶底部中心：

```bash
ros2 action send_goal /arm/tasks/drop_object \
  d1_interfaces/action/DropObject \
  "{target: {header: {frame_id: base_link}, point: {x: 0.00, y: 0.40, z: -0.28}}}" \
  --feedback
```

### 5.4 执行反馈

```text
string current_state
float32 progress
string detail
```

| `current_state` | 含义 |
| --- | --- |
| `CHECK_PRECONDITIONS` | 检查载荷、MoveIt 起始状态、TF、重力和 Go2 禁投区 |
| `PLAN_RELEASE` | 搜索同时满足去程与回程要求的释放候选 |
| `MOVE_RELEASE` | 移动到选定释放位姿 |
| `RELEASE` | 完全张开夹爪并移除附着物体 |
| `RELEASE_HOLD` | 保持夹爪完全张开 |
| `STOWED` | 执行预验证的空载回程并返回 STOWED |
| `RECOVERING_TO_STOWED` | 空载任务失败后恢复 STOWED |
| `RECOVERING_TO_CARRY` | 携物任务在释放前失败后恢复 CARRY |

`progress` 是名义流程进度，不代表剩余时间。候选搜索数量不同会导致 `PLAN_RELEASE` 耗时变化。

### 5.5 返回参数

```text
bool success
uint8 outcome
uint8 failure_category
string failed_state
string detail
bool returned_to_stowed
uint8 final_task_state
uint8 payload_state
geometry_msgs/PoseStamped release_pose
float64 height_offset_m
float64 release_yaw_degrees
```

| 字段 | 说明 |
| --- | --- |
| `success` | 是否完成释放并确认返回 STOWED |
| `outcome` | 面向调用方的最终决策码 |
| `failure_category` | 失败技术分类 |
| `failed_state` | 首次确定失败的内部阶段 |
| `detail` | 详细原因、候选统计和恢复结果 |
| `returned_to_stowed` | 返回结果时是否确认处于 STOWED |
| `final_task_state` | 返回结果时的任务状态 |
| `payload_state` | 返回结果时的载荷状态 |
| `release_pose` | 成功时实际选中的 TCP 释放位姿 |
| `height_offset_m` | 成功候选相对基准释放高度的偏移，单位为米 |
| `release_yaw_degrees` | 成功候选相对参考方向的 yaw 偏移，单位为度 |

`release_pose`、`height_offset_m` 和 `release_yaw_degrees` 只在 `success=true` 时有效。

### 5.6 `outcome` 枚举与处理方式

| 值 | 常量 | 含义 | Go2 处理方式 |
| --- | --- | --- | --- |
| `0` | `OUTCOME_SUCCESS` | 释放成功并返回 STOWED | 继续下一项任务 |
| `1` | `OUTCOME_REPOSITION_REQUIRED` | 当前相对位置没有安全释放往返路径；释放前失败时会保持载荷并恢复 CARRY | 调整 Go2 与垃圾桶的相对位置，等待稳定状态后重试 |
| `2` | `OUTCOME_ARM_FAULTED` | 执行、通信、状态一致性或恢复失败 | 停止自动任务并人工处理 |
| `3` | `OUTCOME_NEW_TARGET_REQUIRED` | 请求点位于 Go2 禁投区；机械臂保持或恢复至调用前稳定状态 | 重新指定垃圾桶底部中心，不应原样重发 |

### 5.7 `failure_category` 枚举

| 值 | 常量 | 含义 |
| --- | --- | --- |
| `0` | `FAILURE_NONE` | 无失败 |
| `1` | `FAILURE_INCOMPLETE_INFORMATION` | 载荷、附着物体、TF、重力或规划场景信息不足/不一致 |
| `2` | `FAILURE_THEORETICALLY_INFEASIBLE` | 理论释放约束无法满足 |
| `3` | `FAILURE_EXECUTION_ERROR` | 规划执行、夹爪、控制器、通信、取消、恢复或内部运行错误 |
| `4` | `FAILURE_REPOSITION_REQUIRED` | 搜索后没有安全可达的释放往返方案 |
| `5` | `FAILURE_TARGET_IN_KEEP_OUT` | 垃圾桶底部中心位于 Go2 平台禁投区 |

### 5.8 返回示例

携物投放成功：

```yaml
success: true
outcome: 0                 # OUTCOME_SUCCESS
failure_category: 0        # FAILURE_NONE
failed_state: ""
returned_to_stowed: true
final_task_state: 1        # READY_STOWED
payload_state: 0           # PAYLOAD_EMPTY
height_offset_m: 0.0
release_yaw_degrees: 0.0
detail: held object released above trash bin and arm returned to STOWED
```

投放点位于禁投区：

```yaml
success: false
outcome: 3                 # OUTCOME_NEW_TARGET_REQUIRED
failure_category: 5        # FAILURE_TARGET_IN_KEEP_OUT
failed_state: CHECK_PRECONDITIONS
detail: drop target XY=(...) lies inside the Go2 platform keep-out region; submit a new drop target; ...
```

## 6. 状态接口 `/arm/task_status`

### 6.1 基本信息

| 项目 | 内容 |
| --- | --- |
| Topic 名称 | `/arm/task_status` |
| 消息类型 | `d1_interfaces/msg/ArmTaskStatus` |
| QoS | Reliable、Transient Local、Keep Last 1 |
| 发布方式 | 状态变化时发布；新订阅者会立即收到最近一次状态 |

`/arm/task_status` 是保留最近值的系统状态流，不是某次 Action 的返回值。Action Result 描述单次调用的最终结果；本 Topic 描述机械臂当前的全局状态。发生断线、客户端超时或 Goal 被拒绝时，应以本 Topic 判断机械臂是否仍在执行或已经故障。

### 6.2 订阅方式

```bash
ros2 topic echo /arm/task_status
```

只读取最近一次状态：

```bash
ros2 topic echo /arm/task_status --once
```

空载待命状态示例：

```yaml
state: 1                   # READY_STOWED
payload_state: 0           # PAYLOAD_EMPTY
canonical_pose: 0          # POSE_STOWED
active_operation: ''
active_phase: ''
failure_code: ''
detail: Verified canonical STOWED feedback and base planning scene; ...
```

### 6.3 消息字段

```text
builtin_interfaces/Time stamp
uint8 state
uint8 payload_state
uint8 canonical_pose
string active_operation
string active_phase
string failure_code
string detail
```

| 字段 | 说明 |
| --- | --- |
| `stamp` | 状态生成时间 |
| `state` | 当前业务状态 |
| `payload_state` | 当前是否确认持有物体 |
| `canonical_pose` | 当前是否处于标准 STOWED/CARRY 姿态 |
| `active_operation` | 当前操作，通常为 `pick_object`、`drop_object` 或空字符串 |
| `active_phase` | 当前 Action 内部阶段；空闲时为空字符串 |
| `failure_code` | 机器可记录的故障标识；正常稳定状态下为空 |
| `detail` | 当前状态、阶段或故障的详细说明 |

### 6.4 `state` 枚举

| 值 | 常量 | 含义 | 允许的新任务 |
| --- | --- | --- | --- |
| `0` | `INITIALIZING` | 正在验证关节反馈、STOWED 和基础规划场景 | 无 |
| `1` | `READY_STOWED` | 空载且处于 STOWED | `pick_object`；允许空载 `drop_object` 验收 |
| `2` | `READY_CARRY` | 已确认持物且处于 CARRY | `drop_object` |
| `3` | `PICKING` | 正在执行抓取 | 无 |
| `4` | `DROPPING` | 正在执行投放 | 无 |
| `5` | `RECOVERING_TO_STOWED` | 正在自动恢复 STOWED | 无 |
| `6` | `RECOVERING_TO_CARRY` | 正在保留载荷并恢复 CARRY | 无 |
| `7` | `FAULTED` | 状态、执行或恢复发生不可自动处理的故障 | 无；需要人工处理 |

启动阶段只有在收到新鲜关节反馈、确认 STOWED 并确认基础规划场景已建立后才会进入 `READY_STOWED`。若当前姿态位于配置的近 STOWED 范围内，系统可能执行一次有界启动恢复；偏差过大时不会运动并进入 `FAULTED`。

### 6.5 `payload_state` 枚举

| 值 | 常量 | 含义 |
| --- | --- | --- |
| `0` | `PAYLOAD_EMPTY` | 已确认夹爪未携带任务物体 |
| `1` | `PAYLOAD_HELD` | 已确认夹爪携带任务物体 |
| `2` | `PAYLOAD_UNKNOWN` | 无法可靠判断载荷状态；不得自行假设为空或持物 |

### 6.6 `canonical_pose` 枚举

| 值 | 常量 | 含义 |
| --- | --- | --- |
| `0` | `POSE_STOWED` | 标准收纳姿态 |
| `1` | `POSE_CARRY` | 标准携带姿态 |
| `2` | `POSE_OTHER` | 已知不在两个标准姿态 |
| `3` | `POSE_UNKNOWN` | 无法可靠判断姿态 |

## 7. 调用方决策表

### 7.1 `pick_object`

| `outcome` | 预期状态 | 调用方动作 |
| --- | --- | --- |
| `OUTCOME_SUCCESS` | `READY_CARRY / PAYLOAD_HELD` | 导航到垃圾桶附近并调用 `drop_object` |
| `OUTCOME_REPOSITION_REQUIRED` | `READY_STOWED / PAYLOAD_EMPTY` | 调整 Go2 位置或视角后重试抓取 |
| `OUTCOME_ARM_FAULTED` | `FAULTED / PAYLOAD_UNKNOWN` | 停止自动流程并人工排障 |

### 7.2 `drop_object`

| `outcome` | 预期状态 | 调用方动作 |
| --- | --- | --- |
| `OUTCOME_SUCCESS` | `READY_STOWED / PAYLOAD_EMPTY` | 继续下一任务 |
| `OUTCOME_REPOSITION_REQUIRED` | 通常为 `READY_CARRY / PAYLOAD_HELD`；空载验收时为 `READY_STOWED / PAYLOAD_EMPTY` | 调整 Go2 与垃圾桶的相对位置后重试 |
| `OUTCOME_NEW_TARGET_REQUIRED` | 保持或恢复调用前稳定状态 | 重新估计/指定垃圾桶底部中心后重试 |
| `OUTCOME_ARM_FAULTED` | `FAULTED / PAYLOAD_UNKNOWN` | 停止自动流程并人工排障 |

调用方应同时校验 Action Result 中的 `final_task_state/payload_state` 与最新 `/arm/task_status`。两者不一致时禁止继续任务，并将其作为状态同步故障处理。

## 8. 接入代码依赖

Go2 业务仓库只需编译依赖 `d1_interfaces`，不应依赖 `d1_manipulation` 内部包。

Python：

```python
from d1_interfaces.action import PickObject, DropObject
from d1_interfaces.msg import ArmTaskStatus
```

C++：

```cpp
#include "d1_interfaces/action/pick_object.hpp"
#include "d1_interfaces/action/drop_object.hpp"
#include "d1_interfaces/msg/arm_task_status.hpp"
```

`package.xml`：

```xml
<depend>d1_interfaces</depend>
```

推荐的调用顺序为：订阅并等待稳定状态、发送 Action、持续处理 Feedback、等待 Result、根据 `outcome` 决策、最后使用 `/arm/task_status` 复核全局状态。不要根据 `detail` 文本匹配业务分支；该字段用于日志和人工诊断，稳定的程序分支只应依赖枚举字段。
