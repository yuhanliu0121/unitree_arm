# D1 机械臂抓放服务 API 接口文档

简体中文 | [English](README.md)

本文面向机械臂抓放模块的调用方，说明模块提供哪些公共接口、如何发送请求、请求和返回字段的含义，以及调用方应如何根据返回结果继续业务流程。

## 1. 整体工作流程
<!-- 这是一张图片，ocr 内容为： -->
![](images/manipulation_workflow.png)  


模块启动后先进入 `INITIALIZING` 状态，完成关节反馈、标准姿态和规划场景检查后进入 `READY_STOWED` 状态，表示机械臂已调整至收纳（STOWED）姿态，可以接受抓取任务。调用 `pick_object` 后，机械臂进入 `PICKING` 状态；抓取成功时进入 `READY_CARRY`，表示已运动至携带（CARRY）姿态并通过夹持复检，确认持有物体，可以接受投放任务。调用 `drop_object` 后，机械臂进入 `DROPPING` 状态；投放成功后释放物体并回到 `READY_STOWED`，可开始下一轮任务。

抓取过程中发生可自动恢复的业务失败时（如未识别到物体、抓取姿态不可达或找不到无碰撞的运动轨迹），机械臂会通过 `RECOVERING_TO_STOWED` 状态尝试返回收纳姿态；携物投放在释放前失败时，会尽量通过 `RECOVERING_TO_CARRY` 阶段保留物体并回到携带姿态。恢复失败，或出现无法自动恢复的执行、通信或状态故障时，系统进入 `FAULTED`，调用方应停止自动任务并进行人工排障。各状态的数值、允许操作和完整转移条件见后文。

<font style="color:#df2a3f;">注意：本抓取模块并非通用物体抓取方案，暂不支持任意形状物体抓取；当前仅支持黄色立方体、绿色条状物（如西葫芦）和碗这三类目标，物体详情参见：</font>[充当垃圾的物体](objects/README_CN.md)

## 2. 接口概览
机械臂抓放模块对外提供两个 ROS 2 Action 接口和一个状态 Topic。调用方只需提供待抓物体的大致位置或者投放点的位置即可让机械臂执行抓取和投放任务，无需指定物体类别、抓取姿态、释放姿态或机械臂轨迹。

若任务失败，接口也会返回相应的信息（如：调用方应调整机器人载体与目标的相对位置，使目标进入可观测、可抓取范围后重试）供调用方进行下一步决策。

| 接口名称 | 接口形式 | ROS 2 类型 | 用途 | 可调用时机 | 成功后的状态 |
| --- | --- | --- | --- | --- | --- |
| `/arm/tasks/pick_object` | Action | `d1_interfaces/action/PickObject` | 根据物体粗略位置完成识别、抓取、抬升和携带 | 机械臂处于`READY_STOWED` 状态时 | `READY_CARRY` |
| `/arm/tasks/drop_object` | Action | `d1_interfaces/action/DropObject` | 根据垃圾桶底部中心完成投放并返回收纳姿态 | 机械臂处于`READY_CARRY`或 `READY_STOWED` 状态时 | `READY_STOWED` |
| `/arm/task_status` | Topic | `d1_interfaces/msg/ArmTaskStatus` | 发布机械臂当前业务状态、载荷状态、执行阶段和故障信息 | 任意状态均可订阅 | 不改变机械臂状态 |


### 2.1 ROS 2 Action 接口的参数组成
ROS 2 Action 由 Goal、Feedback 和 Result 三部分组成。它们是 ROS 2 强类型消息，并非 JSON：命令行使用 YAML 输入和显示，Python/C++ 使用对应的消息对象。

Goal 是任务入参；Feedback 是执行中的过程反馈；Result 是任务结束后的最终结果：

| 数据 | 数据方向 | 发送或返回次数 | 作用 |
| --- | --- | --- | --- |
| Goal（任务请求） | 调用方 → 机械臂 | 每次调用发送一次 | Action 的入参，用于告诉机械臂执行什么任务 |
| Feedback（执行过程反馈） | 机械臂 → 调用方 | 执行过程中返回零次或多次 | 报告当前阶段、估算进度和过程说明；仅用于进度显示和辅助诊断 |
| Result（最终返回结果） | 机械臂 → 调用方 | 已接受的任务结束时返回一次 | Action 的最终输出，用于说明任务是否成功、机械臂最终状态以及调用方下一步应做什么 |


完整调用过程如下：

```text
调用方发送 Goal
    ↓
机械臂执行任务，期间持续返回 Feedback
    ↓
任务结束，机械臂返回一次 Result
```

## 3. 调用前准备
以下操作假设已按照[部署教程](../../deploy/README_CN.md)完成环境配置，并进入开发容器。在第一个终端启动机械臂控制栈：

```bash
# 启动控制栈与Rviz
cd /workspace && ./scripts/real_bringup.zsh --rviz

# 只启动控制栈，不启动rivz
cd /workspace && ./scripts/real_bringup.zsh
```

出现 `D1 REAL CONTROL STACK READY` 后，保持该终端运行。

另开终端进入同一容器，加载相同的 ROS 环境和 `ROS_DOMAIN_ID`，首先确认两个 Action 和状态 Topic 均已发现：

```bash
ros2 action list | grep '^/arm/tasks/'
ros2 topic list | grep '^/arm/task_status$'
```

正常输出至少包含：

```latex
/arm/tasks/drop_object
/arm/tasks/pick_object
/arm/task_status
```

然后读取机械臂当前状态：

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

开始正常抓取前，应确认 `state: 1`（`READY_STOWED`）和 `payload_state: 0`（`PAYLOAD_EMPTY`），此时机械臂服务已就绪。

## 4. 抓取接口 `/arm/tasks/pick_object`
### 4.1 工作流程概述
调用 `pick_object` 并传入待抓取物体位置后，机械臂会先驱动相机对准该位置，进行初步观测和物体识别。识别成功后，机械臂会移动至物体上方开展精细观测，进行抓取位姿估计。若抓取位姿及后续运动路径可行，机械臂会下降并闭合夹爪，随后抬升、返回携带姿态并进行夹持复检，最后返回任务结果。

任务结束后，接口通过最终结果 Result 中的 `outcome` 字段，指示调用方下一步应继续投放、调整机器人载体位置后重新抓取，还是停止自动任务并人工排障，同时返回机械臂的最终状态和载荷状态。

### 4.2 接口描述
| 项目 | 内容 |
| --- | --- |
| Action 名称 | `/arm/tasks/pick_object` |
| Action 类型 | `d1_interfaces/action/PickObject` |
| 调用前置状态 | `READY_STOWED` |
| 成功后状态 | `READY_CARRY` |
| 成功后载荷状态 | `PAYLOAD_HELD` |


### 4.3 任务请求（Goal）参数
| 字段 | 类型 | 必填 | 取值与说明 |
| --- | --- | --- | --- |
| `target` | `geometry_msgs/PointStamped` | 是 | 调用方估计的目标物体粗略位置，用于规划观测动作和关联视觉目标 |
| `target.header.frame_id` | `string` | 是 | 目标数据产生时所属的原始坐标系；必须存在从该坐标系到机械臂规划坐标系的 TF |
| `target.header.stamp` | `Time` | 否 | 优先沿用目标数据的原始时间戳，以便查询目标产生时刻的 TF；零值表示明确使用最新 TF |
| `target.point.x/y/z` | `float64` | 是 | 目标位置，单位为米，必须为有限数值 |
| `stop_after` | `uint8` | 是 | 生产环境固定传 `4`，即 `GRASP_AND_CARRY` |


`stop_after` 的完整枚举如下。`0–3` 仅用于分阶段调试，默认生产启动不会开放；生产环境传入非 `4` 值时，Goal 会被拒绝且机械臂不会运动。

| 值 | 枚举 | 含义 |
| --- | --- | --- |
| `0` | `COMPUTE_ONLY` | 仅计算，不执行抓取动作 |
| `1` | `MOVE_PREGRASP` | 执行到预抓取姿态后停止 |
| `2` | `DESCEND` | 执行到下降结束后停止 |
| `3` | `GRASP_AND_LIFT` | 抓取并抬升后停止 |
| `4` | `GRASP_AND_CARRY` | 完成抓取并进入 CARRY；生产环境固定使用此值 |


### 4.4 反馈（Feedback）参数
| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `current_state` | `string` | 当前抓取的内部执行阶段，与 `/arm/task_status.state` 的公共状态不同；例如 `MOVE_PREGRASP`、`DESCEND` 或 `CARRY` |
| `progress` | `float32` | 按执行阶段估算的进度，范围为 `0.0–1.0`，不表示剩余执行时间 |
| `detail` | `string` | 当前阶段的人类可读说明，不应用于程序分支 |


Feedback 只用于显示任务进度和辅助诊断。调用方不能根据某条 Feedback 判断任务已经成功，必须等待最终 Result。

### 4.5 结果（Result）参数
| 字段 | 类型 | 取值或有效条件 | 含义与调用方处理方式 |
| --- | --- | --- | --- |
| `success` | `bool` | `true` / `false` | 是否完成完整抓取、进入 CARRY 并通过夹持复检。快速查看可使用该字段，程序决策应以 `outcome` 为准 |
| `outcome` | `uint8` | `0 OUTCOME_SUCCESS` | 抓取成功；调用方可导航至投放区域并调用 `drop_object` |
|  |  | `1 OUTCOME_REPOSITION_REQUIRED` | 当前相对位置没有安全可达的抓取方案；机械臂预期已回到 `READY_STOWED / PAYLOAD_EMPTY`，调用方应调整机器人载体的位置或视角后重新抓取 |
|  |  | `2 OUTCOME_ARM_FAULTED` | 机械臂发生无法自动恢复的故障；停止自动任务并人工排障 |
| `failure_category` | `uint8` | `0–5` | 技术失败分类，仅用于统计和诊断，不得替代 `outcome` 参与业务决策；完整枚举见附录 A |
| `failed_state` | `string` | 成功时通常为空 | 首次确定失败的内部执行阶段，仅用于诊断 |
| `detail` | `string` | 自由文本 | 详细原因、恢复结果和建议；内容可能随实现更新，不得用于程序分支 |
| `returned_to_stowed` | `bool` | `true` / `false` | 返回 Result 时是否已经确认处于 STOWED；抓取成功停在 CARRY 时为 `false`，不代表任务失败 |
| `final_task_state` | `uint8` | 使用与 `/arm/task_status.state` 相同的枚举定义 | 返回 Result 时的机械臂任务状态 |
| `payload_state` | `uint8` | 使用与 `/arm/task_status.payload_state` 相同的枚举定义 | 返回 Result 时的载荷状态 |
| `class_name` | `string` | `yellow_cube`、`zucchini`、`bowl`；识别前失败时可能为空 | 自动识别出的目标类别 |
| `estimated_center` | `geometry_msgs/PointStamped` | `success=true` 时有效 | 精观测或微调后的物体中心估计 |
| `grasp_pose` | `geometry_msgs/PoseStamped` | `success=true` 时有效 | 最终选中的 TCP 抓取位姿 |
| `pregrasp_pose` | `geometry_msgs/PoseStamped` | `success=true` 时有效 | 最终选中的 TCP 预抓取位姿 |
| `pregrasp_distance_m` | `float64` | `success=true` 时有效，单位为米 | 物体策略选中的预抓取距离或离地间隙 |
| `grasp_distance_m` | `float64` | `success=true` 时有效，单位为米 | 物体策略选中的抓取深度或离地间隙 |
| `grasp_yaw_degrees` | `float64` | `success=true` 时有效，单位为度 | 最终抓取 yaw |
| `approach_tilt_degrees` | `float64` | `success=true` 时有效，单位为度；当前生产策略通常为 `0` | 最终 approach 倾角 |


位姿、距离和识别结果只应在 `success=true` 时作为可靠结果使用。失败时这些字段仍会返回，但可能保留默认值或不完整的中间结果；空坐标系、全零四元数等不能作为有效位姿使用。调用方应根据 `outcome` 和最终状态处理失败，不应根据这些几何字段自行复现机械臂轨迹。

### 4.6 Goal 调用示例
#### 命令行
```bash
ros2 action send_goal /arm/tasks/pick_object \
  d1_interfaces/action/PickObject \
  "{target: {header: {frame_id: target_source_frame}, point: {x: 0.20, y: 0.35, z: -0.15}}, stop_after: 4}" \
  --feedback
```

注意：

+ `--feedback` 是 `ros2 action send_goal` 命令行工具的显示选项，用于在终端持续打印机械臂返回的 Feedback。它不是 `PickObject` 的 Goal 字段，不会作为请求参数发送给机械臂，也不会改变任务的执行逻辑。省略该选项仍会正常发送任务并等待最终 Result，只是不在终端显示执行过程反馈。使用 Python 或 C++ 客户端时，应通过 Action 客户端的 Feedback 回调接收这些信息，而不是传入 `--feedback`。
+ `target_source_frame` 是格式占位符，实际调用必须替换为目标数据产生时所属的 frame。例如，目标由相机感知节点直接产生时，应使用对应相机坐标系；目标由融合定位模块在 `map` 或 `odom` 中产生时，应使用相应的 `map` 或 `odom` frame。调用方不应仅为适配机械臂接口而预先把目标转换到 `base_link`；机械臂模块作为数据使用方，负责通过 TF 将原始目标转换到规划坐标系。
+ 示例坐标仅用于说明消息格式，不能直接作为其他现场的安全抓取点。实际调用必须使用现场估计值，并保证 `frame_id`、时间戳和坐标数值来自同一份目标数据。

#### Python（`rclpy`）
下面的最小客户端会等待 Action Server、发送 Goal、打印每一帧 Feedback，并在任务结束后读取 Result：

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from d1_interfaces.action import PickObject


class PickObjectClient(Node):
    def __init__(self):
        super().__init__('pick_object_client')
        self._client = ActionClient(
            self, PickObject, '/arm/tasks/pick_object')

    def feedback_callback(self, message):
        feedback = message.feedback
        self.get_logger().info(
            f'phase={feedback.current_state}, '
            f'progress={feedback.progress:.2f}, '
            f'detail={feedback.detail}')

    def execute(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('pick_object Action Server is unavailable')

        goal = PickObject.Goal()
        goal.target.header.frame_id = 'target_source_frame'
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.target.point.x = 0.20
        goal.target.point.y = 0.35
        goal.target.point.z = -0.15
        goal.stop_after = PickObject.Goal.GRASP_AND_CARRY

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('pick_object Goal was rejected; query /arm/task_status')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def main():
    rclpy.init()
    node = PickObjectClient()
    try:
        result = node.execute()
        node.get_logger().info(
            f'success={result.success}, outcome={result.outcome}, '
            f'final_task_state={result.final_task_state}, '
            f'payload_state={result.payload_state}, detail={result.detail}')

        if result.outcome == PickObject.Result.OUTCOME_SUCCESS:
            node.get_logger().info('Pick succeeded; the payload is ready to carry')
        elif result.outcome == PickObject.Result.OUTCOME_REPOSITION_REQUIRED:
            node.get_logger().warning('Reposition the mobile platform before retrying')
        else:
            node.get_logger().error('The arm is faulted; stop automatic operation')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

示例中的 `target_source_frame`、时间戳和坐标必须替换为目标数据自身的 frame、采集时间和位置。`execute()` 返回的是强类型 `PickObject.Result` 对象，不是 JSON 或 YAML 字符串。

#### C++（`rclcpp_action`）
```cpp
#include <chrono>
#include <iostream>
#include <memory>

#include "d1_interfaces/action/pick_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using PickObject = d1_interfaces::action::PickObject;
using PickGoalHandle = rclcpp_action::ClientGoalHandle<PickObject>;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_object_client");
  auto client = rclcpp_action::create_client<PickObject>(
    node, "/arm/tasks/pick_object");

  if (!client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node->get_logger(), "pick_object Action Server is unavailable");
    rclcpp::shutdown();
    return 1;
  }

  PickObject::Goal goal;
  goal.target.header.frame_id = "target_source_frame";
  goal.target.header.stamp = node->now();
  goal.target.point.x = 0.20;
  goal.target.point.y = 0.35;
  goal.target.point.z = -0.15;
  goal.stop_after = PickObject::Goal::GRASP_AND_CARRY;

  rclcpp_action::Client<PickObject>::SendGoalOptions options;
  options.feedback_callback =
    [node](PickGoalHandle::SharedPtr,
      const std::shared_ptr<const PickObject::Feedback> feedback) {
      RCLCPP_INFO(
        node->get_logger(), "phase=%s progress=%.2f detail=%s",
        feedback->current_state.c_str(), feedback->progress,
        feedback->detail.c_str());
    };

  auto send_future = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, send_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send pick_object Goal");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = send_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      node->get_logger(), "pick_object Goal was rejected; query /arm/task_status");
    rclcpp::shutdown();
    return 1;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed while waiting for pick_object Result");
    rclcpp::shutdown();
    return 1;
  }

  const auto wrapped = result_future.get();
  const auto & result = *wrapped.result;
  RCLCPP_INFO(
    node->get_logger(),
    "success=%s outcome=%u final_task_state=%u payload_state=%u detail=%s",
    result.success ? "true" : "false",
    static_cast<unsigned>(result.outcome),
    static_cast<unsigned>(result.final_task_state),
    static_cast<unsigned>(result.payload_state), result.detail.c_str());

  if (result.outcome == PickObject::Result::OUTCOME_SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Pick succeeded; the payload is ready to carry");
  } else if (result.outcome == PickObject::Result::OUTCOME_REPOSITION_REQUIRED) {
    RCLCPP_WARN(node->get_logger(), "Reposition the mobile platform before retrying");
  } else {
    RCLCPP_ERROR(node->get_logger(), "The arm is faulted; stop automatic operation");
  }

  rclcpp::shutdown();
  return 0;
}
```

示例中的 `target_source_frame`、时间戳和坐标同样必须替换为上游目标数据的真实值。`wrapped.result` 指向强类型 `PickObject::Result` 对象。

### 4.7 Feedback 返回示例
以下是一帧完整的过程反馈。各字段会随执行阶段更新，具体值和说明文本以实际返回为准。

```yaml
Feedback:
  current_state: MOVE_PREGRASP
  progress: 0.55
  detail: moving to selected pregrasp pose
```

该反馈只表示当前执行阶段，不表示抓取已经成功；调用方必须等待最终 Result。这里的 `Feedback:` 是命令行显示标签，不属于 Feedback 消息字段。

### 4.8 Result 返回示例
以下为抓取成功的完整结果示例，包含全部字段及嵌套结构。失败时返回相同字段结构，字段含义、有效条件及处理方式见第 4.5 节的 Result 参数表。

数值、时间戳和 `detail` 文本仅用于说明格式，并非某次实测日志或可直接使用的运动目标；实际值以接口返回为准。示例展示 Result 消息本体；命令行额外打印的 `Result:`、Goal ID 和 `Goal finished with status` 不属于 Result 字段。

```yaml
success: true
outcome: 0
failure_category: 0
failed_state: ""
detail: "yellow_cube grasped, carried and mechanically verified"
returned_to_stowed: false
final_task_state: 2
payload_state: 1
class_name: "yellow_cube"
estimated_center:
  header:
    stamp:
      sec: 1788700000
      nanosec: 0
    frame_id: "base_link"
  point:
    x: 0.2
    y: 0.35
    z: -0.1
grasp_pose:
  header:
    stamp:
      sec: 1788700000
      nanosec: 0
    frame_id: "base_link"
  pose:
    position:
      x: 0.2
      y: 0.35
      z: -0.12
    orientation:
      x: 1
      y: 0
      z: 0
      w: 0
pregrasp_pose:
  header:
    stamp:
      sec: 1788700000
      nanosec: 0
    frame_id: "base_link"
  pose:
    position:
      x: 0.2
      y: 0.35
      z: -0.06
    orientation:
      x: 1
      y: 0
      z: 0
      w: 0
pregrasp_distance_m: 0.04
grasp_distance_m: -0.02
grasp_yaw_degrees: 0
approach_tilt_degrees: 0
```

## 5. 投放接口 `/arm/tasks/drop_object`
### 5.1 工作流程概述
`drop_object` 将传入的目标点解释为垃圾桶底面中心，在该目标点上方（沿重力反方向）搜索可行释放位姿，必要时在配置允许范围内进行小幅横向调整。机械臂会在运动前检查去程和释放后的回程是否可行，随后移动至选定释放位姿、张开夹爪，并返回收纳（STOWED）姿态，最后返回任务结果。

任务结束后，接口通过最终结果 Result 中的 `outcome` 字段，指示调用方下一步应开始下一轮抓取、调整机器人载体位置后重试、重新指定投放点，还是停止自动任务并人工排障，同时返回机械臂的最终状态和载荷状态。

### 5.2 接口描述
| 项目 | 内容 |
| --- | --- |
| Action 名称 | `/arm/tasks/drop_object` |
| Action 类型 | `d1_interfaces/action/DropObject` |
| 生产任务前置状态 | `READY_CARRY` |
| 空载验收前置状态 | `READY_STOWED` |
| 成功后状态 | `READY_STOWED` |
| 成功后载荷状态 | `PAYLOAD_EMPTY` |


### 5.3 任务请求（Goal）参数
| 字段 | 类型 | 必填 | 取值与说明 |
| --- | --- | --- | --- |
| `target` | `geometry_msgs/PointStamped` | 是 | 调用方估计的垃圾桶底部中心；不是桶沿、所抓物体中心或 TCP 释放位置 |
| `target.header.frame_id` | `string` | 是 | 投放目标产生时所属的原始坐标系；必须存在从该坐标系到机械臂规划坐标系的 TF |
| `target.header.stamp` | `Time` | 否 | 应沿用投放目标的原始时间戳；但当前实现始终使用最新 TF，具体限制见第 5.6 节 |
| `target.point.x/y/z` | `float64` | 是 | 垃圾桶底部中心，单位为米，必须为有限数值 |


垃圾桶中心与机械臂 `base_link` 的推荐水平距离为 `0.35–0.45 m`。该范围仅为建议，不是请求格式约束。

机器人载体平台（如 Go2）的 XY 包络及其向外扩展 5 cm 的区域均属于禁投区。目标点落入禁投区时，机械臂不会运动，并返回 `OUTCOME_NEW_TARGET_REQUIRED`。禁投区由部署配置中的载体平台几何建立。

### 5.4 反馈（Feedback）参数
| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `current_state` | `string` | 当前投放的内部执行阶段，与 `/arm/task_status.state` 的公共状态不同；例如 `PLAN_RELEASE`、`MOVE_RELEASE` 或 `STOWED` |
| `progress` | `float32` | 按执行阶段估算的进度，范围为 `0.0–1.0`，不表示剩余执行时间 |
| `detail` | `string` | 当前阶段的人类可读说明，不应用于程序分支 |


### 5.5 结果（Result）参数
| 字段 | 类型 | 取值或有效条件 | 含义与调用方处理方式 |
| --- | --- | --- | --- |
| `success` | `bool` | `true` / `false` | 是否完成释放并确认返回 STOWED。快速查看可使用该字段，程序决策应以 `outcome` 为准 |
| `outcome` | `uint8` | `0 OUTCOME_SUCCESS` | 投放成功；机械臂已返回 `READY_STOWED / PAYLOAD_EMPTY`，可继续下一项任务 |
|  |  | `1 OUTCOME_REPOSITION_REQUIRED` | 当前相对位置没有安全可达的释放往返方案；携物任务通常恢复为 `READY_CARRY / PAYLOAD_HELD`，调用方应调整机器人载体与垃圾桶的相对位置后重试 |
|  |  | `2 OUTCOME_ARM_FAULTED` | 机械臂发生无法自动恢复的故障；停止自动任务并人工排障 |
|  |  | `3 OUTCOME_NEW_TARGET_REQUIRED` | 投放目标无效或位于禁投区；重新估计或指定垃圾桶底部中心，不要原样重发 |
| `failure_category` | `uint8` | `0–5` | 技术失败分类，仅用于统计和诊断，不得替代 `outcome` 参与业务决策；完整枚举见附录 A |
| `failed_state` | `string` | 成功时通常为空 | 首次确定失败的内部执行阶段，仅用于诊断 |
| `detail` | `string` | 自由文本 | 详细原因、候选统计和恢复结果；内容可能随实现更新，不得用于程序分支 |
| `returned_to_stowed` | `bool` | `true` / `false` | 返回 Result 时是否已确认处于 STOWED |
| `final_task_state` | `uint8` | 使用与 `/arm/task_status.state` 相同的枚举定义 | 返回 Result 时的机械臂任务状态 |
| `payload_state` | `uint8` | 使用与 `/arm/task_status.payload_state` 相同的枚举定义 | 返回 Result 时的载荷状态 |
| `release_pose` | `geometry_msgs/PoseStamped` | `success=true` 时有效 | 实际选中的 TCP 释放位姿 |
| `height_offset_m` | `float64` | `success=true` 时有效，单位为米 | 成功候选相对基准释放高度的偏移 |
| `release_yaw_degrees` | `float64` | `success=true` 时有效，单位为度 | 成功候选相对参考方向的 yaw 偏移 |


`release_pose`、`height_offset_m` 和 `release_yaw_degrees` 只在 `success=true` 时有效。失败时这些字段仍会返回，但可能保留默认值或不完整的中间结果，不能据此执行运动。调用方应根据 `outcome`、`final_task_state` 和 `payload_state` 决定下一步，不应假设投放失败后一定为空载或已经返回 STOWED。



### 5.6 Goal 调用示例
#### 命令行
```bash
ros2 action send_goal /arm/tasks/drop_object \
  d1_interfaces/action/DropObject \
  "{target: {header: {frame_id: target_source_frame}, point: {x: 0.00, y: 0.40, z: -0.28}}}" \
  --feedback
```

这里的 `--feedback` 同样只是 ROS 2 命令行工具的显示选项，用于在终端持续打印投放过程的 Feedback；它不属于 `DropObject` 的 Goal 参数，也不影响投放任务本身。省略后仍会正常获得最终 Result。

`target_source_frame` 是格式占位符，实际调用必须替换为垃圾桶位置数据产生时所属的 frame。示例坐标仅用于说明消息格式，调用前必须替换为现场估计的垃圾桶底部中心。

当前 `drop_object` 会使用目标源坐标系到规划坐标系的最新 TF，而不会按非零 `target.header.stamp` 查询历史 TF。因此，投放目标来自随机器人运动的坐标系时，应在机器人停止并确认目标仍然有效后立即调用；也可以由上游融合定位模块直接提供位于 `map`、`odom` 等稳定坐标系中的目标。该限制不改变 `frame_id` 的语义，调用方仍应填写目标数据的原始坐标系。

#### Python（`rclpy`）
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from d1_interfaces.action import DropObject


class DropObjectClient(Node):
    def __init__(self):
        super().__init__('drop_object_client')
        self._client = ActionClient(
            self, DropObject, '/arm/tasks/drop_object')

    def feedback_callback(self, message):
        feedback = message.feedback
        self.get_logger().info(
            f'phase={feedback.current_state}, '
            f'progress={feedback.progress:.2f}, '
            f'detail={feedback.detail}')

    def execute(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('drop_object Action Server is unavailable')

        goal = DropObject.Goal()
        goal.target.header.frame_id = 'target_source_frame'
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.target.point.x = 0.00
        goal.target.point.y = 0.40
        goal.target.point.z = -0.28

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('drop_object Goal was rejected; query /arm/task_status')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def main():
    rclpy.init()
    node = DropObjectClient()
    try:
        result = node.execute()
        node.get_logger().info(
            f'success={result.success}, outcome={result.outcome}, '
            f'returned_to_stowed={result.returned_to_stowed}, '
            f'final_task_state={result.final_task_state}, '
            f'payload_state={result.payload_state}, detail={result.detail}')

        if result.outcome == DropObject.Result.OUTCOME_SUCCESS:
            node.get_logger().info('Drop succeeded; the arm is ready for the next task')
        elif result.outcome == DropObject.Result.OUTCOME_REPOSITION_REQUIRED:
            node.get_logger().warning('Reposition the mobile platform before retrying')
        elif result.outcome == DropObject.Result.OUTCOME_NEW_TARGET_REQUIRED:
            node.get_logger().warning('Provide a new drop target; do not resend unchanged')
        else:
            node.get_logger().error('The arm is faulted; stop automatic operation')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`execute()` 返回强类型 `DropObject.Result` 对象。调用方应根据 `result.outcome` 决定调整机器人载体、重新指定投放点或停止自动任务。

#### C++（`rclcpp_action`）
```cpp
#include <chrono>
#include <memory>

#include "d1_interfaces/action/drop_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using DropObject = d1_interfaces::action::DropObject;
using DropGoalHandle = rclcpp_action::ClientGoalHandle<DropObject>;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("drop_object_client");
  auto client = rclcpp_action::create_client<DropObject>(
    node, "/arm/tasks/drop_object");

  if (!client->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node->get_logger(), "drop_object Action Server is unavailable");
    rclcpp::shutdown();
    return 1;
  }

  DropObject::Goal goal;
  goal.target.header.frame_id = "target_source_frame";
  goal.target.header.stamp = node->now();
  goal.target.point.x = 0.00;
  goal.target.point.y = 0.40;
  goal.target.point.z = -0.28;

  rclcpp_action::Client<DropObject>::SendGoalOptions options;
  options.feedback_callback =
    [node](DropGoalHandle::SharedPtr,
      const std::shared_ptr<const DropObject::Feedback> feedback) {
      RCLCPP_INFO(
        node->get_logger(), "phase=%s progress=%.2f detail=%s",
        feedback->current_state.c_str(), feedback->progress,
        feedback->detail.c_str());
    };

  auto send_future = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, send_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send drop_object Goal");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = send_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      node->get_logger(), "drop_object Goal was rejected; query /arm/task_status");
    rclcpp::shutdown();
    return 1;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed while waiting for drop_object Result");
    rclcpp::shutdown();
    return 1;
  }

  const auto wrapped = result_future.get();
  const auto & result = *wrapped.result;
  RCLCPP_INFO(
    node->get_logger(),
    "success=%s outcome=%u returned_to_stowed=%s "
    "final_task_state=%u payload_state=%u detail=%s",
    result.success ? "true" : "false",
    static_cast<unsigned>(result.outcome),
    result.returned_to_stowed ? "true" : "false",
    static_cast<unsigned>(result.final_task_state),
    static_cast<unsigned>(result.payload_state), result.detail.c_str());

  if (result.outcome == DropObject::Result::OUTCOME_SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Drop succeeded; the arm is ready for the next task");
  } else if (result.outcome == DropObject::Result::OUTCOME_REPOSITION_REQUIRED) {
    RCLCPP_WARN(node->get_logger(), "Reposition the mobile platform before retrying");
  } else if (result.outcome == DropObject::Result::OUTCOME_NEW_TARGET_REQUIRED) {
    RCLCPP_WARN(node->get_logger(), "Provide a new drop target; do not resend unchanged");
  } else {
    RCLCPP_ERROR(node->get_logger(), "The arm is faulted; stop automatic operation");
  }

  rclcpp::shutdown();
  return 0;
}
```

`wrapped.result` 指向强类型 `DropObject::Result` 对象。示例中的 frame、时间戳和坐标必须替换为上游投放目标的真实值。

### 5.7 Feedback 返回示例
以下是一帧完整的过程反馈。各字段会随执行阶段更新，具体值和说明文本以实际返回为准。

```yaml
Feedback:
  current_state: MOVE_RELEASE
  progress: 0.50
  detail: moving to selected release pose
```

该反馈只表示当前执行阶段，不表示投放已经成功；调用方必须等待最终 Result。这里的 `Feedback:` 是命令行显示标签，不属于 Feedback 消息字段。

### 5.8 Result 返回示例
以下为投放成功的完整结果示例，包含全部字段及嵌套结构。失败时返回相同字段结构，字段含义、有效条件及处理方式见第 5.5 节的 Result 参数表。

数值、时间戳和 `detail` 文本仅用于说明格式，并非某次实测日志或可直接使用的运动目标；实际值以接口返回为准。示例展示 Result 消息本体；命令行额外打印的 `Result:`、Goal ID 和 `Goal finished with status` 不属于 Result 字段。

```yaml
success: true
outcome: 0
failure_category: 0
failed_state: ""
detail: "held object released above trash bin and arm returned to STOWED"
returned_to_stowed: true
final_task_state: 1
payload_state: 0
release_pose:
  header:
    stamp:
      sec: 1788700000
      nanosec: 0
    frame_id: "base_link"
  pose:
    position:
      x: 0
      y: 0.4
      z: -0.1
    orientation:
      x: 1
      y: 0
      z: 0
      w: 0
height_offset_m: 0
release_yaw_degrees: 0
```

## 6. 状态接口 `/arm/task_status`
### 6.1 工作流程概述
`/arm/task_status` 用于返回机械臂当前的全局状态，是保留最近值的系统状态流，不是某次 Action 的返回值。调用方连接中断后重新连接、等待任务超时或 Goal 被拒绝时，可通过该 Topic 复核全局状态，判断机械臂仍在执行、已经恢复还是进入故障。

### 6.2 接口描述
| 项目 | 内容 |
| --- | --- |
| Topic 名称 | `/arm/task_status` |
| 消息类型 | `d1_interfaces/msg/ArmTaskStatus` |
| QoS | Reliable、Transient Local、Keep Last 1 |
| 发布方式 | 状态变化时发布；新订阅者会立即收到最近一次状态 |


### 6.3 消息字段
| 字段 | 类型 | 取值 | 含义与调用方处理方式 |
| --- | --- | --- | --- |
| `stamp` | `builtin_interfaces/Time` | ROS 2 时间戳 | 本条状态的生成时间 |
| `state` | `uint8` | `0–7` | 当前公共业务状态；完整枚举和允许操作见第 7.1 节，状态转移见第 7.2 节 |
| `payload_state` | `uint8` | `0 PAYLOAD_EMPTY` | 已确认空载 |
|  |  | `1 PAYLOAD_HELD` | 已确认持有物体 |
|  |  | `2 PAYLOAD_UNKNOWN` | 无法可靠判断；调用方不得自行假设为空载或持物 |
| `canonical_pose` | `uint8` | `0 POSE_STOWED` | 已确认处于标准收纳姿态 |
|  |  | `1 POSE_CARRY` | 已确认处于标准携带姿态 |
|  |  | `2 POSE_OTHER` | 已知不在标准 STOWED 或 CARRY 姿态 |
|  |  | `3 POSE_UNKNOWN` | 无法可靠判断当前是否处于标准姿态 |
| `active_operation` | `string` | `pick_object`、`drop_object` 或空字符串 | 当前正在执行的任务；空闲时通常为空 |
| `active_phase` | `string` | 内部执行阶段或空字符串 | 当前 Action 的内部阶段，仅用于进度显示和诊断 |
| `failure_code` | `string` | 正常稳定状态下为空 | 可供程序记录和日志检索的故障标识 |
| `detail` | `string` | 自由文本 | 当前状态、阶段或故障的详细说明；不得用于程序分支 |


### 6.4 订阅示例
#### 命令行
持续查看状态：

```bash
ros2 topic echo /arm/task_status --qos-reliability reliable --qos-durability transient_local
```

只读取最近一次状态：

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

#### Python（`rclpy`）
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from d1_interfaces.msg import ArmTaskStatus


class ArmStatusSubscriber(Node):
    def __init__(self):
        super().__init__('arm_status_subscriber')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscription = self.create_subscription(
            ArmTaskStatus,
            '/arm/task_status',
            self.status_callback,
            qos,
        )

    def status_callback(self, message):
        self.get_logger().info(
            f'state={message.state}, payload_state={message.payload_state}, '
            f'canonical_pose={message.canonical_pose}, '
            f'operation={message.active_operation}, phase={message.active_phase}, '
            f'failure_code={message.failure_code}, detail={message.detail}')

        if message.state == ArmTaskStatus.FAULTED:
            self.get_logger().error('The arm is faulted; stop automatic operation')


def main():
    rclpy.init()
    node = ArmStatusSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### C++（`rclcpp`）
```cpp
#include <memory>

#include "d1_interfaces/msg/arm_task_status.hpp"
#include "rclcpp/rclcpp.hpp"

using ArmTaskStatus = d1_interfaces::msg::ArmTaskStatus;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("arm_status_subscriber");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  auto subscription = node->create_subscription<ArmTaskStatus>(
    "/arm/task_status", qos,
    [node](const ArmTaskStatus::SharedPtr message) {
      RCLCPP_INFO(
        node->get_logger(),
        "state=%u payload_state=%u canonical_pose=%u "
        "operation=%s phase=%s failure_code=%s detail=%s",
        static_cast<unsigned>(message->state),
        static_cast<unsigned>(message->payload_state),
        static_cast<unsigned>(message->canonical_pose),
        message->active_operation.c_str(), message->active_phase.c_str(),
        message->failure_code.c_str(), message->detail.c_str());

      if (message->state == ArmTaskStatus::FAULTED) {
        RCLCPP_ERROR(node->get_logger(), "The arm is faulted; stop automatic operation");
      }
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

两种客户端都使用与发布端一致的 Reliable、Transient Local、Keep Last 1 QoS，因此即使订阅发生在状态发布之后，也能立即收到最近一次状态。

### 6.5 消息示例
以下示例展示消息的全部字段，包括嵌套字段。数值、时间戳和 `detail` 文本仅用于说明格式，并非某次实测日志或可直接使用的运动目标；实际值以接口返回为准。

机械臂空载并处于 STOWED：

```yaml
stamp:
  sec: 1788700000
  nanosec: 0
state: 1
payload_state: 0
canonical_pose: 0
active_operation: ""
active_phase: ""
failure_code: ""
detail: "Verified canonical STOWED feedback and base planning scene"
```

## 7. 公共状态与转移规则
本节用于查询状态数值、允许操作、完整转移条件和统一的任务结果处理方式。首次调用时只需先理解第 1 节的整体流程，遇到状态或异常问题时再查阅本节。

### 7.1 公共业务状态
`/arm/task_status.state` 会发布以下公共业务状态。这些状态面向调用方，与机械臂内部执行阶段不同。

| 状态 | 值 | 含义 | 调用方允许操作 |
| --- | --- | --- | --- |
| `INITIALIZING` | `0` | 模块正在检查反馈、标准姿态和规划场景 | 等待，不发送任务 |
| `READY_STOWED` | `1` | 机械臂空载并处于标准收纳姿态 | 可调用 `pick_object`；也可空载调用 `drop_object` 进行部署验收 |
| `READY_CARRY` | `2` | 机械臂已确认持物并处于标准携带姿态 | 可调用 `drop_object` |
| `PICKING` | `3` | 正在执行抓取 | 等待，不发送新任务 |
| `DROPPING` | `4` | 正在执行投放 | 等待，不发送新任务 |
| `RECOVERING_TO_STOWED` | `5` | 正在自动恢复至空载收纳姿态 | 等待恢复完成 |
| `RECOVERING_TO_CARRY` | `6` | 正在保留载荷并恢复至携带姿态 | 等待恢复完成 |
| `FAULTED` | `7` | 出现无法自动处理的执行、通信或状态故障 | 停止自动任务并人工排障 |


载荷状态用于表示机械臂对当前持物情况的判断：

| 载荷状态 | 值 | 含义 |
| --- | --- | --- |
| `PAYLOAD_EMPTY` | `0` | 已确认空载 |
| `PAYLOAD_HELD` | `1` | 已确认持有物体 |
| `PAYLOAD_UNKNOWN` | `2` | 无法可靠判断；调用方不得自行假设为空载或持物 |


### 7.2 主要状态转移
| 当前状态 | 触发条件 | 后续状态 | 说明 |
| --- | --- | --- | --- |
| `INITIALIZING` | 启动检查通过 | `READY_STOWED` | 已收到新鲜关节反馈、确认 STOWED，并建立基础规划场景 |
| `INITIALIZING` | 启动检查失败 | `FAULTED` | 不接受抓放任务 |
| `READY_STOWED` | 接受 `pick_object` | `PICKING` | 开始抓取流程 |
| `PICKING` | 抓取成功并通过夹持复检 | `READY_CARRY` | 调用方可继续导航并投放 |
| `PICKING` | 发生可恢复的业务失败 | `RECOVERING_TO_STOWED` → `READY_STOWED` | 调用方根据 `outcome` 调整位置后重试 |
| `READY_CARRY` | 接受 `drop_object` | `DROPPING` | 开始携物投放流程 |
| `READY_STOWED` | 接受空载 `drop_object` | `DROPPING` | 仅用于部署验收 |
| `DROPPING` | 投放成功 | `READY_STOWED` | 物体已释放，机械臂已收纳 |
| `DROPPING` | 携物且在释放前发生可恢复失败 | `RECOVERING_TO_CARRY` → `READY_CARRY` | 调整机器人载体或投放目标后重试 |
| `DROPPING` | 空载验收发生可恢复失败 | `RECOVERING_TO_STOWED` → `READY_STOWED` | 可修正目标后重新验收 |
| 任意状态 | 发生无法自动恢复的执行、通信或状态故障 | `FAULTED` | 停止自动调用并人工排障 |


启动阶段只有在收到新鲜关节反馈、确认 STOWED 并建立基础规划场景后，才会进入 `READY_STOWED`。若当前姿态位于配置的近 STOWED 范围内，系统可能执行一次有界启动恢复；偏差过大时不会运动，并进入 `FAULTED`。

## 8. 调用注意事项
### 8.1 如何处理任务结果
1. Action 调用完成后，首先读取 `outcome`，根据稳定枚举决定下一步。
2. `success` 适合快速查看结果，但调用方的业务分支应以 `outcome` 为准。
3. `failure_category`、`failed_state`、`failure_code` 和 `detail` 用于记录与诊断，不得替代 `outcome` 参与业务分支。
4. 同一 `failure_category` 可能因自动恢复是否成功而产生不同 `outcome`，因此不能只根据技术失败分类决定是否重试。

### 8.2 并发调用、超时、取消与重试
+ 同一时间只允许执行一个 `pick_object` 或 `drop_object`。
+ 客户端等待超时不代表机械臂没有执行。应先查询 `/arm/task_status`，禁止直接重发。
+ 收到 `OUTCOME_REPOSITION_REQUIRED` 或 `OUTCOME_NEW_TARGET_REQUIRED` 后，应根据 Result 中的最终状态和载荷状态，调整机器人载体位置或重新指定投放点，再发送新请求。
+ 收到 `OUTCOME_ARM_FAULTED` 或状态变为 `FAULTED` 后，不得继续调用抓放接口。

#### 如何取消任务
中止任务需要向机械臂发送 Action 取消请求。当前命令行工具支持在任务被接受后按 `Ctrl+C` 请求取消；Python/C++ 调用方需自行实现取消操作。仅关闭客户端或断开网络，不保证机械臂停止。取消被处理后，机械臂进入 `FAULTED`。

ROS 2 已提供取消方法。保存任务被接受时返回的 `goal_handle`（任务句柄），在用户点击“取消”或上层决定中止时调用即可。以下为接入现有客户端的代码片段，抓取和投放均适用；其中 `node` 是客户端节点，C++ 中的 `client` 是发送该任务的 Action 客户端。

Python：

```python
def on_cancel_response(future):
    try:
        response = future.result()
        if response is not None and response.goals_canceling:
            node.get_logger().info('Cancellation request accepted; waiting for the task to finish')
        else:
            node.get_logger().warning('Cancellation was not accepted; check the original task result')
    except Exception as exc:
        node.get_logger().error(f'Failed to confirm cancellation: {exc}')

# Call when cancellation is needed; goal_handle must belong to an accepted goal.
cancel_future = goal_handle.cancel_goal_async()
cancel_future.add_done_callback(on_cancel_response)
```

C++：

```cpp
// Call when cancellation is needed; goal_handle must belong to an accepted goal.
client->async_cancel_goal(
  goal_handle,
  [node](auto response) {
    if (response && !response->goals_canceling.empty()) {
      RCLCPP_INFO(node->get_logger(), "Cancellation request accepted; waiting for the task to finish");
    } else {
      RCLCPP_WARN(node->get_logger(), "Cancellation was not accepted; check the original task result");
    }
  });
```

发送请求后，应让客户端继续运行并处理 ROS 回调，等待原任务的最终结果；不要立即销毁节点或退出程序。上述示例使用异步回调，不需要在已有回调中再次调用 `spin_until_future_complete`。

“取消请求已接受”只表示机械臂开始处理取消。最终应确认原任务的 Action 终态为 `CANCELED`（Python 中为 `GoalStatus.STATUS_CANCELED`，C++ 中为 `rclcpp_action::ResultCode::CANCELED`）。如果取消前任务已经结束，则按该任务实际返回的最终结果处理；若通信中断、无法取得结果，不得假定机械臂已经停止，应在恢复连接后查询 `/arm/task_status`。

### 8.3 目标坐标系与时间戳要求
两个 Action 的目标均使用 `geometry_msgs/PointStamped`：

```text
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
```

+ `frame_id` 必填，表示目标数据产生时所属的原始坐标系，并且必须存在从该坐标系到机械臂规划坐标系的 TF。
+ 数据由相机、定位、导航或其他感知模块产生时，应保留该数据自身的 frame；不要仅为适配机械臂接口而由调用方提前转换到 `base_link`。
+ 机械臂模块作为数据使用方，负责通过 TF 将目标转换到自己的规划坐标系。
+ `x/y/z` 单位为米，必须是有限数值，并且必须与 `frame_id` 和 `stamp` 描述同一份数据。
+ 应优先沿用数据产生时的原始时间戳。`stamp=0` 表示调用方明确要求使用最新 TF。
+ `pick_object` 支持在观测阶段查询非零时间戳对应的 TF；当前 `drop_object` 始终使用最新 TF，因此移动平台应在停止后确认投放目标仍有效再调用，或使用 `map`、`odom` 等稳定坐标系。
+ `pick_object.target` 与 `drop_object.target` 的业务含义不同，不能混用。

### 8.4 命令行中的 SUCCEEDED、ABORTED、CANCELED 和 Goal rejected 是什么意思
ROS 2 Action 终态与机械臂业务结果属于两个不同层次：

| ROS 2 Action 终态 | 是否有完整 Result | 调用方处理方式 |
| --- | --- | --- |
| `SUCCEEDED` | 是 | 本次业务成功，`result.success=true` |
| `ABORTED` | 是 | 业务未完成；必须读取 `outcome`，判断机械臂已经安全恢复还是进入故障 |
| `CANCELED` | 是 | 当前实现会停止活动命令并进入 `FAULTED`，不会自动执行后续恢复 |
| Goal rejected | 否 | 请求格式不合法，或接收时存在并发/状态冲突；立即查询 `/arm/task_status` |


当前 `pick_object` 遇到生产任务状态冲突时，通常会接受 Goal 后返回包含完整原因的 `ABORTED`；`drop_object` 的并发或状态冲突可能直接拒绝 Goal。调用方必须兼容这两种表现。

# 补充参考信息
本部分用于完整消息查询、客户端开发、日志分析和异常排查。普通调用方无需阅读或理解本部分，也不影响正确调用抓取、投放和状态接口；需要使用内部阶段或诊断字段时再按需查询即可。

## A. 技术失败分类
### A.1 `pick_object.failure_category`
| 值 | 枚举 | 含义 |
| --- | --- | --- |
| `0` | `FAILURE_NONE` | 无失败 |
| `1` | `FAILURE_INCOMPLETE_INFORMATION` | 视觉、深度、TF、重力或状态信息不足 |
| `2` | `FAILURE_THEORETICALLY_INFEASIBLE` | 抓取策略或理论候选不可行 |
| `3` | `FAILURE_EXECUTION_ERROR` | 规划执行、控制器、通信、取消、恢复或内部错误 |
| `4` | `FAILURE_REPOSITION_REQUIRED` | 当前相对位置无安全可达方案 |
| `5` | `FAILURE_GRASP_NOT_SECURED` | CARRY 复检确认物体未被可靠保持 |


### A.2 `drop_object.failure_category`
| 值 | 枚举 | 含义 |
| --- | --- | --- |
| `0` | `FAILURE_NONE` | 无失败 |
| `1` | `FAILURE_INCOMPLETE_INFORMATION` | 载荷、附着物体、TF、重力或规划场景信息不足或不一致 |
| `2` | `FAILURE_THEORETICALLY_INFEASIBLE` | 理论释放约束无法满足 |
| `3` | `FAILURE_EXECUTION_ERROR` | 规划执行、夹爪、控制器、通信、取消、恢复或内部错误 |
| `4` | `FAILURE_REPOSITION_REQUIRED` | 没有安全可达的释放往返方案 |
| `5` | `FAILURE_TARGET_IN_KEEP_OUT` | 目标位于机器人载体平台禁投区 |


## B. 内部执行阶段
### B.1 抓取阶段
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


### B.2 投放阶段
| `current_state` | 含义 |
| --- | --- |
| `CHECK_PRECONDITIONS` | 检查载荷、MoveIt 起始状态、TF、重力和载体平台禁投区 |
| `PLAN_RELEASE` | 搜索同时满足去程与回程要求的释放候选 |
| `MOVE_RELEASE` | 移动到选定释放位姿 |
| `RELEASE` | 完全张开夹爪并移除附着物体 |
| `RELEASE_HOLD` | 保持夹爪完全张开 |
| `STOWED` | 执行预验证的空载回程并返回 STOWED |
| `RECOVERING_TO_STOWED` | 空载任务失败后恢复 STOWED |
| `RECOVERING_TO_CARRY` | 携物任务在释放前失败后恢复 CARRY |


## C. 消息定义速查
### C.1 `PickObject.action`
```text
uint8 COMPUTE_ONLY=0
uint8 MOVE_PREGRASP=1
uint8 DESCEND=2
uint8 GRASP_AND_LIFT=3
uint8 GRASP_AND_CARRY=4
geometry_msgs/PointStamped target
uint8 stop_after
---
uint8 FAILURE_NONE=0
uint8 FAILURE_INCOMPLETE_INFORMATION=1
uint8 FAILURE_THEORETICALLY_INFEASIBLE=2
uint8 FAILURE_EXECUTION_ERROR=3
uint8 FAILURE_REPOSITION_REQUIRED=4
uint8 FAILURE_GRASP_NOT_SECURED=5
uint8 OUTCOME_SUCCESS=0
uint8 OUTCOME_REPOSITION_REQUIRED=1
uint8 OUTCOME_ARM_FAULTED=2
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
---
string current_state
float32 progress
string detail
```

### C.2 `DropObject.action`
```text
geometry_msgs/PointStamped target
---
uint8 FAILURE_NONE=0
uint8 FAILURE_INCOMPLETE_INFORMATION=1
uint8 FAILURE_THEORETICALLY_INFEASIBLE=2
uint8 FAILURE_EXECUTION_ERROR=3
uint8 FAILURE_REPOSITION_REQUIRED=4
uint8 FAILURE_TARGET_IN_KEEP_OUT=5
uint8 OUTCOME_SUCCESS=0
uint8 OUTCOME_REPOSITION_REQUIRED=1
uint8 OUTCOME_ARM_FAULTED=2
uint8 OUTCOME_NEW_TARGET_REQUIRED=3
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
---
string current_state
float32 progress
string detail
```

### C.3 `ArmTaskStatus.msg`
```text
builtin_interfaces/Time stamp

uint8 INITIALIZING=0
uint8 READY_STOWED=1
uint8 READY_CARRY=2
uint8 PICKING=3
uint8 DROPPING=4
uint8 RECOVERING_TO_STOWED=5
uint8 RECOVERING_TO_CARRY=6
uint8 FAULTED=7
uint8 state

uint8 PAYLOAD_EMPTY=0
uint8 PAYLOAD_HELD=1
uint8 PAYLOAD_UNKNOWN=2
uint8 payload_state

uint8 POSE_STOWED=0
uint8 POSE_CARRY=1
uint8 POSE_OTHER=2
uint8 POSE_UNKNOWN=3
uint8 canonical_pose

string active_operation
string active_phase
string failure_code
string detail
```

## D. 客户端代码依赖
| 项目 | 内容 |
| --- | --- |
| 接口包 | `d1_interfaces` |
| 通信框架 | ROS 2 Humble / DDS |
| 接口版本 | `0.1.0` |
| 坐标单位 | 米（m） |
| 角度单位 | Result 中的 yaw/tilt 使用度（°）；ROS Pose 姿态使用四元数 |


调用方只需依赖公共接口包 `d1_interfaces` 和所选语言对应的 ROS 2 客户端库，不应依赖 `d1_manipulation` 等内部实现包。

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

Python 客户端的 `package.xml` 至少声明：

```xml
<depend>d1_interfaces</depend>
<exec_depend>rclpy</exec_depend>

```

C++ 客户端的 `package.xml` 至少声明：

```xml
<depend>d1_interfaces</depend>
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>

```

C++ 客户端还需要在 `CMakeLists.txt` 中查找并链接这些依赖：

```cmake
find_package(d1_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(arm_task_client src/arm_task_client.cpp)
ament_target_dependencies(
  arm_task_client
  d1_interfaces
  rclcpp
  rclcpp_action
)
```

推荐的客户端调用顺序为：订阅并等待稳定状态 → 发送 Action Goal → 持续处理 Feedback → 等待 Result → 根据 `outcome` 决策。不要通过匹配 `detail` 文本实现业务分支；该字段仅用于日志和人工诊断，稳定的程序逻辑只应依赖枚举字段。







---

![画板](images/workflow_canvas.jpg)
