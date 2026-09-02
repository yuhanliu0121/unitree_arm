> 来源：宇树科技文档中心  
原始页面：[https://support.unitree.com/home/zh/developer/D1Arm_services](https://support.unitree.com/home/zh/developer/D1Arm_services)  
页面更新时间：2026-04-01 17:33:30
>

## 目录
+ [介绍](#介绍)
+ [规格参数](#规格参数)
+ [机械臂结构示意图](#机械臂结构示意图)
+ [接口协议](#接口协议)
+ [SDK 及示例程序的获取与使用](#sdk-及示例程序的获取与使用)
+ [机械臂 URDF 下载](#机械臂-urdf-下载)
+ [使用示例](#使用示例)
+ [D1-T 配置及使用](#d1-t-配置及使用)
+ [实例演示](#实例演示)

## 介绍
D1舵机机械臂上电自启动运行该服务，通过调用该服务的接口，实现机械臂控制功能。具体实现功能如下。

| 功能 | 说明 |
| --- | --- |
| 单个机械臂关节角度控制 | 通过控制机械臂单个关节的电机角度，实现对机械臂的控制 |
| 全部机械臂关节角度控制 | 通过控制机械臂全部关节的电机角度，实现对机械臂的控制 |
| 单个机械臂关节电机使能/卸力控制 | 控制关节电机状态为锁死/自由，自由状态下配合关节角度反馈接口，可实现拖动记忆示教 |
| 全部机械臂关节电机使能/卸力控制 | 控制所有关节电机状态为锁死/自由，自由状态下配合关节角度反馈接口，可实现拖动记忆示教 |
| 机械臂电机供电开关 | 控制是否给电机供电（可用做紧急停止） |
| 机械臂位姿归零 | 在任意位姿下机械臂回归零位 |
|  |  |
| 机械臂关节角度反馈 | 反馈机械臂关节实时角度 |
| 机械臂状态反馈 | 反馈机械臂的使能/卸力状态、上电状态 |
|  |  |
| 指令接收反馈 | 接收到控制指令后反馈，作为接收成功的标识 |
| 指令执行反馈 | 执行完控制指令后反馈，作为执行成功的标识 |


_**<u><font style="color:#DF2A3F;">实测修正：</font></u>**_

1. <font style="color:#DF2A3F;">“机械臂电机供电开关可用作紧急停止”不成立：</font>`<font style="color:#DF2A3F;">power=0 </font>`<font style="color:#DF2A3F;">在运动中、静止使能时、全部卸力后三种条件下均返回成功 ACK，但没有实际断电。紧急停止必须使用厂商认可的硬件方式。</font>
2. <font style="color:#DF2A3F;">“执行完控制指令后反馈”不成立：执行 ACK 可能在机械运动开始前返回，单关节实际运动时也可能完全没有执行 ACK。是否到位必须依据角度反馈判断。</font>

## 规格参数
### 结构参数
| 参数 | 数值 |
| --- | --- |
| 型号 | D1-550 |
| 重量 | 3152g |
| 自由度 | 6（自由度）+ 1（爪夹） |
| 臂长 | 550mm（不包含夹爪），670mm（包含夹爪） |
| 额定负载 | 500g（包含夹爪重量） |
| 工作半径 | 550mm |
| 各关节电机扭矩 | J0：3.3Nm；J1：3.3Nm；J2～J6：1.7Nm |
| 各关节运动范围 | J0：±135°；J1：±90°；J2：±90°；J3：±135°；J4：±90°；J5：±135°；夹爪行程：0～65mm |
| 电机类型 | 总线舵机 |


### 电气参数
| 电气参数 |  |
| --- | --- |
| 电源需求 | 24V 10A(15～48V) |
| 功率 | 240W |
| 控制器 | 集成（4 x Cortex-A55 1.8GHz ） |
| 通讯方式 | RJ45（Ethernet 100Mbps 通讯）+Type-C（串口调试） |
| 控制方式 | DDS订阅 |
| 控制周期 | 10Hz |


_**<u><font style="color:#DF2A3F;">实测补充：</font></u>**_<font style="color:#DF2A3F;">反馈标称 10Hz，当前设备长期实测约为 8.97Hz。控制算法不得假设严格的 100ms 周期，应使用时间戳和反馈闭环。</font>

## 机械臂结构示意图
<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/39019497/1783949060929-ef18e5cf-376a-4fae-9d04-0d48800fc4bc.png)

机械臂基本6轴+1夹爪，从底座为0开始计算，到夹爪为6。

## 接口协议
所有数据使用Json格式的字符串进行传输，包含seq（唯一识别码）、address（指令地址码）、funcode（指令功能码）、data（数据内容）。data内容可嵌套Json数据。

除主动上送信息 seq为10（写死），其余下发命令的seq都为调用端自动生成。为了区分回复消息序列，不需要做解析处理，只在回复的时候将此条命令的seq发送回去。

_**<u><font style="color:#DF2A3F;">seq 实测补充：</font></u>**_<font style="color:#DF2A3F;">服务不会按 seq 去重；两条内容和 seq 完全相同的请求会被执行两次。seq 只应用于关联请求和反馈，调用端必须生成唯一值，不能依赖它实现幂等。</font>

| 功能 | seq | address | funcode | data | 示例数据 | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| 单个机械臂关节角度控制 | 调用端生成 | 1 | 1 | id、angle、delay_ms | {"seq":4,"address":1,"funcode":1",data":{"id":关节编号,"angle":关节目标角度,"delay_ms":执行时间}} | id为相应的关节编号，angle为关节的目标角度，delay_ms执行时间暂时设置为0 |
| 全部机械臂关节角度控制 | 调用端生成 | 1 | 2 | mode、angle0~6 | {"seq":4,"address":1,"funcode":2,"data":{"mode":控制模式,angle0":关节0角度值,"angle1":关节1角度值,"angle2":关节2角度值,"angle3":关节3角度值,"angle4":关节4角度值,"angle5":关节5角度值,"angle6":关节6角度值}} | 分别为mode控制模式和angle0~6的关节角度值，mode为0是10Hz数据的小平滑，mode为1是轨迹使用的大平滑 |
| 单个机械臂关节电机使能/卸力控制 | 调用端生成 | 1 | 4 | mode | {"seq":4,"address":1,"funcode":4,"data":{"id":关节编号,"mode":使能模式}} | id为相应的关节编号，mode为0卸力，mode为1使能 |
| 全部机械臂关节电机使能/卸力控制 | 调用端生成 | 1 | 5 | mode | {"seq":4,"address":1,"funcode":5,"data":{"mode":使能模式}} | mode为0卸力，mode为1使能 |
| 机械臂电机供电开关 | 调用端生成 | 1 | 6 | power | {"seq":4,"address":1,"funcode":6,"data":{"power":使能模式}} | power为0断电，power为1上电 |
| 机械臂位姿归零 | 调用端生成 | 1 | 7 | 无 | {"seq":4,"address":1,"funcode":7} | 无 |
|  |  |  |  |  |  |  |
| 机械关节角度反馈 | 10 | 2 | 1 | angle0~6 | {"seq":10,"address":2,"funcode":1,"data":{"angle0":关节0角度值,"angle1":关节1角度值,"angle2":关节2角度值,"angle3":关节3角度值,"angle4":关节4角度值,"angle5":关节5角度值,"angle6":关节6角度值}} | 反馈机械臂关节实时角度，上传频率10Hz |
| 机械臂状态反馈 | 10 | 2 | 3 | enable_status、pow_status、error_status | {"seq":10,"address":2,"funcode":3,"data":{"enable_status":使能状态,"power_status":上电状态,"error_status":故障状态}} | 反馈机械臂实时状态，enable_status是使能状态，1使能0卸力；power_status是供电状态，1供电0断电；error_status是故障状态，1正常0异常 |
|  |  |  |  |  |  |  |
| 指令接收反馈 | 调用端生成 | 3 | 1 | recv_status | {"seq":10,"address":3,"funcode":1,"data":{"recv_status":接收状态}} | 接收成功反馈，recv_status为1说明接收成功，为0说明发送数据有错（格式错误无法解析、不在功能范围内等） |
| 指令执行反馈 | 调用端生成 | 3 | 2 | exec_status | {"seq":10,"address":3,"funcode":2,"data":{"exec_status":执行状态}} | 执行成功反馈，exec_status为1说明执行成功，为0说明执行失败 |


_**<u><font style="color:#DF2A3F;">接口表实测修正与补充：</font></u>**_

1. **<font style="color:#DF2A3F;">funcode=1：</font>**`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">delay_ms</font>`<font style="color:#DF2A3F;"> 是轨迹执行/平滑时间，不是延迟一段时间后再启动；实际到位时间可能略长。新的同关节目标会立即抢占旧轨迹，不会排队，快速反向可能产生过冲。</font>
2. **<font style="color:#DF2A3F;">funcode=2：</font>**<font style="color:#DF2A3F;">当前设备可稳定接收 10Hz 全关节指令；mode=0 与 mode=1 均存在跟踪滞后和幅度衰减，最后一帧成功 ACK 也不保证精确到位。停发后机械臂继续使能并保持最后实际位置。</font>
3. **<font style="color:#DF2A3F;">funcode=4/5：</font>**<font style="color:#DF2A3F;">mode 不是简单的 0/1 布尔量。实测 mode=1 仍为卸力；采样值 0～999 报告未使能，1000、10000、32768、65535、80000 报告使能。建议当前固件使用 0 卸力、65535 完整使能。保持力与 mode 的定量关系未标定。</font>
4. **<font style="color:#DF2A3F;">enable_status：</font>**<font style="color:#DF2A3F;">不是可靠的单关节状态，也不是全部关节状态的 AND 汇总。它更接近最近一次使能相关操作的服务级标志；即使只有 J0 锁定、其他关节全部卸力，也可能报告 1。客户端必须自行维护逐关节状态。</font>
5. **<font style="color:#DF2A3F;">funcode=6：</font>**`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">power=1</font>`<font style="color:#DF2A3F;"> 实测可以上电；</font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">power=0</font>`<font style="color:#DF2A3F;"> 在三种测试条件下均虚假返回成功但没有断电，应视为当前固件不可用。</font>
6. **<font style="color:#DF2A3F;">funcode=7：</font>**<font style="color:#DF2A3F;">会让 J0～J6 七个反馈量都移动到数值 0 附近，包括夹爪；不会自动断电或卸力。该动作可能产生大范围、多关节同时运动，必须清空完整运动空间。J6 数值 0 不等于夹爪完全闭合。</font>
7. **<font style="color:#DF2A3F;">状态反馈：</font>**<font style="color:#DF2A3F;">字段实际名称是 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">power_status</font>`<font style="color:#DF2A3F;">，表格中的 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">pow_status</font>`<font style="color:#DF2A3F;"> 是拼写错误。正常控制、回零和流式运行期间 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">error_status</font>`<font style="color:#DF2A3F;"> 始终为 0，因此“1正常、0异常”的说明与实测相反；非零错误码含义仍需厂商提供。</font>
8. **<font style="color:#DF2A3F;">ACK 语义：</font>**`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">recv_status=1</font>`<font style="color:#DF2A3F;">/</font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">exec_status=1</font>`<font style="color:#DF2A3F;"> 不能证明参数安全、物理动作发生或已经到位。非法 id=7、负 mode、无效 power=0 都曾返回成功 ACK。客户端必须限制 id 为 0～6，并校验地址、类型、范围和目标。</font>
9. **<font style="color:#DF2A3F;">执行 ACK：</font>**<font style="color:#DF2A3F;">funcode=1 实际运动时经常没有执行 ACK；funcode=2 和 funcode=7 的执行 ACK 可能早于运动。所有到位判断必须读取实时角度并设置容差与超时。</font>



该服务对外界提供两个接口，分别为`rt/arm_Command`和`rt/arm_Feedback`。通过rt/arm_Command可以向该服务下发数据及指令请求，通过rt/arm_Feedback可以获得该服务上传的数据。其中rt/arm_Feedback需要处于回调监听执行，rt/arm_Command根据需要主动调用即可。

该数据数据使用ArmString_.idl格式，格式如下。

```plain
// generated from rosidl_generator_dds_idl/resource/idl.idl.em
// with input from unitree_arm:msg/ArmString.idl
// generated code does not contain a copyright notice

#ifndef __unitree_arm__msg__arm_string__idl__
#define __unitree_arm__msg__arm_string__idl__


module unitree_arm {

module msg {

module dds_ {


struct ArmString_ {
string data_;

};


};  // module dds_

};  // module msg

};  // module unitree_arm


#endif  // __unitree_arm__msg__arm_string__idl__
```

## SDK 及示例程序的获取与使用
机械臂的SDK及示例程序运行环境依赖于unitree_sdk2，在进行D1的相关开发之前，需要先部署unitree_sdk2。

D1的SDK及示例程序可通过该地址下载：

[D1_SDK.zip](https://oss-global-cdn.unitree.com/static/b37d684fd97a40b59b61b7c4fdcd39c6.zip)

下载完成后在里面创建文件夹，使用cmake构建和make编译即可使用。

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/39019497/1783949078784-b95f85be-b1a2-4e58-8ce6-ab067d77113e.png)

**arm_zero_control.cpp:** 机械臂归零示例程序；

**get_arm_joint_angle.cpp:** 获取机械臂关节角度值示例程序；

**joint_angle_control.cpp:** 机械臂单关节角度控制示例程序；

**joint_enable_control.cpp:** 机械臂关节使能控制示例程序；

**multiple_joint_angle_control:** 机械臂多关节角度控制示例程序。

其中msg文件是我们需要使用到的D1机械臂SDK接口文件，后续在自己项目中只需要加入这几个文件即可。也可以将该文件复制到/usr/local/include下，使用时通过**include_directories加入路径**、**使用link_libraries链接文件**即可，**目前仅支持ubuntu系统，且依赖于unitree_sdk2**。

## 机械臂 URDF 下载
| 类别 | 差异 |
| --- | --- |
| [有夹爪版](https://oss-global-cdn.unitree.com/static/90b2525be5d84531ab9814f48b2f86a7.zip) | 末端夹爪通过滑块接口可实现末端夹爪的打开和闭合，适合需要在仿真下实现抓取任务的开发 |
| [无夹爪版](https://oss-global-cdn.unitree.com/static/02c95ece8e354143b874a3c963241467.zip) | 末端夹爪为固定结构，和机械臂最末端轴固定，夹爪不可打开和闭合，适合只做云末端位姿求解仿真任务 |
| [D1简化模型](https://oss-global-cdn.unitree.com/static/5bade30b2a454376978ab2853589e5f7.zip) | STEP 简化模型下载 |
| [D1-550 简化模型](https://oss-global-cdn.unitree.com/static/fc33175f0fbd4fb0842e588080c2b3d9.zip) | D1-500 STEP 简化模型下载 |
| [D1-550 URDF](https://oss-global-cdn.unitree.com/static/9b20252a26374d50aa369532657d0143.zip) | D1-550 URDF |
| [D1-550 夹爪开源文件](https://oss-global-cdn.unitree.com/static/6f0d89dcedde40f1a59272b71c5363a2.zip) | D1-550 夹爪开源文件 |


## 使用示例
### 1. 机械臂单关节角度控制
通过rt/arm_Command话题实现，假设此时生成的seq值为4，设定5号关节角度为60。该数据内容为`{"seq":4,"address":1,"code":1,"data":{"id":5,"angle":60,"delay_ms":0}}`_**<u><font style="color:#DF2A3F;">格式修正：</font></u>**_<font style="color:#DF2A3F;">此处示例中的 "code":1 应为 "funcode":1，与后面的 C++ 示例一致。</font>

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":1,\"data\":{\"id\":5,\"angle\":60,\"delay_ms\":0}}";
    publisher.Write(msg);

    return 0;
}
```

_**<u><font style="color:#DF2A3F;">单关节控制实测补充：</font></u>**_

1. <font style="color:#DF2A3F;">id 仅允许 0～6。非法 id=7 曾错误返回成功 ACK，并导致 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">current_servo_angle</font>`<font style="color:#DF2A3F;"> 停止；发送合法 J6 使能指令后才恢复。</font>
2. <font style="color:#DF2A3F;">全关节卸力后发送单关节当前位置目标，会自动使能并锁住该目标关节；其他关节仍卸力，但全局 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">enable_status</font>`<font style="color:#DF2A3F;"> 会变成 1。卸力前必须先停止所有控制发布者。</font>
3. <font style="color:#DF2A3F;">发布进程退出后，机械臂至少 30 秒继续保持位置、供电和使能，不会因为客户端退出自动停止。</font>
4. <font style="color:#DF2A3F;">新目标会抢占该关节正在执行的旧目标。实测慢速目标执行 800ms 后发送反向目标，关节立即反向，没有先完成旧目标。</font>

### 2. 机械臂多关节角度控制
通过rt/arm_Command话题实现，假设此时生成的seq值为4，机械臂关节角度为{0，-60，60，0，30，0，0}。该数据内容为`{"seq":4,"address":1,"funcode":2,"data":{"mode":1,"angle0":0,"angle1":-60,"angle2":60,"angle3":0,"angle4":30,"angle5":0,"angle6":0}}`

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":0,\"angle1\":-60,\"angle2\":60,\"angle3\":0,\"angle4\":30,\"angle5\":0,\"angle6\":0}}";
    publisher.Write(msg);

    return 0;
}
```

_**<u><font style="color:#DF2A3F;">多关节控制实测补充：</font></u>**_

1. <font style="color:#DF2A3F;">以 10.000Hz 连续发送 41 帧时，mode=0 和 mode=1 均收到 41/41 接收及执行 ACK，并能驱动 J3/J4 做相反方向的小幅同步运动。</font>
2. <font style="color:#DF2A3F;">两种模式均存在平滑造成的幅度衰减和相位滞后；停止发送并等待 5 秒后，最终位置仍可能偏离最后目标约 0.3～0.5°。</font>
3. <font style="color:#DF2A3F;">停止发布不会卸力，机械臂保持最后实际位置。建议持续读取反馈，并在轨迹末尾增加显式收敛阶段。</font>



### 3. 机械臂关节电机使能/卸力控制
通过rt/arm_Command话题实现，假设此时生成的seq值为4，控制所有关节卸力。该数据内容为`{"seq":4,"address":1,"funcode":5,"data":{"mode":0}}`

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    publisher.Write(msg);

    return 0;
}
```

mode的范围为0~80000，0完全卸力，80000完全锁死。

_**<u><font style="color:#DF2A3F;">使能/卸力实测补充：</font></u>**_

1. <font style="color:#DF2A3F;">mode=0 确认几乎无阻力；mode=40000 阻力明显增加但仍可手推；mode=80000 的手感与 40000 接近。不能据此假设 mode 与保持力线性，40000～80000 可能已饱和或呈非线性。当前建议恢复值使用 65535。</font>
2. <font style="color:#DF2A3F;">JSON 命令路径可接受 80000；SDK 中其他消息的 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">uint16_t</font>`<font style="color:#DF2A3F;"> 字段不能代表 80000，但不限制本文 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">ArmString_</font>`<font style="color:#DF2A3F;"> JSON 路径。</font>
3. **<font style="color:#DF2A3F;">重要安全行为：</font>**<font style="color:#DF2A3F;">关节卸力后手动改变姿态，不会更新控制器缓存目标。随后执行全关节使能时，多个关节会主动追赶旧目标，而不是在手动当前位置原地锁住。重新使能前必须支撑机械臂、清空运动空间，并先处理安全目标。</font>

### 4. 机械臂位姿归零
通过rt/arm_Command话题实现，假设此时生成的seq值为4，控制机械臂回归零位。该数据内容为`{"seq":4,"address":1,"funcode":7}`

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":7}";
    publisher.Write(msg);

    return 0;
}
```

_**<u><font style="color:#DF2A3F;">回零实测补充：</font></u>**_<font style="color:rgb(255, 0, 0);">从约 [-92.7,-88.6,88.9,-1.2,6.5,4.7,63.4] 回零后，七轴反馈约为 [0.1,0.4,0.7,-0.2,1.0,-0.3,0.1]。接收 ACK 约 1.6ms、执行 ACK 约 3.4ms，均早于机械运动完成。回零包含夹爪且运动范围很大；执行前必须确认完整运动空间安全。回零完成后仍保持</font>

### 5. 获取实时关节角度
该指令seq值固定为10，上传频率为10Hz，通过address地址+code功能码进行数据区分，需要使用回调监听处理。

```cpp
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/PubServoInfo_.hpp"
#include "msg/ArmString_.hpp"

#define TOPIC "current_servo_angle"
#define TOPIC1 "rt/arm_Feedback"

using namespace unitree::robot;
using namespace unitree::common;

void Handler(const void* msg)
{
    const unitree_arm::msg::dds_::PubServoInfo_* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;

    std::cout << "servo0_data:" << pm->servo0_data_() << ", servo1_data:" << pm->servo1_data_() << ", servo2_data:" << pm->servo2_data_()<< ", servo3_data:" << pm->servo3_data_()<< ", servo4_data:" << pm->servo4_data_()<< ", servo5_data:" << pm->servo5_data_()<< ", servo6_data:" << pm->servo6_data_() << std::endl;
}

void Handler1(const void* msg)
{
    const unitree_arm::msg::dds_::ArmString_* pm = (const unitree_arm::msg::dds_::ArmString_*)msg;

    std::cout << "armFeedback_data:" << pm->data_() << std::endl;
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subscriber(TOPIC);
    subscriber.InitChannel(Handler);

    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber1(TOPIC1);
    subscriber1.InitChannel(Handler1);

    while (true)
        {
            sleep(10);
        }

    return 0;
}
```

_**<u><font style="color:#DF2A3F;">反馈使用建议：</font></u>**_

1. <font style="color:#DF2A3F;">当前设备角度与状态反馈约为 8.97Hz，而非严格 10Hz。</font>
2. <font style="color:#DF2A3F;">使用 </font>`<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">angle0～angle6</font>`<font style="color:#DF2A3F;"> 做到位判断时必须设置容差和超时，不要等待浮点值与目标完全相等。夹爪端点尤其会稳定在目标内侧。</font>
3. `<font style="color:#DF2A3F;background-color:rgb(240, 240, 240);">error_status=0</font>`<font style="color:#DF2A3F;"> 可与正常核心功能同时存在，也可与非法请求同时存在，因此它不是最近一条命令的成功标志。</font>
4. <font style="color:#DF2A3F;">服务对相同 seq 不去重，重发可能导致同一动作执行两次。</font>

## D1-T 配置及使用
[遥操作基础示例代码](https://oss-global-cdn.unitree.com/static/82b7daf1ca8d43a3aa0f6b8421272abe.zip)

D1-T是使用路由器/交换机实现两台D1机械臂的数据通讯，将一台机械臂的关节数据输入到另一台机械臂执行。D1机械臂出场默认IP为192.168.123.100，在使用D1-T时我们需要将另外一台机械臂IP地址修改为192.168.123.xxx，这是我们使用192.168.123.99，也可根据自身需求自行修改。

D1-T包含两套D1机械臂，其中一套末端具备手持夹爪，该机械臂作为数据采集端。将任意一台机械臂机械臂上电，并通过网线接入到路由器/交换机中。

#### 路由器配置
我们需要进入到路由器后台管理界面，在路由器IP分配位置，修改WAN网段为192.168.123.xx。

修改成功并保存后，使用自己的电脑连接至路由器，通过终端执行ping 192.168.123.100来检测通讯是否正常。

#### 修改机械臂IP
确保电脑和机械臂通讯无误后，打开终端通过ssh登录至机械臂系统，密码123，指令如下。

```bash
ssh ubuntu@192.168.123.100
```

登录成功后我们进入ip配置的路径/etc/network，通过vim指令修改interface文件中的IP为192.168.123.99即可，修改完成后保存并对系统进行重启。

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/39019497/1783949100940-7e45ccc3-29af-42ad-bd8a-534b90e56821.png)

将另一台机械臂上电并接入路由器，此时两台机械臂的IP分别为192.168.123.99和192.168.123.100。通过ping指令分别对两个IP进行检测，确保通讯正常。

#### 多机械臂控制
在使用两台或多台机械臂时，会遇到两种情况：

1. 多台机械臂连接同一台电脑
2. 电脑和机械臂连接同一台路由器

想要分别控制这两台或多台机械臂就有两种方法：

##### 情况一：多台机械臂连接同一台电脑
电脑上会出现多个网卡，只需要将各自的机械臂与电脑上的网卡绑定即可。

首先我们需要知道需要控制的机械臂绑定在哪一个网卡上面，可以通过指令获取网卡信息

<!-- 这是一张图片，ocr 内容为： -->
![](https://cdn.nlark.com/yuque/0/2026/png/39019497/1783949113183-93974e38-8e91-4a32-b7ca-3b50c633f92d.png)

可以看到实例机械臂绑定在了“eth0”网卡上，打开官方示例程序“机械臂位姿归零”，在主函数第一行可以看到机械臂初始化的内容

```cpp
int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":7}";
    publisher.Write(msg);

    return 0;
}
```

在初始化的信息里面加入要绑定的网卡名称即可控制指定网口的机械臂

```plain
ChannelFactory::Instance()->Init(0, "eth0");
```

##### 情况二：电脑和机械臂连接同一台路由器
电脑只有一个网卡与所有机械臂相连，此时需要修改机械臂内部的驱动文件。

首先需要知道机械臂的所有控制指令都是通过dds进行传输的，在程序中订阅需要的话题内容即可获取所需的消息，因为所有的机械臂的控制指令都是使用的同一个话题，想要在同一个局域网内控制不同的机械臂，只需要将每台机械臂的控制指令的话题改成唯一即可，先只连接一条机械臂

在修改机械臂驱动代码时，先要关闭默认正在运行的服务

```bash
sudo systemctl stop marm_controller.service
sudo systemctl stop marm_control.service
sudo systemctl stop marm_communication.service
sudo systemctl stop marm_subscripber.service

sudo systemctl disable marm_controller.service
sudo systemctl disable marm_control.service
sudo systemctl disable marm_communication.service
sudo systemctl disable marm_subscripber.service
```

通过ssh连接机械臂后，输入以下内容：

```bash
vim marm_code/src/marm_communication_node.cpp
```

可以看见内部驱动代码：

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "msg/SetServoAngle_.hpp"
#include "msg/SetServoDumping_.hpp"

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <iostream>
#include <thread>
#include <chrono>

#define SubArmCommand_Topic "rt/arm_Command"
#define SubServoCurrentAngle_Topic "current_servo_angle"
#define PubArmFeedback_Topic "rt/arm_Feedback"
#define PubArmZero_Topic "arm_zero"
#define PubServoAngle_Topic "set_servo_angle"
#define PubServoAngleControl_Topic "set_servo_angle_control"
#define PubServoDumping_Topic "set_servo_dumping"
```

找到与示例程序“机械臂位姿归零”里控制指令一样的话题：

```plain
#define SubArmCommand_Topic "rt/arm_Command"
```

修改成指定的唯一话题，例如

```plain
#define SubArmCommand_Topic "rt/arm_Command_1"
```

完成后保存退出，等待重新编译完成后重新打开服务，重新启动机械臂

```bash
cd
cd marm_code/build/
make clean
make

sudo systemctl enable marm_communication.service 
sudo systemctl enable marm_control.service 
sudo systemctl enable marm_controller.service 
sudo systemctl enable marm_subscripber.service 

sudo reboot

sudo systemctl start marm_communication.service 
sudo systemctl start marm_control.service 
sudo systemctl start marm_controller.service 
sudo systemctl start marm_subscripber.service
```

这个实例操作成功后将臂里所有的`marm_code/src`驱动文件的topic的后缀都加上特殊标志，使每个臂在局域网内的话题唯一，例如：

`/marm_code/src/marm_communication_node.cpp`

```cpp
#define SubArmCommand_Topic "rt/arm_Command_1"
#define SubServoCurrentAngle_Topic "current_servo_angle_1"
#define PubArmFeedback_Topic "rt/arm_Feedback_1"
#define PubArmZero_Topic "arm_zero_1"
#define PubServoAngle_Topic "set_servo_angle_1"
#define PubServoAngleControl_Topic "set_servo_angle_control_1"
#define PubServoDumping_Topic "set_servo_dumping_1"
```

`/marm_code/src/marm_control_node.cpp`

```cpp
#define PubServoAngle_Topic "set_servo_angle_1"
#define PubArmFeedback_Topic "rt/arm_Feedback_1"
#define SubArmZero_Topic "arm_zero_1"
#define SubServoAngleControl_Topic "set_servo_angle_control_1"
#define SubServoCurrentAngle_Topic "current_servo_angle_1"
```

`/marm_code/src/marm_controller_node.cpp`

```cpp
#define PubArmFeedback_Topic "rt/arm_Feedback_1"
#define PubServoAngle_Topic "current_servo_angle_1"
#define SubServoAngle_Topic "set_servo_angle_1"
#define SubServoDumping_Topic "set_servo_dumping_1"
```

> **编译可能遇到的问题及解决方法**
>

> 1. 编译失败：删除整个 `build` 文件夹后重新编译。
> 2. 文件时间错误，例如：`make: Warning: File 'Makefile' has modification time xxx s in the future`、`Warning: File 'xxx' has modification time yyy s in the future` 或 `warning: Clock skew detected. Your build may be incomplete.`。此时可将系统时钟修改为当前时间，例如 `sudo date -s yyyy-mm-dd`。
>

此时将示例程序“机械臂位姿归零”里的控制指令话题修改成一样的内容，即是用相同的话题便可控制指定的机械臂，想要获取指定机械臂的角度数据也是一样的原理，如下所示

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command_1"

using namespace unitree::robot;
using namespace unitree::common;

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":7}";
    publisher.Write(msg);

    return 0;
}
```

这样便完成了控制指定机械臂

#### 启动机械臂程序
不带手持设备的机械臂我们称为执行机械臂，带手持设备的机械臂我们称为采集机械臂。

执行机械臂使用默认程序即可，采集机械臂我们需要停止默认程序，使用我们的采集接口程序，使用以下指令停止当前运行的服务。

```bash
sudo systemctl stop marm_controller.service
sudo systemctl stop marm_control.service
sudo systemctl stop marm_communication.service
sudo systemctl stop marm_subscripber.service
```

该指令会在当前系统中停止出厂程序，若需要永久停止，则继续执行以下指令。

```bash
sudo systemctl disable marm_controller.service
sudo systemctl disable marm_control.service
sudo systemctl disable marm_communication.service
sudo systemctl disable marm_subscripber.service
```

可通过`sudo systemctl status 服务名`，来查看当前服务的状态。

## 实例演示
以下是本公司实现的机械臂遥操作演示，暂不提供源码，如需请自行实现。

## 效果
