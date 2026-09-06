# Unitree D1 机械臂自主视觉感知抓取投放 - 部署教程
中文 | [English](README.md)

## 软硬件要求
| 设备 | 用途 | 是否必需 |
| --- | --- | --- |
| [Unitree D1-550](https://www.unitree.com/D1-T) | 执行抓放动作 | 必需 |
| [Intel RealSense D435i](https://realsenseai.com/products/depth-camera-d435i/) | 腕部 RGB-D 感知和重力测量 | 必需 |
| [相机支架](../hardware/wrist_camera_mount/README_CN.md) | 固定相机与机械臂的几何关系 | 必需 |
| Ubuntu 22.04 LTS/ 20.04 LTS | 开发、仿真和独立真机调试 | 必需 |
| [Unitree Go2 EDU](https://www.unitree.com/go2) | 将机械臂固定于机器狗身上，实现可移动导航，对物体进行抓取投放的自主作业平台 | 非必需。此项目可脱离 Go2 单独运行，也支持 Go2 集成 |




## 部署流程简介
+ 若需对机械臂服务进行开发、调试和独立验收：选择「部署流程1」
+ 若需将本项目部署到 Unitree Go2 上进行集成，需先跑通「部署流程1」，确认机械臂和 RealSense 相机均能按预期工作后再进行「部署流程2」

## 部署流程1：将项目部署于个人开发机
+ 此部署方式适用于机械臂服务的开发、调试和独立验收。
+ 若目标是修改代码、验证抓放流程或排查设备问题，建议选择此方式
+ 若目标是将本项目部署到 Unitree Go2 上进行集成，建议先跑通本流程后再继续「部署流程2」
+ 此部署流程下，机械臂、腕部 RealSense 相机直接连接开发机，由开发机运行完整的感知、规划与控制服务。该方式具有较好的开发工具兼容性和调试性能，便于使用 RViz、日志及可视化工具定位问题。
+ 此流程要求开发机为`x86_64`架构（执行`uname -m`输出`x86_64`即符合要求） + Ubuntu 22.04 LTS/ 20.04 LTS。
+ 项目使用 Docker 进行部署，建议预留至少 20 GB 磁盘空间。

### 1.1 安装基础工具
```bash
sudo apt update
sudo apt install -y git git-lfs python3 python3-yaml openssh-client sshpass tar usbutils
git lfs install
```

### 1.2 获取项目代码及相关大文件
```bash
mkdir -p ~/Project && cd ~/Project
git clone https://github.com/yuhanliu0121/unitree_arm.git
cd unitree_arm
git lfs pull
```

### 1.3 一键构建 Docker 环境
本项目使用 Docker 封装 ROS 2、MoveIt、RealSense 驱动及视觉感知等运行依赖，以减少不同开发机之间的环境差异，同时提供脚本进行 Docker 环境的一键构建。

1. 执行：

```bash
xhost +
cd ~/Project/unitree_arm
./deploy/install_d1_runtime_env.sh
```

执行成功时，末尾应看到类似以下结果。测试数量和耗时可能随代码版本及设备性能变化，应以最终汇总无失败且出现 `D1 runtime environment is ready` 为准：

```bash
============================== 27 passed in 0.08s ==============================
Finished <<< d1_bringup [0.57s]

Summary: 8 packages finished [4.94s]
  1 package had stderr output: d1_camera_visualization
Summary: 87 tests, 0 errors, 0 failures, 0 skipped

D1 runtime environment is ready.
  architecture: amd64
  image:        unitree-d1-manipulation:amd64
  workspace:    /home/<your_user_name>/Project/unitree_arm
  dev container: unitree-d1-manipulation-dev-amd64 (running)

Enter the development environment with:
  docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
```

2. 继续检查：

```bash
test -f install/setup.bash && echo "ROS workspace: OK"
sudo docker image inspect unitree-d1-manipulation:amd64 >/dev/null && echo "Docker image: OK"
sudo docker ps --format '{{.Names}}' | grep '^unitree-d1-manipulation-dev-amd64$'
```

正确结果：

```text
ROS workspace: OK
Docker image: OK
unitree-d1-manipulation-dev-amd64
```

构建或测试失败时应检查终端中的第一条错误和仓库下的 `log/`，不能进入仿真或真机测试。

3. 构建完成后进入开发容器：

```bash
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
```

流程1中后续命令默认在该开发容器内执行；明确标注为宿主机操作的步骤除外。`/workspace` 是宿主机 `unitree_arm/` 目录的挂载路径，因此源码以及 `build/`、`install/`、`log/` 的修改会直接保留在宿主机中。

### 1.4 MuJoCo 仿真验证
先在固定 MuJoCo 场景中完整验证感知、运动规划、抓取、携带和投放流程。本步骤只连接仿真控制器，不会向真实 D1 发送运动命令，也不要求连接 D1 或 RealSense。

1. 在容器内执行带 MuJoCo 和 RViz 界面的交互式仿真验收：

```bash
cd /workspace
./tools/validation/scripts/accept_cube_pick_drop.zsh --rviz
```

脚本会自动启动 MuJoCo、RViz、机械臂控制栈和感知节点。

<table>
  <thead>
    <tr>
      <th>MuJoCo 窗口</th>
      <th>RViz 窗口</th>
      <th>终端输出</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><img src="images/mujoco_window.png" alt="MuJoCo 仿真窗口" width="360"></td>
      <td><img src="images/rviz_simulation.png" alt="RViz 仿真窗口" width="360"></td>
      <td><img src="images/simulation_terminal_ready.png" alt="仿真验收终端输出" width="360"></td>
    </tr>
  </tbody>
</table>


启动就绪后终端会输出`Press Enter to pick yellow_cube and move to CARRY...`，此时在终端按下`enter`开始进行仿真。

成功抓取后，终端应输出`ACCEPTANCE PASSED: yellow_cube picked, carried, released into bin, and arm stowed.`

![MuJoCo 仿真验收通过](images/simulation_acceptance_passed.png)

未出现该结果时，应根据终端错误和 `log/` 中的仿真日志排查问题，不要继续真机部署



注意：上述同时拉起 MuJoCo、RViz 和控制栈会占用较多开发机资源，性能不足时容易导致桌面卡顿以及仿真失败。此时建议使用无界面模式进行仿真验收：

```bash
./tools/validation/scripts/accept_cube_pick_drop.zsh --headless
```

### 1.5 硬件设备连接
1. 使用 DC 电源给 D1 上电，并用网线连接机械臂与开发机。
2. 使用 USB 3.0 线缆连接 RealSense D435i 和开发机。
3. 在容器终端内执行 `lsusb | grep -i realsense`，应能看到类似如下结果：

![容器内识别到 RealSense](images/realsense_lsusb.png)

### 1.6 配置开发机网络
1. 点击 Ubuntu 屏幕右上角的网络/WIFI 图标，选择有线连接（Wired Settings）。
2. 点击有线网络旁边的“齿轮”图标设置，选择 **IPv4** 标签页
3. 将 Method（方法）改为 **Manual（手动）**。
4. 在 Address（地址）处填入：`192.168.123.162`
5. Netmask（子网掩码）填入：`255.255.255.0`

![Ubuntu 有线网络 IPv4 配置](images/ubuntu_wired_ipv4.png)

6. 点击右上角 Apply（应用），然后把有线网络关掉再重新打开，让配置生效。

### 1.7 验证开发机与 D1 的网络连接
继续在容器内终端中输入 `ping -c 3 192.168.123.100` 进行检测，收到类似下图的回复即连接成功。

![D1 网络连通性检查](images/d1_ping.png)

### 1.8 一键更新机械臂板端控制服务
D1-550 内部控制板上运行着机械臂板端控制服务，负责接收开发机或 Go2 发来的关节指令、驱动电机并返回执行状态。官方服务提供的多关节控制及反馈信息较为有限，难以可靠判断指令是否执行、关节是否到位以及失败发生在哪个环节，不利于 ROS 2/MoveIt 抓放任务的执行和故障诊断。

因此，本项目在官方协议基础上增强了板端控制服务，补充了更规范的多关节指令处理、执行结果反馈、状态诊断和版本查询能力，并与项目中的主机端控制节点配套使用。

首次部署时需要将 D1 内的官方板端服务更新为项目配套版本，之后日常运行无需重复安装。

更新板端控制服务只需在项目根目录执行：

```bash
cd /workspace
./deploy/update_d1_onboard_sdk.sh --confirm UPDATE_D1_ONBOARD_SDK
```

脚本会将板端控制服务源码上传至 D1，在板端完成编译和协议测试，备份原有服务，然后安装并启动新服务。该过程不会主动发送关节运动命令，但切换控制服务时机械臂可能暂时失去保持力，应提前支撑机械臂并清空周围空间。完成后会看到：

```bash
100% tests passed, 0 tests failed out of 1
#其他内容......
D1 onboard SDK update passed. The d1-control.service is enabled and active.
```

![D1 板端服务更新成功](images/onboard_update.png)

此时，可以通过ssh查看服务信息

```bash
# 如果需要密码，可尝试D1出厂默认密码123
ssh ubuntu@192.168.123.100 'systemctl is-active d1-control.service && cat /home/ubuntu/marm_code/build/d1-control-release.env'
```

结果应为：

```bash
active
D1_CONTROL_VERSION=0.1.0
D1_CONTROL_PROTOCOL=2
```

![D1 板端服务状态和协议版本](images/onboard_service_status.png)

说明更新成功

### 1.9 一键配置环境参数
机械臂服务运行依赖 D1 通信网卡、RealSense 序列号、感知模型及设备外参等多项参数。为简化部署，本项目提供了一键配置脚本，可自动探测当前平台、D1 通信网卡和腕部 RealSense，并填入已验证的默认配置。

1. 执行脚本前，需先将 D1 和 RealSense D435i 连接至开发机，确认执行 `lsusb | grep -i realsense` 能看到 RealSense，且执行 `ping -c 3 192.168.123.100` 可以收到回复。
2. 运行配置脚本：

```bash
cd /workspace
./deploy/configure_d1_manipulation.sh
```

脚本会自动识别当前设备的网络和硬件连接，并生成配置文件到 `src/d1_bringup/config/real_machine.local.yaml`。各参数说明如下：

| 参数 | 作用 | 是否需要人工配置 | 何时需要修改 |
| --- | --- | --- | --- |
| `deployment.platform` | 标识当前部署平台，并用于阻止在错误架构上加载配置 | 否，脚本自动识别 | 一般不修改；仅自动识别失败且已确认平台时重新生成配置 |
| `deployment.ros_domain_id` | ROS 2 节点发现与通信所使用的 Domain ID | 否，脚本自动填写默认值`31` | 需要与其他 ROS 2 节点通信时改成相同值，范围为 `0–232` |
| `perception.model_path` | YOLO 目标识别模型路径 | 通常需要；脚本默认使用 `runtime/perception_runtime_v1/weights/mcislab_trash_collect.pt` | 默认模型仅适用于项目当前验证过的物体和环境；部署到其他场景时，应替换为针对实际物体和现场数据训练并验证过的模型 |
| `arm.serial_no` | 选择个体机械臂的 URDF 修正配置，目前主要用于加载关节限位覆盖 | 通常否，脚本默认留空 | 只有机械臂存在需要单独修正的关节限位或其他 URDF 参数，并已在 `hardware_profiles.yaml` 中建立对应配置时才填写 |
| `arm.network_interface` | 主机向 D1 收发原生控制和反馈数据所用的有线网卡 | 通常否；未识别或存在多个候选时填写 | 未探测到或存在多个候选时，根据 `ip -br address` 手工填写 |
| `wrist_camera.driver_location` | 指定由本机还是另一台 ROS 2 主机启动 RealSense 驱动 | 通常否；远程相机模式时修改 | 相机接在另一台计算机并由其发布数据时改为 `remote` |
| `wrist_camera.serial_no` | 指定作为腕部相机的 RealSense | 通常否；未识别或存在多个候选时填写 | 未探测到或连接多台 RealSense 时手工填写 |
| `wrist_camera.frame_rate_hz` | RGB、深度和对齐深度流的帧率 | 通常否，默认 15 Hz | 根据算力和网络带宽改为 `6`、`15` 或 `30` Hz |
| `wrist_camera.link6_to_camera_link.translation_xyz_m` | `Link6` 原点到相机坐标系原点的平移，单位为米 | 通常否，脚本默认使用项目自带的相机支架的标定结果 | 相机、支架或相对安装位置改变后重新标定并修改 |
| `wrist_camera.link6_to_camera_link.quaternion_xyzw` | `Link6` 到相机坐标系的旋转四元数，顺序为 `x,y,z,w` | 通常否，脚本默认使用项目自带的相机支架的标定结果 | 相机、支架或相对安装姿态改变后重新标定并修改 |
| `site.go2_base_to_arm_base.translation_xyz_m` | Go2 基座到机械臂基座的安装平移，单位为米 | 通常否，独立开发机测试默认为 `[0,0,0]` | 流程1中使用默认值即可，若安装到 Go2 后必须填写实测值 |
| `site.go2_base_to_arm_base.quaternion_xyzw` | Go2 基座到机械臂基座的安装旋转，顺序为 `x,y,z,w` | 通常否，独立开发机测试默认为单位四元数 | 流程1中使用默认值即可，若安装到 Go2 后必须填写实测值 |


3. 对生成的配置文件进行校验，执行：

```bash
./deploy/configure_d1_manipulation.sh --validate
```

终端显示`Configuration: OK`后即可继续后续步骤；若校验失败，根据终端提示修改对应参数。

### 1.10 一键启动机械臂控制栈
1. 用 USB 3.0 线缆连接 RealSense D435i 与开发机，用网线连接机械臂与开发机，确认在容器内执行 `ping -c 3 192.168.123.100` 能访问 D1，并且执行 `lsusb | grep -i realsense` 能找到 RealSense 相机。
2. 将机械臂摆放成如下收纳姿态（STOWED）：机械臂大臂朝向与网口方向一致

![D1 STOWED 收纳姿态](images/stowed_pose.png)

3. 终端执行命令：

```bash
cd /workspace
./scripts/real_bringup.zsh --rviz
```

启动过程会检查板端版本、关节反馈、RealSense、IMU、TF、MoveIt 和业务接口，并且不会自动执行抓放动作。只有看到以下横幅才表示服务就绪：

+ 注意：控制栈启动时会检查当前姿态，如果与上图姿态偏差太多会启动失败。

```bash
************************************************************
* D1 REAL CONTROL STACK READY                              *
* pick_object and drop_object are ready to accept commands *
************************************************************
```

![真机控制栈启动就绪](images/real_stack_ready.png)

没有出现横幅时不要调用任务接口。流程1直接在持久开发容器中启动控制栈，运行日志位于该容器内的 `/tmp/d1_ros_logs/`。

同时在弹出的 RViz 中确认：

1. 机械臂模型完整显示，关节姿态与真机基本一致。
2. 腕部相机 RGB 画面能够持续更新。
3. aligned depth 伪彩色画面能够持续更新，且与 RGB 视野对齐。
4. RViz 窗口、机械臂模型或任一相机画面异常时，不要进入实际抓放验收。（启动后Wrist Camera YOLO显示No Image是正常的。该窗口在进行视觉估计环节调用yolo算法后才会显示画面）

![RViz 真机模型与腕部相机画面](images/rviz_real_system.png)

### 1.11 机械臂抓取、投放功能验证
功能验证分为基础部署验收和实际抓放验收。验收期间保持控制栈终端运行，并在宿主机新开一个 Bash 终端，进入同一个开发容器：

```bash
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
export ROS_DOMAIN_ID=$(python3 -c 'import yaml; print(yaml.safe_load(open("/workspace/src/d1_bringup/config/real_machine.local.yaml"))["deployment"]["ros_domain_id"])')
```

#### 1.11.1 基础部署验收
首先检查机械臂公共接口：

```bash
ros2 action list | grep '^/arm/tasks/'
ros2 topic list | grep '^/arm/task_status$'
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

结果应包含：

```text
/arm/tasks/drop_object
/arm/tasks/pick_object
/arm/task_status
```

机械臂以 STOWED 姿态正常启动时，`/arm/task_status` 的核心字段应为：

```yaml
state: 1
payload_state: 0
canonical_pose: 0
```

![READY_STOWED 状态消息](images/task_status_ready_stowed.png)

#### 1.11.2 机械臂抓放功能验证
1. 在机械臂底座Y方向0.35m放一个5cm x 5cm x 5cm的黄色立方体（可以用卡纸做一个），D1-550的底座坐标系和黄色立方体如下图所示

![立方体真机抓放验收布置](images/cube_test_layout.png)

2. 确保机械臂运动范围内无人员和障碍物，在执行下面的抓取命令

```bash
ros2 action send_goal /arm/tasks/pick_object d1_interfaces/action/PickObject "{target: {header: {frame_id: base_link}, point: {x: 0.0, y: 0.35, z: 0.0}}, stop_after: 4}" --feedback
```

抓取验收通过时，物体应被实际抓起，机械臂进入 CARRY 姿态，Action 返回 `success: true`、`outcome: 0`、`final_task_state: 2` 和 `payload_state: 1`。

3. 成功抓取物体并返回 `READY_CARRY`，或者抓取失败但安全返回 `READY_STOWED`，均可继续后续的投放功能测试。若进入 `FAULTED`，则说明发生了无法自动恢复的故障，需先完成排查再进行投放功能验证。
4. 进行投放功能验证：这里假设投放点依然为机械臂基座坐标下的`{x: 0.0, y: 0.35, z: 0.0}}`，执行：

```bash
ros2 action send_goal /arm/tasks/drop_object d1_interfaces/action/DropObject "{target: {header: {frame_id: base_link}, point: {x: 0.00, y: 0.35, z: 0.0}}}" --feedback
```

> 注意：执行过程中按下 `Ctrl+C` 会请求机械臂停止运动并保持当前姿态。确认机械臂处于安全恢复范围、运动空间已经清空后，可在 `/workspace` 下执行 `./scripts/arm_stowed_control.zsh --confirm STOWED_MOVE` 请求恢复至 STOWED；该工具不能保证从任意姿态安全恢复。
>

投放验收通过时，物体应被实际释放，机械臂返回 STOWED 姿态，Action 返回 `success: true`、`outcome: 0`、`returned_to_stowed: true`、`final_task_state: 1` 和 `payload_state: 0`。最后再次执行：

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

确认机械臂回到 `READY_STOWED`

5. 在机械臂服务终端按一次 `Ctrl+C`，等待控制栈完成退出清理。该操作只停止机械臂服务，不会删除开发容器。

功能验收结束后，可退出所有容器终端并在宿主机停止开发容器：

```bash
exit
sudo docker stop unitree-d1-manipulation-dev-amd64
```

下次继续执行抓取或投放时，在宿主机打开终端并执行：

```bash
# 启动并进入容器内部
sudo docker start unitree-d1-manipulation-dev-amd64
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash

# cd到工作区
cd /workspace

# 启动控制栈并等待直到横幅出现
./scripts/real_bringup.zsh --rviz
```

然后在宿主机终端打开另一个并执行：

```bash
# 进入容器内部
sudo docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash

# cd到工作区
cd /workspace

# 配置ROS Domain ID
export ROS_DOMAIN_ID=$(python3 -c 'import yaml; print(yaml.safe_load(open("/workspace/src/d1_bringup/config/real_machine.local.yaml"))["deployment"]["ros_domain_id"])')

# 执行抓取
ros2 action send_goal /arm/tasks/pick_object d1_interfaces/action/PickObject "{target: {header: {frame_id: base_link}, point: {x: 0.0, y: 0.35, z: 0.0}}, stop_after: 4}" --feedback

# 执行投放
ros2 action send_goal /arm/tasks/drop_object d1_interfaces/action/DropObject "{target: {header: {frame_id: base_link}, point: {x: 0.00, y: 0.35, z: 0.0}}}" --feedback
```



## 部署流程2：将项目部署于 Unitree Go2
+ 此部署流程适用于机械臂服务与 Go2 导航、运动控制及任务管理模块的整机集成，以及最终现场任务部署。
+ 执行本流程前，需要完成「部署流程1」，并在 x86_64 开发机上至少跑通一次完整的抓取与投放流程以确认机械臂、腕部相机、感知模型和公共接口工作正常。
+ 此流程要求使用带扩展计算单元的 Unitree Go2 EDU。在Go2上执行 `uname -m` 应输出 `aarch64`。机械臂感知、规划与控制服务最终由Go2 板载计算机运行，不依赖外部开发机
+ 部署流程1产生的 AMD64 Docker 镜像以及 `build/`、`install/`、`log/` 构建产物不能用于 Go2。本流程将在 Go2 上重新构建 ARM64 镜像和工作区。
+ 项目使用 Docker 进行部署，建议在 Go2 板载计算机上预留至少 20 GB 磁盘空间。

### 2.1 连接各设备
1. 将 D1-550 固定安装在 Go2 上，并用网线连接 D1 与 Go2 扩展坞。
2. 将腕部 RealSense D435i 通过 USB 3.0 接口连接至 Go2 扩展坞。
3. 给 D1 和 Go2 上电，清空机械臂运动范围并支撑机械臂。
4. 用网线连接 Go2 与开发机

### 2.2 登录 Go2 扩展计算单元
1. 按照「部署流程1」中「配置开发机网络」的做法配置开发机的网络，确认开发机上执行`ping 192.168.123.18`能收到回复
2. 在开发机终端执行：

```bash
# 在开发机终端执行
ssh unitree@192.168.123.18 #出厂默认密码为123
```

### 2.3 安装基础工具
登录后在 Go2 上的终端执行：

```bash
# 在Go2的终端执行
sudo apt update
sudo apt install -y git git-lfs python3 python3-yaml openssh-client sshpass tar usbutils
git lfs install
```

Docker、ROS 2、MoveIt、Conda 和 Python 感知依赖由后续部署脚本处理，不需要在 Go2 宿主系统中手动安装。

### 2.4 检查 Go2 与各设备连接情况
1. 验证 Go2 能访问 D1：

```bash
# 在Go2终端执行
ping -c 3 192.168.123.100
```

  正常情况下应收到三次回复。

 2. 验证 Go2 能识别 RealSense：

```bash
# 在Go2终端执行
lsusb | grep -i realsense
```

 正常情况下应看到 Intel RealSense 设备。若没有输出，应检查 USB 3.0 接口、线缆、扩展坞供电及相机连接。

3. D1 或 RealSense 未通过检查时，不要继续后续步骤。

### 2.5 获取项目代码
在 Go2 扩展计算单元上直接部署机械臂项目仓库。在 Go2 上执行：

```bash
mkdir -p ~/Project && cd ~/Project
git clone https://github.com/yuhanliu0121/unitree_arm.git
cd ~/Project/unitree_arm
git lfs pull
```

### 2.6 一键构建 ARM64 Docker 环境
本项目使用 Docker 封装 ROS 2、MoveIt、RealSense 驱动和视觉感知等运行依赖。Go2 使用 ARM64 镜像和 CPU 感知环境，Python 依赖通过 Miniforge/conda-forge 安装。先构建镜像，后续参数配置脚本才能在 Go2 宿主机未安装 librealsense 工具时复用镜像探测腕部相机。

在 Go2 上执行：

```bash
# 在 Go2 终端上执行：
cd ~/Project/unitree_arm
./deploy/install_d1_runtime_env.sh
```

脚本会自动安装缺失的 Docker，构建 Ubuntu 22.04 + ROS 2 Humble 的 ARM64 镜像，将项目根目录挂载到容器的 `/workspace`，并执行 `colcon build` 和测试。业务源码及 `build/`、`install/`、`log/` 均保留在 Go2 宿主机的 `unitree_arm/` 中。

成功标志：

```text
D1 runtime environment is ready.
  architecture: arm64
  image:        unitree-d1-manipulation:arm64
```

继续检查：

```bash
# 在 Go2 终端执行
test -f install/setup.bash && echo "ROS workspace: OK"
sudo docker image inspect unitree-d1-manipulation:arm64 >/dev/null && echo "Docker image: OK"
```

正确结果：

```text
ROS workspace: OK
Docker image: OK
```

构建或测试失败时应检查终端中的第一条错误和仓库下的 `log/`，不能进入整机测试。

### 2.7 检查 D1 板端控制服务
如果此时使用的 D1 与流程1中是同一台，且板端控制服务已经更新，可跳过此步骤。否则可在 Go2 上执行以下命令确认服务和协议版本：

```bash
# 在Go2的终端上执行
# 如果需要密码，可尝试 D1 的出厂默认密码 123
ssh ubuntu@192.168.123.100 'systemctl is-active d1-control.service && cat /home/ubuntu/marm_code/build/d1-control-release.env'
```

正确结果：

```text
active
D1_CONTROL_VERSION=0.1.0
D1_CONTROL_PROTOCOL=2
```

如果服务未运行或版本不一致，在确认机械臂已被支撑且周围空间安全后执行：

```bash
cd ~/Project/unitree_arm
./deploy/update_d1_onboard_sdk.sh --confirm UPDATE_D1_ONBOARD_SDK
```

成功标志：

```text
D1 onboard SDK update passed. The d1-control.service is enabled and active.
```

### 2.8 一键配置环境参数
机械臂服务依赖 D1 通信网卡、RealSense 序列号、ROS Domain、感知模型和设备外参等参数。本项目提供一键配置脚本，可自动探测 Go2平台、D1 通信网卡和腕部 RealSense，并填入已经验证的默认配置。

1. 执行脚本前，需先将 D1 和 RealSense D435i 连接至 Go2 扩展坞，确认在 Go2 终端执行 `lsusb | grep -i realsense` 能看到 RealSense，且执行 `ping -c 3 192.168.123.100` 可以收到回复。
2. 执行：

```bash
cd ~/Project/unitree_arm
./deploy/configure_d1_manipulation.sh
```

脚本将生成`src/d1_bringup/config/real_machine.local.yaml`并自动填写可探测项和项目默认值，各参数说明如下，需人工确认或修改的参数详见`是否需要人工配置`。

| 参数 | 作用 | 是否需要人工配置 | Go2 部署要求 |
| --- | --- | --- | --- |
| `deployment.platform` | 标识部署平台并校验主机架构 | 否 | 脚本应自动填写 `go2` |
| `deployment.ros_domain_id` | ROS 2 节点发现与通信使用的 Domain ID | 是 | 必须与 Go2 导航、运动控制和任务节点保持一致，范围为 `0–232` |
| `perception.model_path` | YOLO 目标识别模型路径 | 通常需要 | 默认模型仅适用于项目当前验证过的物体和环境；部署到其他场景时，应替换为针对实际物体和现场数据训练并验证过的模型 |
| `arm.serial_no` | 选择个体机械臂的 URDF 修正配置，目前主要用于加载关节限位覆盖 | 通常否 | 只有机械臂存在需要单独修正的关节限位或其他 URDF 参数，并已在 `hardware_profiles.yaml` 中建立对应配置时才填写 |
| `arm.network_interface` | Go2 与 D1 收发原生控制和反馈数据所用的有线网卡 | 通常否 | 唯一候选会自动填写；未检测到或存在多个候选时，根据 `ip -br address` 手工选择 |
| `wrist_camera.driver_location` | 指定 RealSense 驱动在本机还是另一台 ROS 2 主机上运行 | 通常否 | D435i 连接 Go2 时保持 `local` |
| `wrist_camera.serial_no` | 指定作为腕部相机的 RealSense | 通常否 | 唯一 RealSense 会自动填写；存在多个候选时手工填写腕部相机序列号 |
| `wrist_camera.frame_rate_hz` | RGB、深度和对齐深度流的帧率 | 通常否 | 默认 `15` Hz；根据 Go2 算力和网络带宽可改为 `6`、`15` 或 `30` Hz |
| `wrist_camera.link6_to_camera_link.translation_xyz_m` | `Link6` 原点到腕部相机坐标系原点的平移，单位为米 | 通常否 | 相机与支架的安装关系与流程1一致时沿用默认值；安装位置改变后重新标定 |
| `wrist_camera.link6_to_camera_link.quaternion_xyzw` | `Link6` 到腕部相机坐标系的旋转四元数，顺序为 `x,y,z,w` | 通常否 | 相机与支架的安装姿态与流程1一致时沿用默认值；安装姿态改变后重新标定 |
| `site.go2_base_to_arm_base.translation_xyz_m` | Go2 基座到机械臂基座的安装平移，单位为米 | 是 | 必须填写实测值，Go2 部署不允许继续使用默认的零平移 |
| `site.go2_base_to_arm_base.quaternion_xyzw` | Go2 基座到机械臂基座的安装旋转，顺序为 `x,y,z,w` | 是 | 必须填写实测值，Go2 部署不允许继续使用默认的单位四元数 |


3. 对生成的配置文件进行校验，执行：

```bash
./deploy/configure_d1_manipulation.sh --validate
```

终端显示`Configuration: OK`后即可继续后续步骤；若校验失败，根据终端提示修改对应参数。

### 2.9 一键启动机械臂服务
1. 确认 D1 与 RealSense 均已连接至 Go2。
2. 确认机械臂已摆放为收纳姿态（姿态详见流程1的“一键启动机械臂控制栈”章节）
3. 执行：

```bash
# 在 Go2 终端执行
cd ~/Project/unitree_arm
./deploy/start_d1_manipulation.sh
```

Go2 正式部署默认不启动 RViz。启动过程会检查板端服务版本、关节反馈、RealSense、IMU、TF、MoveIt 和业务接口，并且不会自动执行抓放动作。只有看到以下横幅才表示机械臂服务就绪：

```text
************************************************************
* D1 REAL CONTROL STACK READY                              *
* pick_object and drop_object are ready to accept commands *
************************************************************
```

没有出现横幅时不要调用任务接口。运行日志位于 Go2 宿主机的 `unitree_arm/log/runtime/`。

### 2.10 机械臂功能验证
功能验证分为基础部署验收和实际抓放验收。保持机械臂服务终端运行，在 Go2 上打开另一个终端并进入容器：

```bash
# 在GO2新开一个终端并执行
sudo docker exec -it unitree-d1-manipulation-arm64 /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
export ROS_DOMAIN_ID=$(python3 -c 'import yaml; print(yaml.safe_load(open("/workspace/src/d1_bringup/config/real_machine.local.yaml"))["deployment"]["ros_domain_id"])')
```

#### 2.10.1 基础部署验收
检查公共接口：

```bash
# 在 Go2 终端执行
ros2 action list | grep '^/arm/tasks/'
ros2 topic list | grep '^/arm/task_status$'
```

正确结果至少包含：

```text
/arm/tasks/drop_object
/arm/tasks/pick_object
/arm/task_status
```

读取当前机械臂状态：

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

机械臂以 STOWED 姿态正常启动时，核心状态应为：

```yaml
state: 1
payload_state: 0
canonical_pose: 0
```

分别表示：

```text
READY_STOWED
PAYLOAD_EMPTY
POSE_STOWED
```

长期停留在 `INITIALIZING` 或进入 `FAULTED` 时，不要发送抓放任务，应根据 `/arm/task_status` 中的 `failure_code`、`detail` 和 `log/runtime/` 排查问题。

继续验证腕部相机 RGB、aligned depth 和 IMU 数据正在持续发布：

```bash
ros2 topic hz /wrist_camera/color/image_raw
ros2 topic hz /wrist_camera/aligned_depth_to_color/image_raw
ros2 topic echo /wrist_camera/imu --once
```

`ros2 topic hz` 显示稳定帧率后按 `Ctrl+C` 结束当前检查，再执行下一条命令。RGB 和 aligned depth 的帧率应接近配置值（默认15hz），且 IMU 应能返回持续更新的数据。需要直接检查画面时，可在具有图形桌面的 Go2 调试会话中使用 `./deploy/start_d1_manipulation.sh --rviz`重启服务，按照流程1的 RViz 标准检查机械臂模型、RGB 和 aligned depth。

#### 2.10.2 实际抓放验收
1. 将 Go2 保持在稳定贴地姿态，清空机械臂和 Go2 周围的运动空间
2. 在机械臂底座Y方向0.35m放一个黄色立方体。执行前必须替换为 Go2 实际估计的坐标和对应 `frame_id`。

```bash
ros2 action send_goal /arm/tasks/pick_object d1_interfaces/action/PickObject "{target: {header: {frame_id: base_link}, point: {x: 0.0, y: 0.35, z: -0.20}}, stop_after: 4}" --feedback
```

> 注意：执行过程中按下 `Ctrl+C` 会请求机械臂停止运动并保持当前姿态。确认机械臂处于安全恢复范围、运动空间已经清空后，可在 `/workspace` 下执行 `./scripts/arm_stowed_control.zsh --confirm STOWED_MOVE` 请求恢复至 STOWED；该工具不能保证从任意姿态安全恢复。
>

抓取验收通过时，物体应被实际抓起，机械臂进入 CARRY 姿态，Action 返回 `success: true`、`outcome: 0`、`final_task_state: 2` 和 `payload_state: 1`。

3. 确认抓取成功后，再调用投放接口：

```bash
ros2 action send_goal /arm/tasks/drop_object d1_interfaces/action/DropObject "{target: {header: {frame_id: base_link}, point: {x: 0.00, y: 0.40, z: -0.20}}}" --feedback
```

投放验收通过时，物体应被实际释放，机械臂返回 STOWED 姿态，Action 返回 `success: true`、`outcome: 0`、`returned_to_stowed: true`、`final_task_state: 1` 和 `payload_state: 0`。最后执行：

```bash
ros2 topic echo /arm/task_status --once --qos-reliability reliable --qos-durability transient_local
```

确认机械臂回到 `READY_STOWED`。

4. 在机械臂服务终端按一次 `Ctrl+C`。启动脚本会停止 ROS 子进程并删除运行容器。
5. 执行以下命令确认容器已经退出：

```bash
sudo docker ps --format '{{.Names}}' \
  | grep '^unitree-d1-manipulation-arm64$'
```

没有输出表示服务已经停止。如果仍有同名容器，不要启动第二套机械臂服务，应先排查上一次退出失败的原因。

### 2.11 Go2 任务节点集成检查
Go2 任务节点必须使用与机械臂服务相同的 `ROS_DOMAIN_ID`，并依赖 `d1_interfaces` 中的消息和 Action 定义。机械臂模块只向机器狗业务侧提供：

```text
/arm/tasks/pick_object
/arm/tasks/drop_object
/arm/task_status
```

具体请求参数、返回字段、状态枚举和失败处理方式详见：[D1 机械臂抓放 API 文档](../docs/arm_manipulation_api_cn.md)



## 常见问题

### 板端服务版本或协议不匹配
重新执行 D1 板端控制服务更新。禁止跳过版本检查或自动回退到未知板端程序。

### RealSense 预检失败
确认 D435i 已连接至 Go2 扩展坞的 USB 3.0 接口，并检查相机序列号、驱动位置和帧率配置。如果相机由另一台 ROS 2 计算机发布，应将 `wrist_camera.driver_location` 改为 `remote`，并保证两端使用相同的 `ROS_DOMAIN_ID`。

### 状态为 `FAULTED`
读取 `/arm/task_status` 中的 `failure_code` 和 `detail`，并检查 Go2 宿主机的 `unitree_arm/log/runtime/`。重新执行任务前，必须确认机械臂实际姿态、关节反馈、碰撞状态和板端控制服务均正常。

### Go2 任务节点无法发现机械臂接口
确认机械臂容器与 Go2 任务节点使用相同的 `ROS_DOMAIN_ID`，并检查调用方是否已经构建和加载 `d1_interfaces`。Docker 使用 host 网络，一般不需要额外映射 ROS 2 端口。
