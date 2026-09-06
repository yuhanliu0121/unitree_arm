# Unitree D1 机械臂抓放服务部署教程

中文 | [English](README.md)

本文说明 D1 板载控制服务、amd64 开发环境和 arm64 Go2 真机环境的部署与验收方法。机械臂模块对外提供 `/arm/tasks/pick_object`、`/arm/tasks/drop_object` 和 `/arm/task_status` 三个 ROS 2 接口。

## 1. 部署要求

### 1.1 硬件要求

| 设备 | 用途 | 是否必需 |
| --- | --- | --- |
| Unitree D1-550 | 执行抓放动作，包含 D1 板载计算机 | 必需 |
| Intel RealSense D435i | 腕部 RGB-D 感知和重力测量 | 必需 |
| amd64 Ubuntu 电脑 | 开发、仿真和独立真机调试 | 开发阶段使用 |
| 带 arm64 扩展坞的 Go2 EDU | 最终运行机械臂、导航和机器狗业务节点 | 正式部署使用 |
| 网线 | 连接运行机械臂服务的主机与 D1 | 必需 |

部署和真机验证前必须固定 D1 底座，支撑机械臂，并清除运动范围内的人员、线缆和障碍物。

### 1.2 主机前置软件

部署脚本支持 Ubuntu 20.04/22.04。Docker、ROS 2、MoveIt、Conda 和 Python 感知依赖均由部署脚本处理，不需要手动安装。执行脚本前只需安装以下基础工具：

```bash
sudo apt update
sudo apt install -y git python3 python3-yaml openssh-client sshpass tar
```

说明：`sshpass` 仅在没有配置 SSH 密钥时使用。运行环境首次构建需要访问 Ubuntu、GitHub、Conda 和 Python 软件源，并建议预留至少 20 GB 磁盘空间。

D1 板载计算机必须保留原厂 `/home/ubuntu/marm_code`、CMake/C++ 编译工具以及 FashionStar、CSerialPort 和 Unitree SDK 依赖。板载更新脚本会使用这些已有环境进行原生编译，不负责重装缺失的原厂依赖。

在仓库根目录执行以下命令检查文件是否完整：

```bash
test -f deploy/install_d1_runtime_env.sh \
  && test -f deploy/start_d1_manipulation.sh \
  && test -f deploy/configure_d1_manipulation.sh \
  && test -f deploy/update_d1_onboard_sdk.sh \
  && echo "Repository: OK"
```

正确结果：

```text
Repository: OK
```

### 1.3 设备职责与网络关系

| 计算机 | 运行内容 | 处理器架构 |
| --- | --- | --- |
| D1 板载计算机 | 轻量关节控制服务 `d1-control.service` | 与主机 Docker 无关 |
| amd64 开发电脑 | ROS 2、MoveIt、感知、抓放服务和 MuJoCo | amd64 |
| Go2 扩展坞 | 正式部署时运行 ROS 2、MoveIt、感知和抓放服务 | arm64 |

开发调试时由 amd64 电脑连接 D1；正式部署时由 Go2 扩展坞连接 D1。两种情况下，运行机械臂服务的主机都必须有一张网卡位于 `192.168.123.0/24` 网段，并能够访问默认地址 `192.168.123.100`。RealSense 接在当前运行机械臂服务的主机上时使用 `local` 模式；只有相机由另一台 ROS 2 计算机发布时才使用 `remote` 模式。

## 2. 选择部署流程

### amd64 开发电脑

用于仿真、RViz 和独立真机调试，按以下顺序执行：

1. 执行 `xhost +` 并运行 `install_d1_runtime_env.sh`，构建 amd64 镜像、创建长期开发容器并构建工作区；
2. 进入开发容器并完成 MuJoCo 仿真验收；
3. 连接设备，检查或更新 D1 板载控制服务；
4. 在开发容器内填写并校验真机配置；
5. 在开发容器内执行 `real_bringup.zsh --rviz`，验证三个 ROS 2 接口并进行受控真机测试。

### arm64 Go2 扩展坞

用于最终任务部署，按以下顺序执行：

1. 将本仓库放入 Go2 代码仓库的 `manipulator/` 目录；
2. 检查或更新 D1 板载控制服务；
3. 填写 Go2 网卡、ROS Domain、相机和安装外参；
4. 在 Go2 上执行 `install_d1_runtime_env.sh`，生成 arm64 镜像并构建工作区；
5. 执行 `start_d1_manipulation.sh`；
6. 从 Go2 业务节点验证并调用三个 ROS 2 接口。

D1 板载服务只需保持与当前仓库版本一致，不需要分别为 amd64 和 arm64 安装两次。哪台主机能够通过 `192.168.123.100` 访问 D1，就可以从哪台主机执行板载更新。

## 3. 检查或更新 D1 板载控制服务

D1 原厂控制服务缺少当前抓放任务需要的完整七关节协调命令、执行反馈和协议版本校验，因此需要部署仓库内的增强服务。原厂程序不会被覆盖，并保留为回滚路径。本步骤只在首次部署或板端代码更新后执行。

### 3.1 配置 D1 有线网络

将主机上连接 D1 的网卡配置为 `192.168.123.0/24` 网段内的未占用地址，例如：

```text
IPv4 地址：192.168.123.162
子网掩码：255.255.255.0
```

验证连接：

```bash
ping -c 3 192.168.123.100
```

正确结果应包含：

```text
3 packets transmitted, 3 received, 0% packet loss
```

如果全部丢包，检查 D1 供电、网线和主机 IPv4 配置，不要执行板载更新。

### 3.2 检查现有板载版本

```bash
ssh ubuntu@192.168.123.100 'systemctl is-active d1-control.service && cat /home/ubuntu/marm_code/build/d1-control-release.env'
```

当前仓库对应的正确结果为：

```text
active
D1_CONTROL_VERSION=0.1.0
D1_CONTROL_PROTOCOL=2
```

如果服务、版本和协议均一致，可跳过更新。修改过 `arm.onboard_control.host` 或 `ssh_user` 时，应同步替换上述 SSH 地址或用户。

### 3.3 更新板载服务

支撑机械臂并清理工作区，然后执行：

```bash
./deploy/update_d1_onboard_sdk.sh --confirm UPDATE_D1_ONBOARD_SDK
```

脚本默认使用 D1 地址 `192.168.123.100`、用户 `ubuntu` 和密码 `123`，在 D1 上原生编译并执行协议测试，备份现有部署，然后安装并启动新服务。非默认设备可通过 `--host`、`--user` 和 `--password` 覆盖。脚本不会发送关节运动命令，但切换控制服务时机械臂可能失去保持力。

成功标志：

```text
D1 onboard SDK update passed. The d1-control.service is enabled and active.
```

更新失败时不要继续启动控制栈。脚本会尝试恢复旧部署，并输出板端备份目录供排查。

## 4. amd64 开发电脑部署

### 4.1 检测并配置开发机参数

在仓库根目录执行：

```bash
./deploy/configure_d1_manipulation.sh
```

脚本会识别 amd64 平台，探测 D1 网卡和唯一连接的 RealSense，并生成仅供本机使用的 `src/d1_bringup/config/real_machine.local.yaml`。配置文件参数如下：

| 参数 | 作用 | 默认值或获取方式 | 何时需要修改 |
| --- | --- | --- | --- |
| `deployment.platform` | 标识部署平台并校验主机架构 | 自动填写 `amd64` | 一般不修改 |
| `deployment.ros_domain_id` | ROS 2 节点发现和通信使用的 Domain ID | 默认 `31` | 与其他 ROS 2 节点通信时改成相同值，范围 `0–232` |
| `perception.model_path` | YOLO 模型路径 | 默认正式物体模型 `mcislab_trash_collect.pt` | 纸制物体调试时改为 `paper_objects_dev_best.pt`，或换用其他已验证模型 |
| `arm.serial_no` | 选择个体机械臂的 URDF 修正配置，目前主要加载关节限位覆盖 | 默认留空，使用基础 URDF | 只有已建立并验证对应 `hardware_profiles.yaml` 配置时才填写 |
| `arm.network_interface` | 与 D1 通信使用的有线网卡 | 根据 `192.168.123.0/24` 网段自动探测 | 未探测到或存在多个候选时手工填写 |
| `wrist_camera.driver_location` | 指定 RealSense 驱动运行位置 | 默认 `local` | 相机由另一台 ROS 2 主机发布时改为 `remote` |
| `wrist_camera.serial_no` | 指定腕部 RealSense | 唯一相机自动探测 | 未探测到或存在多个候选时手工填写 |
| `wrist_camera.frame_rate_hz` | RGB 和深度流帧率 | 默认 `15` Hz | 按算力和带宽改为 `6`、`15` 或 `30` Hz |
| `wrist_camera.link6_to_camera_link.translation_xyz_m` | `Link6` 到相机原点的平移，单位米 | 当前支架的手眼标定值 | 安装关系改变后重新标定 |
| `wrist_camera.link6_to_camera_link.quaternion_xyzw` | `Link6` 到相机的旋转四元数，顺序 `x,y,z,w` | 当前支架的手眼标定值 | 安装关系改变后重新标定 |
| `site.go2_base_to_arm_base.translation_xyz_m` | Go2 基座到机械臂基座的平移，单位米 | 独立测试使用零平移 | 安装到 Go2 后填写实测值 |
| `site.go2_base_to_arm_base.quaternion_xyzw` | Go2 基座到机械臂基座的旋转四元数 | 独立测试使用单位四元数 | 安装到 Go2 后填写实测值 |

查看网卡名称：

```bash
ip -br address
```

应能看到某张网卡具有 `192.168.123.x/24` 地址。填写完成后执行：

```bash
./deploy/configure_d1_manipulation.sh --validate
```

正确结果为 `Configuration: OK`。验证失败时根据输出逐项修正。不要直接修改受 Git 管理的 `real_machine.yaml`，也不要编辑自动生成的 `real_machine.effective.yaml`。

### 4.2 构建 amd64 环境

```bash
./deploy/install_d1_runtime_env.sh
```

脚本会自动安装缺失的 Docker，构建 Ubuntu 22.04 + ROS 2 Humble 的 amd64 镜像，创建长期运行的 `unitree-d1-manipulation-dev-amd64` 开发容器，并在其中构建工作区和 commissioning tools。当前仓库挂载到 `/workspace`，业务源码及 `build/`、`install/`、`log/` 均保留在宿主机仓库中。

成功标志：

```text
D1 runtime environment is ready.
  architecture: amd64
  image:        unitree-d1-manipulation:amd64
  dev container: unitree-d1-manipulation-dev-amd64 (running)
```

继续检查：

```bash
test -f install/setup.bash && echo "ROS workspace: OK"
docker image inspect unitree-d1-manipulation:amd64 >/dev/null && echo "Docker image: OK"
```

正确结果：

```text
ROS workspace: OK
Docker image: OK
```

如果当前用户只能通过管理员权限访问 Docker，将命令改为 `sudo docker ...`。构建或测试失败时应检查终端中的第一条错误和仓库下的 `log/`，不能进入真机测试。

修改业务代码后直接在开发容器中重新执行 `colcon build --symlink-install` 即可。只有 Docker 环境发生变化时才需要重新运行安装脚本；重新安装前应先停止现有开发容器。

### 4.3 进入开发容器并启动 amd64 控制栈

```bash
docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
cd /workspace
./scripts/real_bringup.zsh --rviz
```

启动过程会检查板端版本、关节反馈、RealSense、IMU、TF、MoveIt 和业务接口，并且不会自动执行抓放动作。只有看到以下横幅才表示服务就绪：

```text
************************************************************
* D1 REAL CONTROL STACK READY                              *
* pick_object and drop_object are ready to accept commands *
************************************************************
```

没有出现横幅时不要调用任务接口。运行日志位于宿主机的 `log/runtime/`。

## 5. arm64 Go2 部署

### 5.1 放置代码

将机械臂代码放在 Go2 总仓库的独立目录中：

```text
Go2_Codebase/
└── manipulator/
    ├── src/
    ├── runtime/
    ├── third_party/
    └── deploy/
```

以下命令均在 `Go2_Codebase/manipulator/` 中执行。部署脚本通过自身位置确定工作区，不依赖 `/home/tony/...` 等开发机绝对路径。

### 5.2 检测并配置 Go2 参数

在 Go2 的 `manipulator/` 目录执行：

```bash
./deploy/configure_d1_manipulation.sh
```

脚本会识别 arm64 Go2 平台，探测 D1 网卡和唯一连接的 RealSense，并生成 `src/d1_bringup/config/real_machine.local.yaml`。编辑其中标记为 `REQUIRED` 的字段：

| 配置项 | arm64 Go2 配置 |
| --- | --- |
| `deployment.ros_domain_id` | 与 Go2 导航及任务节点保持一致 |
| `perception.model_path` | 默认使用正式物体模型 `mcislab_trash_collect.pt`；部署其他模型时再修改 |
| `arm.serial_no` | 可留空并使用基础 URDF；只有准备好对应机械臂的硬件配置后才填写序列号 |
| `arm.network_interface` | 唯一候选会自动填写；否则手工填写 Go2 上连接 D1 的实际网卡名 |
| `wrist_camera.frame_rate_hz` | 按整机网络和算力预算选择 6、15 或 30 Hz |
| `wrist_camera.serial_no` | 唯一 RealSense 会自动填写；否则手工填写腕部相机序列号 |
| `wrist_camera.link6_to_camera_link` | 相机安装与开发验证时一致可沿用默认值；安装关系改变后必须重新标定 |
| `site.go2_base_to_arm_base` | 必须填写实测的 Go2 到 D1 安装外参 |

填写完成后执行：

```bash
./deploy/configure_d1_manipulation.sh --validate
```

只有出现 `Configuration: OK` 才能继续构建。不要直接修改受 Git 管理的 `real_machine.yaml`，也不要编辑自动生成的 `real_machine.effective.yaml`。再执行以下命令确认 D1 网卡：

```bash
ip -br address
ping -c 3 192.168.123.100
```

必须同时满足：配置中的网卡存在、地址位于 `192.168.123.0/24`、D1 可以连通。

### 5.3 构建 arm64 环境

```bash
./deploy/install_d1_runtime_env.sh
```

该命令必须直接在 Go2 arm64 主机上执行。它会原生构建 arm64 镜像和 ROS 2 工作区，无需从 amd64 开发机复制 `build/` 或 `install/` 产物。

成功标志：

```text
D1 runtime environment is ready.
  architecture: arm64
  image:        unitree-d1-manipulation:arm64
```

继续检查：

```bash
test -f install/setup.bash && echo "ROS workspace: OK"
docker image inspect unitree-d1-manipulation:arm64 >/dev/null && echo "Docker image: OK"
```

两行都显示 `OK` 后才能启动服务。

### 5.4 启动 Go2 控制栈

Go2 通常不直接显示 RViz，正常启动命令为：

```bash
./deploy/start_d1_manipulation.sh
```

启动成功标志与 amd64 相同，必须看到 `D1 REAL CONTROL STACK READY` 横幅。需要在带图形桌面的 Go2 调试会话中显示 RViz 时，才额外传入 `--rviz`。

## 6. 验证 ROS 2 接口

保持控制栈终端运行，在同一主机打开另一个终端。

amd64 执行：

```bash
docker exec -it unitree-d1-manipulation-dev-amd64 /usr/local/bin/d1-docker-entrypoint bash
```

arm64 执行：

```bash
docker exec -it unitree-d1-manipulation-arm64 /usr/local/bin/d1-docker-entrypoint bash
```

如果 Docker 需要管理员权限，在命令前增加 `sudo`。进入容器后执行：

```bash
ros2 action list | grep '^/arm/tasks/'
ros2 topic list | grep '^/arm/task_status$'
```

正确结果至少包含：

```text
/arm/tasks/drop_object
/arm/tasks/pick_object
/arm/task_status
```

读取当前状态：

```bash
ros2 topic echo /arm/task_status --once
```

机械臂以 STOWED 姿态正常启动时应返回 `READY_STOWED`。长期停留在 `INITIALIZING` 或进入 `FAULTED` 时，应先根据状态详情和 `log/runtime/` 排查，不要发送抓放任务。

## 7. Joint6 空载旁路

Joint6 故障但需要验证其他关节和业务流程时，可以显式启动空载旁路。

amd64 在开发容器内使用：

```bash
./scripts/real_bringup.zsh --rviz --joint6-bypass
```

arm64 使用：

```bash
./deploy/start_d1_manipulation.sh --joint6-bypass
```

需要 RViz 时增加 `--rviz`。启动时必须看到：

```text
*** JOINT6 BYPASS REQUESTED: gripper motion and retention will be simulated ***
```

该模式不会实际开合夹爪，只能用于空载流程验证，不能用于真实抓取或验证夹持成功率。

## 8. 停止服务

在控制栈终端按一次 `Ctrl+C`。amd64 下只停止 ROS 控制栈，开发容器继续保留；arm64 下启动脚本会停止 ROS 子进程并删除服务容器。

amd64 检查：

```bash
docker ps --format '{{.Names}}' | grep '^unitree-d1-manipulation-dev-amd64$'
```

需要结束 amd64 开发环境时执行 `docker stop unitree-d1-manipulation-dev-amd64`。

arm64 检查：

```bash
docker ps --format '{{.Names}}' | grep '^unitree-d1-manipulation-arm64$'
```

arm64 检查没有输出表示服务退出完成。如果仍有同名服务容器，不要启动第二套控制栈，应先排查上一次退出失败的原因。

## 9. 常见问题

### Docker 已安装但提示 daemon unavailable

先启动 Docker 服务：

```bash
sudo systemctl enable --now docker
```

然后重新执行安装脚本。当前用户无 Docker 权限时，脚本会尝试使用 `sudo docker`。

### RViz 没有显示

执行 `echo "$DISPLAY"`。输出为空表示当前终端没有图形环境，应使用本地图形会话、NoMachine 等远程桌面，或者不传入 `--rviz`。

### 板载版本或协议不匹配

重新执行第 3 节的板载更新。禁止跳过版本检查或自动回退到未知板端程序。

### RealSense 预检失败

`driver_location: local` 时，相机必须连接到当前运行容器的主机；`remote` 时，外部相机节点必须使用相同的 `ROS_DOMAIN_ID`，并发布配置文件约定的 RGB、aligned depth、CameraInfo 和 IMU 话题。

### 状态为 `FAULTED`

查看 `/arm/task_status` 中的失败阶段和详细原因，并检查宿主机 `log/runtime/`。重新执行任务前必须确认机械臂实际姿态、关节反馈、碰撞状态和板载服务均正常。
