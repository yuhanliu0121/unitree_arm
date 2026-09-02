# D1 ROS 2 使用与功能验证教程

本文说明如何从发布 ZIP 构建 `d1_constrained_description`，并验证纯
URDF 可视化和真机状态同步。本文以 ROS 2 Humble、Ubuntu 22.04 和
默认网卡 `eno1` 为例。

文中的灰色代码框都是终端命令，可以逐段复制粘贴；不要复制代码框
外的说明文字。路径中的 `~` 表示当前用户的主目录，例如
`/home/student`。一条命令成功时可能没有任何输出，这也是正常的。

## 1. 从零安装环境与依赖

以下命令按顺序逐段执行。每输入一段，都应等待它执行完成且没有红色
报错，再继续下一段。需要联网，并需要当前账户具有 `sudo` 权限。

### 1.1 确认操作系统和 CPU

打开终端（Ubuntu 中按 `Ctrl+Alt+T`），执行：

```bash
grep PRETTY_NAME /etc/os-release
uname -m
```

本教程要求：

- 系统显示 `Ubuntu 22.04`；
- CPU 显示 `x86_64` 或 `aarch64`；
- 使用带桌面的 Ubuntu，因为 RViz 需要图形界面。

如果系统是 Ubuntu 20.04、24.04 或其他版本，不要继续照抄本教程。
ROS 2 版本与 Ubuntu 版本必须匹配，本包验证使用的是 ROS 2 Humble。

### 1.2 安装基础工具

```bash
sudo apt update
sudo apt install -y \
  locales software-properties-common curl git unzip \
  build-essential cmake g++ \
  python3-pip python3-colcon-common-extensions

sudo add-apt-repository universe
sudo apt update
```

`sudo` 要求输入密码时，屏幕不会显示星号，这是 Linux 的正常行为；
输入完成后按回车即可。配置 UTF-8 语言环境：

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

最后一条命令中应能看到 `LANG=en_US.UTF-8`。

### 1.3 安装 ROS 2 Humble

先让 Ubuntu 识别 ROS 2 官方软件源：

```bash
sudo apt install -y curl

export ROS_APT_SOURCE_VERSION=$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' \
  | awk -F'"' '{print $4}'
)

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

如果下载命令报网络错误，先检查浏览器能否访问 GitHub，再重试这一
小节；不要跳过报错继续。安装 ROS 2 和本包所需工具：

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-ros
```

加载 ROS 2 环境并检查：

```bash
source /opt/ros/humble/setup.bash
echo "$ROS_DISTRO"
ros2 --help
colcon --help
```

`echo` 应输出 `humble`，后两条命令应显示帮助信息。为了让以后打开的
Bash 终端自动加载 ROS 2，只需执行一次：

```bash
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

如果 `echo $SHELL` 显示 `/bin/zsh`，请在本文所有命令中把
`setup.bash` 换成 `setup.zsh`，并把上一条命令中的 `~/.bashrc`
换成 `~/.zshrc`。

### 1.4 安装 Unitree SDK2 和配套 Cyclone DDS

本 ROS 2 包中的真机桥接程序依赖 Unitree SDK2。SDK2 仓库已经包含
与它配套的 Cyclone DDS/C++ 头文件和预编译库；执行安装后，它们会
一起进入 `/usr/local`。不要再单独安装另一套 Cyclone DDS，以免
头文件与动态库版本混用。

先安装 Unitree SDK2 的编译依赖：

```bash
sudo apt update
sudo apt install -y \
  cmake g++ build-essential \
  libyaml-cpp-dev libeigen3-dev libboost-all-dev \
  libspdlog-dev libfmt-dev
```

下载并切换到本发布包验证过的版本：

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
git checkout 7740f8b
```

如果提示目录 `unitree_sdk2` 已存在，不要反复下载；执行：

```bash
cd ~/unitree_sdk2
git fetch --all
git checkout 7740f8b
```

编译并安装到本包默认查找的 `/usr/local`：

```bash
cd ~/unitree_sdk2
cmake -S . -B build \
  -DBUILD_EXAMPLES=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig
```

检查关键文件是否存在：

```bash
test -f /usr/local/include/unitree/robot/channel/channel_subscriber.hpp \
  && echo "Unitree headers: OK"
test -f /usr/local/include/ddscxx/dds/dds.hpp \
  && echo "Cyclone DDS C++ headers: OK"
test -f /usr/local/lib/libunitree_sdk2.a \
  && echo "Unitree library: OK"
test -f /usr/local/lib/libddsc.so \
  && echo "Cyclone DDS C library: OK"
test -f /usr/local/lib/libddscxx.so \
  && echo "Cyclone DDS C++ library: OK"
```

五行都应以 `OK` 结束。若没有输出，对应文件没有安装成功，应回到
本小节检查第一条报错，而不是继续编译 ROS 2 包。

安装依据可参考：

- ROS 2 Humble 官方安装文档：
  <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>
- Unitree SDK2 官方仓库：
  <https://github.com/unitreerobotics/unitree_sdk2>

## 2. 校验、解压与编译

将 ZIP 和 `.zip.sha256` 放在同一目录：

```bash
cd ~/Downloads
sha256sum -c d1_constrained_description_20260728.zip.sha256

mkdir -p ~/d1_ros2_ws/src
cd ~/d1_ros2_ws/src
unzip ~/Downloads/d1_constrained_description_20260728.zip

cd ~/d1_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select d1_constrained_description
source install/setup.bash
ros2 pkg prefix d1_constrained_description
```

最后一条命令应打印当前工作空间中的包路径。每个新终端都需要重新
执行 ROS 和工作空间的两条 `source` 命令。

## 3. 验证一：滑块纯可视化

此模式不会连接或控制真机：

```bash
ros2 launch d1_constrained_description display.launch.py
```

在 `Joint State Publisher` 窗口中分别拖动 `Joint0`～`Joint5`。RViz
中的各段模型应连续旋转，不应反向、跳变或脱离相邻连杆。拖动
`Joint6` 时，`0 m` 表示闭合，`0.03 m` 表示约 60 mm 开口。夹指尖
中心的彩色 `Grasp TCP` 坐标轴应随末端运动，但不随夹爪开合漂移。

功能通过标准：

- 所有关节滑块都能独立更新模型；
- 全限位范围内没有模型“脱臼”；
- 两根夹指对称开合；
- `Grasp TCP` 始终位于两根夹指尖端之间。

完成后在启动终端按 `Ctrl+C`，不要让此模式与真机同步模式同时运行。

## 4. 验证二：真机只读状态同步

先连接机械臂网线并确认网卡名称：

```bash
ip -br link
```

启动实时可视化：

```bash
ros2 launch d1_constrained_description live_display.launch.py interface:=eno1
```

该启动文件不运行滑块程序。内置桥接节点只订阅 Unitree DDS
`current_servo_angle`，将 J0～J5 从度转换为弧度，并把夹爪电机角度
映射到 `Joint6` 的 `0～0.03 m`，然后发布 ROS 2 `/joint_states`。
它不包含 Unitree 指令发布器，不会使能、卸力或驱动机械臂。

在另一个已 `source` 的终端检查：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 topic info /joint_states --verbose
ros2 run tf2_ros tf2_echo base_link tcp_link
```

`/joint_states` 应持续更新，且通常只有一个发布者。真机已有安全姿态
变化时，RViz 应同步显示相同的关节方向、幅度、夹爪开合和 TCP 姿态。
不要为了验证显示而擅自卸力或推动仍处于使能状态的机械臂。

## 5. 常见问题

- **`Unable to locate package ros-humble-...`**：先确认系统确实是 Ubuntu
  22.04，再重新执行 1.3 节的软件源安装命令和 `sudo apt update`。
- **CMake 报 `Missing UNITREE_SDK2...` 或 `Missing CYCLONEDDS...`**：
  Unitree SDK2 没有完整安装到 `/usr/local`。重新执行 1.4 节，并确认
  五项文件检查全部输出 `OK`。
- **运行时报 `libddsc.so` 或 `libddscxx.so` 找不到**：执行
  `sudo ldconfig`，关闭当前终端，重新打开后再次 `source` 和运行。
- **找不到包**：重新执行 `source ~/d1_ros2_ws/install/setup.bash`。
- **没有关节反馈**：确认 `interface` 名称正确，并检查机械臂网络、
  `unitree_sdk2`、Cyclone DDS/C++ 和 DDS 反馈话题。
- **模型不动或抖动**：停止其他 `display.launch.py`、桥接程序或
  `/joint_states` 发布者，再用 `ros2 topic info /joint_states --verbose`
  确认只剩一个发布者。
- **修改后没有生效**：重新 `colcon build`，重新 `source`，并彻底
  关闭后再启动 launch；URDF 在启动时载入，不会自动热更新。

已知精度范围和未验证参数见 `KNOWN_ISSUES.md`。
