# TITA 程序部署与键盘控制指南

本文接续 [TITA 上电与 SSH 连接指南](TITA_USB_SSH连接指南.md)，说明如何将当前仓库及自己训练的 `flat.onnx` 部署到 TITA。

## 1. 前提与安全要求

- 已通过 USB 或 Wi-Fi 成功 SSH 登录机器人。
- 机器人具备 Ubuntu 22.04 / ROS 2 Humble 环境，且能访问所需软件源。
- 已确认当前运控固件与仓库 `tita_robot` 驱动库兼容。README 对旧版 `d1-ros2` 有分支兼容警告，不要直接套用到未知版本的 TITA。
- 新模型和 YAML 已完成离线检查及仿真验证。能够编译或加载模型不代表策略可以安全用于真机。
- 真机接管前准备可靠支撑/吊架，人员、线缆远离关节与轮子，并确认当前固件在 Direct Mode 下可用的硬件停机方式。

**SSH 断开、退出键盘节点或 `Ctrl+C` 都不能视为急停。不要同时运行原厂 SDK Demo 和本仓库关节控制器。**

## 3. 上传当前源码

以下命令在 **外接电脑终端**执行。

进入电脑上的当前仓库目录：

```bash
cd /你的路径/ddt_ros2_control
```

创建机器人工作空间并上传：

```bash
ssh "robot@$tita_ip" 'mkdir -p ~/ws/src/ddt_ros2_control'

rsync -av \
  --exclude='.git/' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  ./ "robot@$tita_ip:~/ws/src/ddt_ros2_control/"
```

按连接方式设置 `tita_ip`。

## 4. 安装依赖并编译

以下命令在 **机器人 SSH 终端**执行。安装软件需要机器人可以访问软件源；电脑连接机器人热点本身不代表机器人已联网。

### 4.1 ROS 依赖

若 `/opt/ros/humble/setup.bash` 不存在，应先按官方说明准备 ROS 2 Humble 环境。

```bash
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake-auto \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-generate-parameter-library \
  ros-humble-eigen3-cmake-module \
  ros-humble-realtime-tools \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  libeigen3-dev \
  can-utils
```

### 4.2 ONNX Runtime

本仓库默认链接 ONNX Runtime。先检查已有库，避免盲目覆盖：

```bash
ldconfig -p | grep onnxruntime
```

如果尚未安装，并且 `uname -m` 为 `aarch64`，可参考 README 的 1.10.0 版本安装 ARM64 包：

```bash
cd ~
wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-aarch64-1.10.0.tgz
tar xf onnxruntime-linux-aarch64-1.10.0.tgz

sudo cp -a onnxruntime-linux-aarch64-1.10.0/include/* /usr/local/include/
sudo cp -a onnxruntime-linux-aarch64-1.10.0/lib/* /usr/local/lib/
ldconfig -p | grep onnxruntime
```

如果是 x86_64，需要对应的 x64 包。新模型的 opset/算子可能与旧版 Runtime 不兼容，应先测试模型加载；更换 Runtime 版本也可能需要调整当前 C++ 推理器 API，不能仅升级库就假定兼容。

### 4.3 构建真机所需包

```bash
cd ~/ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to \
  rl_controller hardware_bridge tita_description keyboard_controller \
  --parallel-workers 2 \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

这里显式编译 `tita_description`，因为硬件启动需要模型描述，但 `hardware_bridge` 未声明该包依赖。编译失败时先解决错误，不要继续真机接管。

## 5. 检查自己的模型与配置

当前 TITA 使用：

```text
controller/rl_controller/config/tita/flat.onnx
controller/rl_controller/config/tita/controllers.yaml
```

两者必须成对更新。当前配置包含 `output_name: nn_output`、`policy_type: np3o`、33 维单帧观测、10 帧历史和 8 个动作。对应实现传入一个展平历史张量，通常为固定形状 `[1, 330]`，输出通常为 `[1, 8]`。

部署前必须核对训练代码：

- 模型输入数量、名称、维度及输出名称；当前推理器没有处理动态维度。
- 观测顺序、缩放、坐标系、默认关节角及历史顺序。
- 关节顺序、符号和 `reindex`。
- `time_interval` 是否等于训练 `dt × decimation`；当前为 `0.02` 秒，即 50 Hz 策略推理。
- `action_scales`、PD 参数、轮关节控制公式及 `control_type`。


### 5.1 确认启动时使用的资源

在 **机器人终端**执行：

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

PKG_SHARE=$(ros2 pkg prefix --share rl_controller)
cat "$PKG_SHARE/config/tita/controllers.yaml"

sha256sum \
  src/ddt_ros2_control/controller/rl_controller/config/tita/flat.onnx \
  "$PKG_SHARE/config/tita/flat.onnx"
```

确认安装目录 YAML 是新配置，两个模型哈希一致，并可与电脑源模型哈希比较。

### 5.2 后续仅更新模型和 YAML

先让机器人处于安全状态并结束使用模型的控制进程，不要在线覆盖正在使用的文件。

在 **电脑仓库目录**执行：

```bash
rsync -av \
  controller/rl_controller/config/tita/controllers.yaml \
  controller/rl_controller/config/tita/flat.onnx \
  "robot@$tita_ip:~/ws/src/ddt_ros2_control/controller/rl_controller/config/tita/"
```

在 **机器人终端**刷新安装资源：

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to rl_controller \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

重新检查资源，并在安全条件下重新启动控制器。运行中的进程不会自动重新加载模型。

## 6. 上位机接管并启动控制器

**以下步骤会涉及电机控制。只有固件兼容性、模型验证、支撑和停机方式均已确认后才执行。**

### 6.1 关闭原厂运控

官方 TITA 文档使用 `tita-bringup.service`，README 中的 `d1_bringup.service` 是 D1 示例。以第 2 节查到的实际服务为准。

如果确认当前运控服务确实是 `tita-bringup.service`，在机器人终端执行：

```bash
sudo systemctl stop tita-bringup.service
systemctl is-active tita-bringup.service
```

### 6.2 切换 Direct Mode

在 **机器人终端**执行：

```bash
cd ~/ws/src/ddt_ros2_control
sudo ip link set can0 txqueuelen 5000
bash start.bash
```

脚本向 `can0` 发送两条 CAN FD 指令，用于切换 Direct Mode。

### 6.3 启动控制器

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rl_controller hw.launch.py robot:=tita
```

启动后观察日志。若出现 IMU/电机超时、模型加载失败或控制器错误，不要发送站起或 RL 指令，先按已确认的安全流程停机并排查。

## 7. 键盘控制

在 **电脑另一个终端**重新设置当前连接的 `tita_ip`，再登录：

```bash
# USB；热点连接改为 10.42.0.1
tita_ip=192.168.42.1
ssh -t "robot@$tita_ip"
```

登录后的 **机器人终端**执行：

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run keyboard_controller keyboard_controller_node
```

键盘节点也运行在机器人上，因此不需要配置电脑和机器人之间的 ROS DDS 通信。机器人上的控制器和键盘节点需使用相同的 `ROS_DOMAIN_ID`。

### 7.1 操作顺序

1. 按 `r`，确认速度指令清零。
2. 在保护条件下按 `7`，进入 `transform_up`。
3. 观察姿态和日志，等待站起实际完成，不要仅按固定 2 秒计时。
4. 确认可以安全运行策略后，按 `0` 进入当前第一个 RL 策略 `rl_flat`。
5. 单次、小幅尝试运动按键，按 `r` 清零速度指令。
6. 按 `8`，让机器人趴下。


## 8. 参考资料

- [本仓库 README](../README.md)
- [TITA 上电与 SSH 连接指南](TITA_USB_SSH连接指南.md)
- [TITA 官方开发手册](https://tita-development-manual-uc.readthedocs.io/zh-cn/latest/index.html)
- [官方 Ubuntu 系统教程](https://tita-development-manual-uc.readthedocs.io/zh-cn/latest/pages/ubuntu-flash.html)

官方 Quick Start 的 `use-sdk mode` 属于原厂高层 SDK 路线，不是本仓库 Direct Mode 部署流程，请勿混用。
