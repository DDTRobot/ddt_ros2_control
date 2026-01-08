# DDT 机器人 sim2sim/sim2real
本仓库是一个基于 ROS 2 的多包工作空间，包含机器人控制器、硬件桥接、仿真桥接（`Mujoco` / `Gazebo` / `Webots`）、交互控制以及机器人模型描述。控制器支持基于有限状态机的策略，并可加载 ONNX 强化学习模型进行推理。
同时提供 [Docker 镜像与启动说明](./docker/README.md)，将依赖与配置固化为可复现环境，便于快速部署、复现实验与跨设备一致运行（sim2sim）。

## 主要功能
- 强化学习控制器（支持 ONNX 推理），基于有限状态机组织控制逻辑
- `ros2_control` 硬件桥接，连接真实机器人驱动库
- `Mujoco` / `Gazebo` / `Webots` 三种仿真环境桥接与示例世界
- 键盘控制与遥控（ELRS）交互模块
- 多机器人模型与描述（`tita`、`d1`(四轮足)、`d1h`(双轮足)）

## 目录结构
- `controller/rl_controller`：基于`ros2_control`框架强化学习控制器
- `hardware`：`hardware_bridge`为`ros2_control` 硬件桥接节点与启动文件，`tita_robot`真实机器人底层驱动接口
- `interaction`：`keyboard_controller`键盘交互节点，`teleop_command`遥控手柄交互节点
- `simulation`：`Mujoco`、`Gazebo`、`Webots` 仿真桥接
- `ros_utils`: 主要为`ros`话题名称相关
- `urdfs`：机器人模型描述文件（`URDF`/`XACRO`/`Mujoco`等）

## 环境与依赖
- 安装onnx推理引擎
``` bash 
# 根据你的系统架构选择x64或者aarch64
wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
tar xvf onnxruntime-linux-x64-1.10.0.tgz
sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
```
- Ubuntu 22.04, ros2 humble, gazebo classic, webots R2025a
安装好ros2 humble后，安装以下依赖：
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```
- 根据需要安装仿真环境所需的依赖
1. `webots`
```bash
sudo apt install ros-humble-webots-ros2 ros-humble-webots-ros2-control
```
2. `gazebo`
```bash
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
```
3. `mujoco`
见下方构建部分

## 构建
以下命令展示了所有可行的编译命令，根据你的实际需要选择必要的组件进行编译。
```bash
# 创建并进入工作空间
mkdir -p ~/ddt_ros2_ws && cd ~/ddt_ros2_ws
# 将本仓库放置于 ~/ddt_ros2_ws/
mv ddt_ros2 src
# 如需使用mujoco，执行下方
git clone -b 3.3.0 https://github.com/google-deepmind/mujoco.git
# 构建
cd ~/ddt_ros2_ws
# 编译rl_controller
colcon build --symlink-install --packages-up-to rl_controller 
# 编译仿真环境
colcon build --symlink-install --packages-up-to webots_bridge # 可替换为gazebo_bridge， mujoco_bridge
# 编译机器人模型描述
colcon build --symlink-install --packages-up-to d1_description d1h_description
# 编译硬件桥接
colcon build --symlink-install --packages-up-to hardware_bridge
# 载入环境
source install/setup.bash
```

## 仿真运行
- Webots 仿真（地形可选 `empty_world`、`stairs`、`uneven`）：
```bash
ros2 launch rl_controller sim_webots.launch.py robot:=d1 terrain:=empty_world
```
- Gazebo 仿真：
```bash
ros2 launch rl_controller sim_gazebo.launch.py robot:=d1h # d1暂时加载不出来
```
- Mujoco 仿真：
在仿真`d1`时，需要手动将`d1h_description`中的`meshes`复制到`d1_description`中，并且修改`d1_description/CMakeLists.txt`中，将meshes注释取消。然后重新编译d1_description
```bash
ros2 launch rl_controller sim_mujoco.launch.py robot:=d1
```

## 硬件运行
硬件桥接依赖真实机器人驱动库（见 `hardware/tita_robot/lib/*/libtita_robot.so`）。需要将代码拷贝在机器上，需要配置硬件编译需要的环境，例如`colcon`后，再进行编译。
```bash
sudo apt install python3-colcon-common-extensions
```
编译运行在硬件上运行运动控制必要的包
```bash
colcon build --symlink-install --packages-up-to rl_controller hardware_bridge
```


- 启动控制器（硬件环境）：
在启动有硬件环境的机器上，需要手动关闭已经启动的运控服务：
```bash
sudo systemctl stop joy_controller.service
sudo systemctl stop rl8_controller.service
sudo systemctl stop rl16_controller.service
```

**如果运行在TITA上，需要注意：**

TITA上电后运控板默认进入Ready Mode，需要运行此[start.bash](./start.bash)，让运控板进入 Direct mode


确保无其他ros2节点在运行，然后启动硬件运控服务：
```bash
ros2 launch rl_controller hw.launch.py robot:=d1
```

## 交互控制
需编译下述功能包来启用键盘控制与遥控（ELRS）交互模块
```bash
colcon build --symlink-install --packages-up-to teleop_command keyboard_controller 
source install/setup.bash
```

- 键盘控制：
```bash
ros2 run keyboard_controller keyboard_controller_node
```

- 遥控（ELRS）：
```bash
ros2 launch teleop_command teleop_command.launch.py
```

## 控制器配置与模型
- 控制器参数与模型位于：
  - `controller/rl_controller/config/<robot>/controllers.yaml`
  - ONNX 模型示例：
    - `controller/rl_controller/config/tita/stand.onnx`
    - `controller/rl_controller/config/d1/flat.onnx`、`stairs.onnx`
- 更新控制策略时，修改对应 `controllers.yaml` 与 ONNX 文件路径
- 状态机接口与实现：`controller/rl_controller/include/rl_controller/fsm/*` 与 `src/fsm/*`


## 诊断与常见问题
- Webots未找到：  的安装路径要在环境变量中，例如：
```bash
export WEBOTS_HOME=/usr/lib/webots
```
- Mujoco 未找到：确认 `mujoco` 已安装并设置 `MUJOCO_DIR`
- 控制器未加载：检查 `controller_manager` 日志与 `controllers.yaml` 配置。  
- 模型描述加载失败：确认 `robot:=<name>` 与对应 `*_description` 包存在且可用
- 如果在TITA上遇到如下编译问题，可以尝试把代码中黄框部分去掉
![bug1](/docker/bug1.png)

```
#include "rl_controller/rl_controller_parameters.hpp"
改成#include "rl_controller_parameters.hpp"
```

## 许可证
各子包的许可证可能不同，请参考对应 `package.xml` 中的 `license` 字段。