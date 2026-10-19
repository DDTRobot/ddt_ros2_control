# DDT 机器人 sim2sim/sim2real
本仓库提供了 DDT 机器人 sim2sim 和部署，遥控的工具，通过 15 个 ROS 2 的包提供
| 分类 | 数量 | 包名 |
|---|---:|---|
| `controller/` | 1 | `rl_controller` 基于`ros2_control`的强化学习控制器 |
| `interaction/` | 5 | `gazebo_bridge`、`webots_bridge`、`mujoco_bridge`、`mujoco_ros2_control`、`mujoco_sim_ros2` |
| `interaction` | 2 | `keyboard_controller` 键盘交互、`teleop_command` 遥控手柄交互 |
| `ros_utils`/ | 1 | `ros_utils` 提供`ros`话题 |
| `hardware/` | 2 | `hardware_bridge` 硬件桥接节点与启动文件、`tita_robot` 机器人硬件驱动 |
| `urdfs/` | 4 | `d1_description`、`d1h_description`、`tita_description`、`titatit_description` 提供 `URDF`/`XACRO`/`Mujoco` 机器人模型描述 |

使用这些 ROS 2 包可快速进行 sim2sim 和控制策略真机部署。

## 环境依赖

- 系统：Ubuntu 22.04 可以使用我们提供的 [Docker 镜像](./docker/README.md) 。
- 安装  ros2 humble 并安装
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```
- 按需安装仿真环境
  1. `webots`
  ```bash
  sudo apt install ros-humble-webots-ros2 ros-humble-webots-ros2-control
  ```
  1. `gazebo`
  ```bash
  sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
  ```
  1. `mujoco`: 在工作空间中
  ```bash
  git clone -b 3.3.0 https://github.com/google-deepmind/mujoco.git
  ```

- 要使用 RL 控制器，则安装onnx推理引擎
``` bash 
 # 根据你的系统架构选择x64或者aarch64
wget https://github.com/microsoft/onnxruntime/releases/download/v1.10.0/onnxruntime-linux-x64-1.10.0.tgz
tar xvf onnxruntime-linux-x64-1.10.0.tgz
sudo cp -a onnxruntime-linux-x64-1.10.0/include/* /usr/include
sudo cp -a onnxruntime-linux-x64-1.10.0/lib/* /usr/lib
```

## 构建
按需编译组件
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
```


## 仿真运行
> 在运行所有 `ros2` 命令前都需要载入环境
> ```bash
> # 在工作空间路径下
> source install/setup.bash
> ```

开启仿真环境后，可进行[交互控制](#交互控制)
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

## 交互控制
用于键盘控制与遥控（ELRS）交互。

编译下述功能包
```bash
colcon build --symlink-install --packages-up-to teleop_command keyboard_controller 
```

- 键盘控制：
```bash
source install/setup.bash
ros2 run keyboard_controller keyboard_controller_node
```
先按 `7` 进入 `transform_up` 等待 2 秒左右，让机器人站起，然后按 `0` 切换到我们提供的 RL 控制器。

- 遥控（ELRS）：
```bash
source install/setup.bash
ros2 launch teleop_command teleop_command.launch.py
```


## 实机部署

所需文件在 `hardware/` 文件夹下。 

### D1

安装依赖
```bash
sudo apt install python3-colcon-common-extensions
```
需要将代码拷贝在机器上，并编译
```bash
colcon build --symlink-install --packages-up-to rl_controller hardware_bridge
```
**注意事项**
在使用 `hardware/` 之前，请检查当前 `d1-ros2` 软件版本
```
dpkg -l d1-ros2
```
如果当前软件版本为4月01号，则使用`compress_v1`分支代码，切忌使用`main`分支，否则**失控**，务必**注意安全**。

以下`compress_v1`分支使用方法：
```bash
git clone https://github.com/DDTRobot/ddt_ros2_control/tree/compress_v1
cd ddt_ros2_control
#在编译hardware_bridge前，source /opt/d1-ros2/setup.bash
source /opt/d1_ros2/setup.bash 
#确保前后机无其他ros2节点在运行，然后启动硬件运控服务
colcon build --symlink-install --packages-up-to rl_controller hardware_bridge
sudo systemctl stop d1_bringup.service
ros2 launch rl_controller hw.launch.py robot:=d1
```
- 启动控制器（硬件环境）：
在启动有硬件环境的机器上，需要手动关闭已经启动的运控服务：
```bash
sudo systemctl stop d1_bringup.service
```

确保无其他ros2节点在运行，然后启动硬件运控服务：
```bash
ros2 launch rl_controller hw.launch.py robot:=d1
```

### TITA

TITA 的实机部署指引见 `hardware/tita_robot/docs/`。


## 控制器配置与模型
- 控制器参数与模型位于：
  - `controller/rl_controller/config/<robot>/controllers.yaml`
  - ONNX 模型示例：
    - `controller/rl_controller/config/tita/stand.onnx`
    - `controller/rl_controller/config/d1/flat.onnx`、`stairs.onnx`
- 更新控制策略时，修改对应 `controllers.yaml` 与 ONNX 文件路径
- 状态机接口与实现：`controller/rl_controller/include/rl_controller/fsm/*` 与 `src/fsm/*`


## 常见问题
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
