# canfd_api

基于 CAN FD 协议的双足/四足机器人关节控制与传感器读取工具集。

## 环境要求

- OS: Ubuntu 22.04
- ROS2: Humble
- 依赖库: `canfd_api`（位于 `/opt/d1_ros2/lib`，头文件在 `/opt/d1_ros2/include`）
- 编译器: GCC，C++17
## 前置条件
使用该接口前需要前机和后机原有的控制器关掉
```bash
systemctl stop d1_bringup
```
## 编译

```bash
mkdir -p build && cd build
cmake ..
make
```

## 可执行程序

所有程序均通过 `-c` / `--can-interface` 指定 CAN 接口（默认 `can0`）。

| 程序 | 用途 |
|------|------|
| `read_joints_imu_status` | 读取 8 个关节的位置 / 速度 / 力矩，以及 IMU 四元数、加速度、角速度 |
| `read_battery_info` | 读取电池电压、电流、温度、电量 |
| `read_dock_status` | 读取拼接连接状态 |
| `write_joints` | 向 8 个关节发送控制指令（纯力矩模式测试） |

### 运行示例

```bash
./read_joints_imu_status -c can0
./read_battery_info -c can0
./read_dock_status -c can0
./write_joints -c can0
```

## 硬件说明

- 双足机器人：2 条腿，每条腿 4 个关节，共 8 个电机
- 构造接口：`CanfdApi(num_motors, num_legs, can_interface)`

## 控制接口

`write_joints` 使用 `api_motor_out_t` 结构体发送指令：

| 字段 | 说明 |
|------|------|
| `kp` | 位置比例增益 |
| `kd` | 速度微分增益 |
| `position` | 目标位置 (rad) |
| `velocity` | 目标速度 (rad/s) |
| `torque` | 前馈力矩 (N·m) |

## 安全退出

`write_joints` 捕获 `SIGINT`（Ctrl+C）和 `SIGTERM`（kill）信号，退出前自动向所有电机发送零力矩指令，防止电机保持通电状态。

## 安装

```bash
make install  # 安装到 lib/canfd_api/
```

