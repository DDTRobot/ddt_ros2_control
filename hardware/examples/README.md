# canfd_api

基于 CAN FD 协议的双足/四足机器人关节控制与传感器读取工具集。

## 环境要求

- OS: Ubuntu 22.04
- ROS2: Humble
- 依赖库: `canfd_api`（位于 `/opt/d1_ros2/lib`，头文件在 `/opt/d1_ros2/include`）
- 编译器: GCC，C++17

## 前置条件

使用该接口前需要先关掉前机和后机原有的控制器：

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

---

## API 接口说明

### 数据结构

```cpp
struct api_motor_in_t {
  uint64_t timestamp;  // 时间戳 (µs)
  float    position;   // 关节位置 (rad)
  float    velocity;   // 关节速度 (rad/s)
  float    torque;     // 关节力矩 (N·m)
};

struct api_motor_out_t {
  uint64_t timestamp;  // 时间戳 (µs)
  float    position;   // 目标位置 (rad)
  float    kp;         // 位置比例增益
  float    velocity;   // 目标速度 (rad/s)
  float    kd;         // 速度微分增益
  float    torque;     // 前馈力矩 (N·m)
};

struct api_imu_data_t {
  uint64_t timestamp;    // 时间戳 (µs)
  float    accl[3];      // 线加速度 [x, y, z] (m/s²)
  float    gyro[3];      // 角速度   [x, y, z] (rad/s)
  float    quaternion[4];// 姿态四元数，顺序为 x y z w
  float    temperature;  // IMU 温度 (°C)
};

struct api_battery_info_t {
  uint64_t timestamp;    // 时间戳 (µs)
  float    voltage;      // 电压 (V)
  float    current;      // 电流 (A)
  float    temperature1; // 电芯温度 1 (°C)
  float    temperature2; // 电芯温度 2 (°C)
  float    temperature3; // 电芯温度 3 (°C)
  float    temperature_mos; // MOS 管温度 (°C)
  float    percentage;   // 剩余电量百分比 (%)
  float    vcells[12];   // 各节电芯电压 (V)
  uint16_t max_cap;      // 最大容量 (mAh)
  uint16_t remain_cap;   // 剩余容量 (mAh)
  uint16_t cycle_count;  // 充放电循环次数
  uint16_t pack_status;  // 电池包状态码
  uint16_t bat_status;   // 电池状态码
  uint16_t pack_config;  // 电池包配置
  uint32_t app_version;  // 固件版本号
};

struct dock_status_t {
  uint64_t timestamp;  // 时间戳 (µs)
  bool     slave;      // 从机对接状态：true = 已对接
  bool     connect;    // 总线连接状态：true = 已连接
};

struct api_channel_input_t {
  uint32_t timestamp;      // 时间戳 (µs)
  float    forward;        // 前进速度指令 (m/s)
  float    yaw;            // 偏航角速度指令 (rad/s)
  float    pitch;          // 俯仰角指令 (rad)
  float    roll;           // 横滚角指令 (rad)
  float    height;         // 站立高度指令 (m)
  float    split;          // 劈叉角度指令 (rad)
  float    tilt;           // 侧倾角指令 (rad)
  float    forward_accel;  // 前进加速度指令 (m/s²)
  float    yaw_accel;      // 偏航角加速度指令 (rad/s²)
};

struct api_rpc_response_t {
  uint32_t timestamp;  // 时间戳 (µs)
  uint16_t key;        // RPC 命令键，见 api_rpc_key_t
  uint32_t value;      // 命令参数值
};
```

---

### 构造 / 析构

```cpp
CanfdApi(size_t motor_size = 8, size_t battery_size = 2,
         std::string can_interface = "can0", std::string can_name = "robot_can0");
/**
 * @brief Construct a CanfdApi instance and start CAN communication.
 * @param motor_size     Total number of motors. Set to 0 if motor data is not needed.
 * @param battery_size   Number of battery packs. Set to 0 if battery data is not needed.
 * @param can_interface  SocketCAN interface name (e.g. "can0").
 * @param can_name       Logical name used internally to identify this CAN bus.
 */

~CanfdApi();
/**
 * @brief Destroy the CanfdApi instance and release all CAN resources.
 */
```

**典型初始化示例：**

```cpp
can_device::CanfdApi robot(8, 2, "can0");  // 8 个电机 + 2 块电池（完整功能）
can_device::CanfdApi robot(0, 2, "can0");  // 仅读电池
can_device::CanfdApi robot(0, 0, "can0");  // 仅读对接状态
```

---

### 读取接口

```cpp
const std::vector<api_motor_in_t> & get_motors_in() const;
/**
 * @brief Get the latest feedback data of all motors.
 * @return const std::vector<api_motor_in_t> &: motor states (position / velocity / torque),
 *         vector size equals motor_size passed to the constructor.
 */

const std::vector<uint16_t> & get_motors_status() const;
/**
 * @brief Get the error status codes of all motors.
 * @note  A value of 0 means normal; any non-zero value indicates a fault.
 * @return const std::vector<uint16_t> &: status codes, size equals motor_size.
 */

const api_imu_data_t & get_imu_data() const;
/**
 * @brief Get the latest IMU data from the MCU.
 * @note  Quaternion order is x y z w.
 * @return const api_imu_data_t &: IMU data including quaternion, linear acceleration,
 *         angular velocity, and temperature.
 */

const std::vector<api_battery_info_t> & get_batteries_info() const;
/**
 * @brief Get the battery pack information for all batteries.
 * @return const std::vector<api_battery_info_t> &: battery states,
 *         vector size equals battery_size passed to the constructor.
 */

const dock_status_t & get_dock_status() const;
/**
 * @brief Get the current docking / connection status.
 * @return const dock_status_t &: docking status with slave and connect flags.
 */
```

---

### 控制接口

```cpp
bool send_motors_can(std::vector<api_motor_out_t> motors);
/**
 * @brief Send motor commands to all motors at once.
 * @note  motors must have exactly motor_size elements.
 * @param motors  Per-motor command structs (position, velocity, torque, kp, kd).
 * @return true if the CAN frame was sent successfully, false otherwise.
 */

bool send_leg_motors_can(std::vector<api_motor_out_t> motors, size_t leg_id);
/**
 * @brief Send motor commands to a single leg.
 * @note  motors size must equal leg_dof (4 for a biped robot).
 * @param motors  Command structs for each joint of the leg.
 * @param leg_id  Leg index starting from 0 (biped: 0 = left leg, 1 = right leg).
 * @return true if the CAN frame was sent successfully, false otherwise.
 */

bool send_command_can_channel_input(api_channel_input_t data);
/**
 * @brief Send a locomotion channel input command (velocity / posture setpoints).
 * @param data  Channel input struct: forward, yaw, pitch, roll, height, split, tilt, etc.
 * @return true if the CAN frame was sent successfully, false otherwise.
 */

bool send_command_can_rpc_request(api_rpc_response_t data);
/**
 * @brief Send an RPC request to the locomotion controller.
 * @note  Supported keys are defined in api_rpc_key_t, e.g. SET_READY_NEXT, SET_STAND_MODE,
 *        SET_JUMP, SET_MOTOR_ZERO.
 * @param data  RPC struct with command key (api_rpc_key_t) and value.
 * @return true if the CAN frame was sent successfully, false otherwise.
 */

bool send_can_message(struct canfd_frame frame);
/**
 * @brief Send a raw CAN FD frame directly on the bus.
 * @param frame  A fully populated canfd_frame struct.
 * @return true if the frame was sent successfully, false otherwise.
 */

bool unlock_dock();
/**
 * @brief Send the unlock command to release the docking mechanism.
 * @note  Sends CAN ID 0x89 with data byte 0x01.
 * @return true if the CAN frame was sent successfully, false otherwise.
 */
```

**控制模式说明（`send_leg_motors_can` / `send_motors_can`）：**

| 模式 | kp | kd | 说明 |
|------|----|----|------|
| 纯力矩 | `0` | `0` | 只输出 `torque` 前馈力矩 |
| 阻抗控制 | `>0` | `>0` | 位置 + 速度 PD 叠加前馈力矩 |
| 纯位置 | `>0` | `0` | 仅位置比例控制 |

**发送示例：**

```cpp
size_t leg_dof = 4;
std::vector<can_device::api_motor_out_t> motors(leg_dof);
for (size_t id = 0; id < leg_dof; id++) {
    motors[id].kp       = 0.0f;
    motors[id].kd       = 0.0f;
    motors[id].position = 0.0f;
    motors[id].velocity = 0.0f;
    motors[id].torque   = 0.5f;  // 0.5 N·m 前馈力矩
}
robot.send_leg_motors_can(motors, 0);  // 左腿
robot.send_leg_motors_can(motors, 1);  // 右腿
```

---

### 枚举说明

```cpp
enum api_rpc_key_t {
  RPC_UNDEFINED    = 0x000,  // 未定义
  GET_MODEL_INFO   = 0x100,  // 获取型号信息
  GET_SERIAL_NUMBER= 0x101,  // 获取序列号
  SET_READY_NEXT   = 0x200,  // 设置下一步就绪模式，value 见 ready_next_t
  SET_BOARDCAST    = 0x201,  // 设置广播
  SET_INPUT_MODE   = 0x221,  // 设置输入模式
  SET_STAND_MODE   = 0x222,  // 设置站立模式
  SET_HEAD_MODE    = 0x223,  // 设置头部模式
  SET_JUMP         = 0x231,  // 触发跳跃
  SET_MOTOR_ZERO   = 0x280,  // 电机零位标定
};

enum ready_next_t {
  READY_WAITING      = 0x00,  // 等待就绪
  AUTO_LOCOMOTION    = 0x01,  // 自动运动模式
  FORCE_LOCOMOTION   = 0x02,  // 强制运动模式
  FORCE_DIRECT       = 0x03,  // 强制直接控制
  MOTOR_ZERO_CAL     = 0x04,  // 进入电机零位标定
  BOOT_RECOVERY_MODE = 0x05,  // 启动恢复模式
  ESTOP_MODE         = 0x06,  // 急停模式
};
```

---

## 硬件说明

- 双足机器人：2 条腿，每条腿 4 个关节，共 8 个电机
- 构造参数：`CanfdApi(motor_size, battery_size, can_interface)`

## 安全退出

`write_joints` 捕获 `SIGINT`（Ctrl+C）和 `SIGTERM`（kill）信号，退出前自动向所有电机发送零力矩指令，防止电机保持通电状态。

自行开发时建议同样注册信号处理函数：

```cpp
signal(SIGINT, signal_handler);
signal(SIGTERM, signal_handler);
// 在 signal_handler 中对所有腿调用 send_leg_motors_can，torque=0.0f
```


