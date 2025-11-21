// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TITA_ROBOT__CANFD_API_HPP_
#define TITA_ROBOT__CANFD_API_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "protocol/can_utils.hpp"

namespace can_device
{
#define PACKED __attribute__((__packed__))
#define GRAVITY 9.81f

// CAN ID 定义
#define CAN_ID_BATTERY1 0x081
// #define CAN_ID_BATTERY2 0x082
#define CAN_ID_MOTOR_ID_OFFSET 0x385
#define CAN_ID_STATE 0x102
#define CAN_ID_STATE2 0x104

#define CAN_ID_LOCOMOTION_STATE 0x103
#define CAN_ID_IMU1 0x118
// #define CAN_ID_IMU2 0x119
#define CAN_ID_MOTOR_IN 0x108
#define CAN_ID_SEND_MOTORS 0x120
#define CAN_ID_CHANNEL_INPUT 0x12D
#define CAN_ID_RPC_REQUEST 0x170

struct api_battery_info_t
{
  uint16_t vcell0;
  uint16_t null;
  uint32_t raw_voltage;
  int32_t current_cadc;
  uint16_t temperature1;
  uint16_t temperature2;
  uint16_t temperature3;
  uint16_t temperature_mos;
  uint16_t max_cap;
  uint16_t remain_cap;
  uint16_t remain_soc;
  uint16_t cycle_count;
  uint16_t pack_status;
  uint16_t bat_status;
  uint16_t pack_config;
  uint32_t app_version;
  uint32_t build_timestamp;
  uint16_t vcells[11];
} PACKED;

struct api_motor_in_t
{
  uint32_t timestamp;
  float position;
  float velocity;
  float torque;
} PACKED;

struct api_imu_data_t
{
  uint32_t timestamp;
  float accl[3];
  float gyro[3];
  float quaternion[4];  // x y z w
  float temperature;
} PACKED;

struct api_motor_status_t
{
  uint32_t timestamp;
  uint16_t key;
  uint8_t value[8];
} PACKED;

struct api_locomotion_status_t
{
  uint32_t timestamp;
  uint16_t key;
  union {
    uint8_t u8;
    uint8_t u8x4[4];
    uint16_t u16;
    uint16_t u16x4[4];
    uint32_t u32;
    uint32_t u32x4[4];
    float f32;
    float f32x4[4];
    float f32x9[9];
  } value;
} PACKED;

// 枚举定义
enum locomotion_status_key_t {
  ROBOT_MODE = 0x1001,
  JUMP_STATUS = 0x1002,
};

enum robot_status_t {
  DIE = 0x00,
  INIT = 0x01,
  TRANSFORM_UP = 0x02,
  STAND = 0x03,
  TRANSFORM_DOWN = 0x04,
  CRASH = 0x05,
  SUSPENDING = 0x06,
  JUMP = 0x07,
};

enum jump_status_t {
  LIFT = 0x00,
  RETRACT = 0x01,
  EXTEND = 0x02,
  FINISH = 0x03,
  DISCHARGE = 0x04,
  CHARGE = 0x05,
  OFF = 0x06,
};

enum api_rpc_key_t {
  RPC_UNDEFINED = 0x000,
  GET_MODEL_INFO = 0x100,
  GET_SERIAL_NUMBER = 0x101,
  SET_READY_NEXT = 0x200,
  SET_BOARDCAST = 0x201,
  SET_INPUT_MODE = 0x221,
  SET_STAND_MODE = 0x222,
  SET_HEAD_MODE = 0x223,
  SET_JUMP = 0x231,
  SET_MOTOR_ZERO = 0x280,
};

enum ready_next_t {
  READY_WAITING = 0x00U,
  AUTO_LOCOMOTION = 0x01U,
  FORCE_LOCOMOTION = 0x02U,
  FORCE_DIRECT = 0x03U,
  MOTOR_ZERO_CAL = 0x04U,
  BOOT_RECOVERY_MODE = 0x05U,
  ESTOP_MODE = 0x06U,
};

struct api_channel_input_t
{
  uint32_t timestamp;
  float forward;
  float yaw;
  float pitch;
  float roll;
  float height;
  float split;
  float tilt;
  float forward_accel;
  float yaw_accel;
} PACKED;

struct api_rpc_response_t
{
  uint32_t timestamp;
  uint16_t key;
  uint32_t value;
} PACKED;

struct api_motor_out_t
{
  uint32_t timestamp;
  float position;
  float kp;
  float velocity;
  float kd;
  float torque;
} PACKED;

// 获取当前时间
inline uint32_t get_current_time()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000000 + tv.tv_usec;
}

// 主类定义
class CanfdApi
{
public:
  // 构造函数和析构函数
  CanfdApi(
    size_t motor_size = 8, size_t battery_size = 2, std::string can_interface = "can0",
    std::string can_name = "robot_can0");
  ~CanfdApi();

  // 数据获取方法
  const std::vector<api_battery_info_t> * get_batteries_info() const { return &battery_infos_; }
  const std::vector<api_motor_in_t> * get_motors_in() const { return &motors_in_; }
  const api_imu_data_t * get_imu_data() const { return &imu_data_; }
  const std::vector<uint16_t> * get_motors_status() const { return &motors_status_; }
  const robot_status_t * get_robot_status() const { return &robot_status_; }
  const jump_status_t * get_jump_status() const { return &jump_status_; }

  // CAN发送方法
  bool send_motors_can(std::vector<api_motor_out_t> motors);
  bool send_command_can_channel_input(api_channel_input_t data);
  bool send_command_can_rpc_request(api_rpc_response_t data);

  // 超时检查
  bool is_imu_timeout() const { return imu_timeout_.load(); }
  bool is_motors_timeout() const { return motors_timeout_.load(); }

private:
  // 常量定义
#define MIN_TIME_OUT_US 1'000L      // 1ms
#define MAX_TIME_OUT_US 3'000'000L  // 3s

  // CAN配置参数
  // std::string can_interface = "vcan0";
  // std::string can_name = "robot_can0";
  bool can_extended_frame = false;
  bool can_rx_is_block = false;
  int64_t timeout_us = MAX_TIME_OUT_US;
  uint8_t can_id_offset = 0x00U;

  // CAN设备和回调
  can_device::socket_can::can_fd_callback proprio_can_receive_callback_,
    batteries_can_receive_callback_, motors_id_offset_callback_;
  std::shared_ptr<can_device::socket_can::CanDev> proprio_can_receive_api_,
    batteries_can_receive_api_, motors_id_offset_api_;
  std::shared_ptr<can_device::socket_can::CanDev> can_send_api_;

  // 注册过滤器
  void register_motors_device_can_filter();

  // 数据回调函数
  void motors_id_offset_callback(std::shared_ptr<struct canfd_frame> recv_frame);
  void batteries_data_callback(std::shared_ptr<struct canfd_frame> recv_frame);
  void proprio_data_callback(std::shared_ptr<struct canfd_frame> recv_frame);
  void motors_data_callback(std::shared_ptr<struct canfd_frame> recv_frame);
  void imu_data_callback(std::shared_ptr<struct canfd_frame> recv_frame);
  void motors_status_callback(std::shared_ptr<struct canfd_frame> recv_frame);
  void locomotion_status_callback(std::shared_ptr<struct canfd_frame> recv_frame);

  // 数据存储
  std::vector<api_battery_info_t> battery_infos_;
  std::vector<api_motor_in_t> motors_in_;
  api_imu_data_t imu_data_;
  std::vector<uint16_t> motors_status_;
  robot_status_t robot_status_;
  jump_status_t jump_status_;
  bool motors_index_offset_{false};

  // 线程安全锁
  mutable std::shared_mutex motors_in_mutex_, imu_mutex_, battery_mutex_;

  // 设备参数
  size_t leg_dof_{4}, leg_num_{2}, battery_num_{2};

  // 超时检测
  std::atomic<bool> imu_timeout_{false};
  std::atomic<bool> motors_timeout_{false};
  std::atomic<std::chrono::steady_clock::time_point> last_imu_activity_time_;
  std::vector<std::chrono::steady_clock::time_point> last_legs_activity_time_;
  std::vector<std::chrono::steady_clock::time_point> last_bats_activity_time_;
  
  mutable std::mutex last_legs_activity_mutex_, last_bats_activity_mutex_;
  std::thread timer_thread_;
  void check_timeout();
};
}  // namespace can_device

#endif  // TITA_ROBOT__CANFD_API_HPP_
