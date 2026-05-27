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

#include <signal.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "canfd_api/canfd_api.hpp"

static volatile bool g_running =
    true; // 主循环运行标志，信号处理器置 false 后退出
static can_device::CanfdApi *g_robot = nullptr; // 供信号处理器访问的机器人实例

// Ctrl+C / SIGTERM 安全退出：先发零力矩再退出主循环
static void signal_handler(int) {
  g_running = false;
  if (g_robot) {
    size_t leg_dof = 4;
    std::vector<can_device::api_motor_out_t> motors(leg_dof);
    for (size_t id = 0; id < leg_dof; id++) {
      motors[id].kp = 0.0;
      motors[id].kd = 0.0;
      motors[id].position = 0.0;
      motors[id].velocity = 0.0;
      motors[id].torque = 0.0;
    }
    for (size_t leg_index = 0; leg_index < 2; leg_index++)
      g_robot->send_leg_motors_can(motors, leg_index);
  }
}

int main(int argc, char *argv[]) {
  std::string can_interface = "can0";

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-c" || arg == "--can-interface") {
      if (i + 1 < argc) {
        can_interface = argv[++i];
      }
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: read_joints_imu_status [options]" << std::endl;
      std::cout << "Options:" << std::endl;
      std::cout << "  -c, --can-interface  CAN interface name (default: can0)"
                << std::endl;
      std::cout << "  -h, --help           Show this help message" << std::endl;
      return 0;
    }
  }
  can_device::CanfdApi robot(8, 2, can_interface); // 8 个电机，2 条腿
  g_robot = &robot;
  signal(SIGINT, signal_handler);  // Ctrl+C
  signal(SIGTERM, signal_handler); // kill
  while (g_running) {
    std::cout << "=================================" << std::endl;
    size_t leg_dof = 4; // 每条腿 4 个自由度
    std::vector<can_device::api_motor_out_t> motors(leg_dof);
    for (size_t id = 0; id < leg_dof; id++) {
      motors[id].kp = 0.0;
      motors[id].kd = 0.0;
      motors[id].position = 0.0;
      motors[id].velocity = 0.0;
      motors[id].torque = 0.5; // 测试用恒定力矩 (N·m)
    }
    for (size_t leg_index = 0; leg_index < 2; leg_index++)
      robot.send_leg_motors_can(motors, leg_index);
    sleep(1);
  }
  return 0;
}
