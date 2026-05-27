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

#include <time.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "canfd_api/canfd_api.hpp"

int main(int argc, char * argv[])
{
  std::string can_interface = "can0";

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-c" || arg == "--can-interface") {
      if (i + 1 < argc) {
        can_interface = argv[++i];
      }
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: read_joint_imu_status [options]" << std::endl;
      std::cout << "Options:" << std::endl;
      std::cout << "  -c, --can-interface  CAN interface name (default: can0)" << std::endl;
      std::cout << "  -h, --help           Show this help message" << std::endl;
      return 0;
    }
  }
  can_device::CanfdApi robot(8, 2, can_interface);

  while (1) {
    std::cout << "=================================" << std::endl;
    auto motors_in = robot.get_motors_in();
    auto imu = robot.get_imu_data();
    for (size_t i = 0; i < motors_in.size(); i++) {
      std::cout << "motor[" << i << "] = " << motors_in[i].position << " " << motors_in[i].velocity
                << " " << motors_in[i].torque << std::endl;
    }

    std::cout << std::endl;
    std::cout << "quat = " << imu.quaternion[0] << " " << imu.quaternion[1] << " "
              << imu.quaternion[2] << " " << imu.quaternion[3] << std::endl;
    std::cout << "accl = " << imu.accl[0] << " " << imu.accl[1] << " " << imu.accl[2] << std::endl;
    std::cout << "gyro = " << imu.gyro[0] << " " << imu.gyro[1] << " " << imu.gyro[2] << std::endl;
    sleep(1);
  }

  return 0;
}
