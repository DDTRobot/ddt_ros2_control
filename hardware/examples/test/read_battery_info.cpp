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
      std::cout << "Usage: read_battery_info [options]" << std::endl;
      std::cout << "Options:" << std::endl;
      std::cout << "  -c, --can-interface  CAN interface name (default: can0)" << std::endl;
      std::cout << "  -h, --help           Show this help message" << std::endl;
      return 0;
    }
  }
  can_device::CanfdApi robot(0, 2, can_interface);

  while (1) {
    std::cout << "=================================" << std::endl;
    auto info = robot.get_batteries_info();
    for (size_t i = 0; i < info.size(); i++) {
      std::cout << "Battery[" << i << "]" << std::endl;
      std::cout << "Time: " << info[i].timestamp << std::endl;
      std::cout << "Voltage: " << info[i].voltage << std::endl;
      std::cout << "Current: " << info[i].current << std::endl;
      std::cout << "Temperature: " << info[i].temperature1 << " " << info[i].temperature2 << " "
                << info[i].temperature3 << " " << info[i].temperature_mos << std::endl;
      std::cout << "Percentage: " << info[i].percentage << std::endl;
      std::cout << "Max Cap: " << info[i].max_cap << std::endl;
      std::cout << std::endl;
    }
    sleep(1);
  }
  return 0;
}
