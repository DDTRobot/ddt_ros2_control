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

#include "tita_robot/tita_robot.hpp"

tita_robot robot(8, 2, "can0");

void test_read()
{
  while (1) {
    std::cout << "=================================" << std::endl;
    auto q = robot.get_joint_q();
    auto v = robot.get_joint_v();
    auto t = robot.get_joint_t();
    auto status = robot.get_joint_status();
    auto quat = robot.get_imu_quaternion();
    auto accl = robot.get_imu_acceleration();
    auto gyro = robot.get_imu_angular_velocity();
    for (size_t i = 0; i < q.size(); i++) {
      std::cout << "q[" << i << "] = " << q[i] << "\tv[" << i << "] = " << v[i] << "\tt[" << i
                << "] = " << t[i] << std::endl;
    }
    for (size_t i = 0; i < status.size(); i++) {
      std::cout << "status[" << i << "] = " << status[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "quat = " << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3]
              << std::endl;
    std::cout << "accl = " << accl[0] << " " << accl[1] << " " << accl[2] << std::endl;
    std::cout << "gyro = " << gyro[0] << " " << gyro[1] << " " << gyro[2] << std::endl;
    sleep(1);
  }
}

void test_write()
{
  while (1) {
    std::cout << "=================================" << std::endl;
    std::vector<double> t = {0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5};
    robot.set_target_joint_t(t);
    sleep(1);
  }
}

int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;
  test_read();
  // test_write();

  return 0;
}
