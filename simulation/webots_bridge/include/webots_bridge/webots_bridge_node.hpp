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

#ifndef WEBOTS_BRIDGE__WEBOTS_BRIDGE_NODE_HPP_
#define WEBOTS_BRIDGE__WEBOTS_BRIDGE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "webots/accelerometer.h"
#include "webots/gyro.h"
#include "webots/inertial_unit.h"
#include "webots/motor.h"
#include "webots/position_sensor.h"
#include "webots_ros2_control/Ros2ControlSystemInterface.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace tita_webots_ros2_control
{
struct Joint
{
  double position = 0.0f;
  double velocity = 0.0f;
  double effort = 0.0f;
  double acceleration = 0.0f;

  double effortCommand = 0.0f;
  double positionCommand = 0.0f;
  double velocityCommand = 0.0f;
  double kp = 0.0f;
  double kd = 0.0f;

  std::string name;
  WbDeviceTag motor;
  WbDeviceTag sensor;
};

struct InertiaUnit
{
  std::string name;
  WbDeviceTag inertialUnit{0};
  WbDeviceTag gyro{0};
  WbDeviceTag accelerometer{0};
  double linear_acceleration[3];
  double angular_velocity[3];
  double orientation[4];  // x y z w
};

class WebotsBridge : public webots_ros2_control::Ros2ControlSystemInterface
{
public:
  WebotsBridge();
  void init(
    webots_ros2_driver::WebotsNode * node, const hardware_interface::HardwareInfo & info) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

private:
  webots_ros2_driver::WebotsNode * mNode;
  std::vector<Joint> mJoints;
  InertiaUnit mImu;
};
}  // namespace tita_webots_ros2_control

#endif  // WEBOTS_BRIDGE__WEBOTS_BRIDGE_NODE_HPP_
