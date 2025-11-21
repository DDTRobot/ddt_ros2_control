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

#include "webots_bridge/webots_bridge_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "webots/device.h"
#include "webots/robot.h"
namespace tita_webots_ros2_control
{
WebotsBridge::WebotsBridge() { mNode = NULL; }
void WebotsBridge::init(
  webots_ros2_driver::WebotsNode * node, const hardware_interface::HardwareInfo & info)
{
  mNode = node;
  for (hardware_interface::ComponentInfo component : info.joints) {
    Joint joint;
    joint.name = component.name;

    WbDeviceTag device = wb_robot_get_device(joint.name.c_str());
    WbNodeType type = wb_device_get_node_type(device);
    joint.motor = (type == WB_NODE_LINEAR_MOTOR || type == WB_NODE_ROTATIONAL_MOTOR)
                    ? device
                    : wb_position_sensor_get_motor(device);
    device = (component.parameters.count("sensor") == 0)
               ? wb_robot_get_device(joint.name.c_str())
               : wb_robot_get_device(component.parameters.at("sensor").c_str());
    type = wb_device_get_node_type(device);
    joint.sensor =
      (type == WB_NODE_POSITION_SENSOR) ? device : wb_motor_get_position_sensor(device);

    if (joint.sensor) {
      wb_position_sensor_enable(joint.sensor, wb_robot_get_basic_time_step());
      wb_motor_enable_torque_feedback(joint.motor, wb_robot_get_basic_time_step());
    }
    if (!joint.sensor && !joint.motor) {
      throw std::runtime_error("Cannot find a Motor or PositionSensor with name " + joint.name);
    }
    // Check if state interfaces have initial positions
    for (hardware_interface::InterfaceInfo stateInterface : component.state_interfaces) {
      if (stateInterface.name == "position" && !stateInterface.initial_value.empty()) {
        joint.position = std::stod(stateInterface.initial_value);
        wb_motor_set_position(joint.motor, std::stod(stateInterface.initial_value));
      }
    }

    mJoints.push_back(joint);
  }
  wb_robot_step(wb_robot_get_basic_time_step());

  for (hardware_interface::ComponentInfo component : info.sensors) {
    std::string sensor_name = component.name;
    WbDeviceTag device = wb_robot_get_device(std::string(sensor_name + " inertial").c_str());
    WbNodeType type = wb_device_get_node_type(device);
    if (type == WB_NODE_INERTIAL_UNIT) {
      mImu.name = sensor_name;
      mImu.inertialUnit = device;
      mImu.gyro = wb_robot_get_device(std::string(sensor_name + " gyro").c_str());
      mImu.accelerometer =
        wb_robot_get_device(std::string(sensor_name + " accelerometer").c_str());  // defalt name
      wb_inertial_unit_enable(mImu.inertialUnit, wb_robot_get_basic_time_step());
      if (mImu.gyro) wb_gyro_enable(mImu.gyro, wb_robot_get_basic_time_step());
      if (mImu.accelerometer)
        wb_accelerometer_enable(mImu.accelerometer, wb_robot_get_basic_time_step());
    }
    // std::cout << "\033[35m" << sensor_name << type << "\033[0m" << std::endl;
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WebotsBridge::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WebotsBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (Joint & joint : mJoints) {
    if (joint.sensor) {
      interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));
      interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocity)));
      interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effort)));
    }
  }

  for (hardware_interface::ComponentInfo component : info_.sensors) {
    if (component.name == mImu.name) {
      for (uint i = 0; i < 4; i++) {
        interfaces.emplace_back(hardware_interface::StateInterface(
          component.name, component.state_interfaces[i].name, &mImu.orientation[i]));
      }
      for (uint i = 0; i < 3; i++) {
        interfaces.emplace_back(hardware_interface::StateInterface(
          component.name, component.state_interfaces[i + 4].name, &mImu.angular_velocity[i]));
      }
      for (uint i = 0; i < 3; i++) {
        interfaces.emplace_back(hardware_interface::StateInterface(
          component.name, component.state_interfaces[i + 7].name, &mImu.linear_acceleration[i]));
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> WebotsBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (Joint & joint : mJoints) {
    if (joint.motor) {
      interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));

      interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));

      interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));

      interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kp", &(joint.kp)));

      interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kd", &(joint.kd)));
    }
  }
  return interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WebotsBridge::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WebotsBridge::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WebotsBridge::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  static double lastReadTime = 0;

  const double deltaTime = wb_robot_get_time() - lastReadTime;
  lastReadTime = wb_robot_get_time();

  for (Joint & joint : mJoints) {
    if (joint.sensor) {
      const double position = wb_position_sensor_get_value(joint.sensor);
      const double velocity =
        std::isnan(joint.position) ? NAN : (position - joint.position) / deltaTime;

      if (!std::isnan(joint.velocity)) {
        joint.acceleration = (joint.velocity - velocity) / deltaTime;
      }
      joint.velocity = velocity;
      joint.position = position;
      auto effort = joint.kp * (joint.positionCommand - joint.position) +
                    joint.kd * (joint.velocityCommand - joint.velocity) + joint.effortCommand;
      joint.effort = wb_motor_get_torque_feedback(joint.motor) + effort;
    }
  }

  if (mImu.accelerometer != 0) {
    const double * values = wb_accelerometer_get_values(mImu.accelerometer);
    mImu.linear_acceleration[0] = values[0];
    mImu.linear_acceleration[1] = values[1];
    mImu.linear_acceleration[2] = values[2];
  }

  if (mImu.gyro != 0) {
    const double * values = wb_gyro_get_values(mImu.gyro);
    mImu.angular_velocity[0] = values[0];
    mImu.angular_velocity[1] = values[1];
    mImu.angular_velocity[2] = values[2];
  }

  if (mImu.inertialUnit != 0) {
    const double * values = wb_inertial_unit_get_quaternion(mImu.inertialUnit);
    mImu.orientation[0] = values[0];
    mImu.orientation[1] = values[1];
    mImu.orientation[2] = values[2];
    mImu.orientation[3] = values[3];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WebotsBridge::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (Joint & joint : mJoints) {
    if (joint.motor) {
      auto effort = joint.kp * (joint.positionCommand - joint.position) +
                    joint.kd * (joint.velocityCommand - joint.velocity) + joint.effortCommand;
      // joint.effortCommand = effort;
      wb_motor_set_torque(joint.motor, effort);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace tita_webots_ros2_control

PLUGINLIB_EXPORT_CLASS(
  tita_webots_ros2_control::WebotsBridge, webots_ros2_control::Ros2ControlSystemInterface)
