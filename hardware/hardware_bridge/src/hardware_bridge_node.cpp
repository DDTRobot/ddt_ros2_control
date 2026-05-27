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

#include "hardware_bridge/hardware_bridge_node.hpp"

#include <pthread.h>
#include <sched.h>

#include "pluginlib/class_list_macros.hpp"

namespace tita_locomotion
{

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareBridge::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (hardware_interface::ComponentInfo component : info.joints) {
    Joint joint;
    joint.name = component.name;
    mJoints.push_back(joint);
  }
  for (hardware_interface::ComponentInfo component : info.sensors) {
    if (component.name.find("_imu") != std::string::npos) {
      mImu.name = component.name;
    }
  }
  canfd_api_ =
    std::make_unique<can_device::CanfdApi>(mJoints.size(), 0, "can0");  // depends on urdf
  if (mJoints.size() % 8 == 0)
    leg_dof_ = 4;
  else if (mJoints.size() % 6 == 0)
    leg_dof_ = 3;
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (Joint & joint : mJoints) {
    interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));
    interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocity)));
    interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effort)));
  }

  for (hardware_interface::ComponentInfo component : info_.sensors) {
    if (component.name == mImu.name) {
      for (uint i = 0; i < 4; i++) {
        interfaces.emplace_back(
          hardware_interface::StateInterface(
            component.name, component.state_interfaces[i].name, &mImu.orientation[i]));
      }
      for (uint i = 0; i < 3; i++) {
        interfaces.emplace_back(
          hardware_interface::StateInterface(
            component.name, component.state_interfaces[i + 4].name, &mImu.angular_velocity[i]));
      }
      for (uint i = 0; i < 3; i++) {
        interfaces.emplace_back(
          hardware_interface::StateInterface(
            component.name, component.state_interfaces[i + 7].name, &mImu.linear_acceleration[i]));
      }
    }
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (Joint & joint : mJoints) {
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));
    interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kp", &(joint.kp)));
    interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kd", &(joint.kd)));
  }
  return interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareBridge::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareBridge::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HardwareBridge::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto motors_in = canfd_api_->get_motors_in();
  auto status = canfd_api_->get_motors_status();
  // RCLCPP_INFO(rclcpp::get_logger("hardware_bridge"), "monotonictime: %lu, systemtime: %u", can_device::get_monotonic_time(), can_device::get_current_time());
  for (size_t id = 0; id < mJoints.size(); id++) {
    mJoints[id].errorId = status[id];
    mJoints[id].position = motors_in[id].position;
    mJoints[id].velocity = motors_in[id].velocity;
    mJoints[id].effort = motors_in[id].torque;
  }
  // for (size_t id = 0; id < mJoints.size(); id++) {
  //   if (mJoints[id].errorId != 0x00) {
  //     return hardware_interface::return_type::ERROR;
  //   }
  // }

  auto imu_data = canfd_api_->get_imu_data();

  if ((can_device::get_monotonic_time() - imu_data.timestamp) / 1.0e6 > 2 /*s*/) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("hardware_bridge"), clock_, 10000,
      "Imu data read timeout, imu time: %f s, current time: %f s", imu_data.timestamp / 1.0e6,
      can_device::get_monotonic_time() / 1.0e6);
    // return hardware_interface::return_type::ERROR;
  }
  for (size_t id = 0; id < 3; id++) {
    mImu.linear_acceleration[id] = imu_data.accl[id];
    mImu.angular_velocity[id] = imu_data.gyro[id];
    mImu.orientation[id] = imu_data.quaternion[id];
  }
  mImu.orientation[3] = imu_data.quaternion[3];
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HardwareBridge::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t leg_id = 0; leg_id < mJoints.size() / leg_dof_; leg_id++) {
    std::vector<can_device::api_motor_out_t> motors(leg_dof_);
    bool ok = true;
    for (size_t id = 0; id < leg_dof_; id++) {
      motors[id].kp = mJoints[id + leg_id * leg_dof_].kp;
      motors[id].kd = mJoints[id + leg_id * leg_dof_].kd;
      motors[id].position = mJoints[id + leg_id * leg_dof_].positionCommand;
      motors[id].velocity = mJoints[id + leg_id * leg_dof_].velocityCommand;
      motors[id].torque = mJoints[id + leg_id * leg_dof_].effortCommand;
      ok &= (mJoints[id + leg_id * leg_dof_].errorId == 0);
    }
    if (ok) {
      if (!canfd_api_->send_leg_motors_can(motors, leg_id)) {
        // for (size_t id = 0; id < leg_dof_; id++) {
        //   mJoints[id + leg_id * leg_dof_].errorId = 0x200;
        // }
        RCLCPP_ERROR(
          rclcpp::get_logger("hardware_bridge"), "Failed to set target joint torque on write");
        return hardware_interface::return_type::ERROR;
      } else {
        // for (size_t id = 0; id < leg_dof_; id++) {
        //   if (mJoints[id + leg_id * leg_dof_].errorId == 0x200) {
        //     mJoints[id + leg_id * leg_dof_].errorId = 0x00;  // clear send error
        //   }
        // }
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("hardware_bridge"), clock_, 10000,
        "[write] Leg %ld has motor error, skip to send command", leg_id);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace tita_locomotion
PLUGINLIB_EXPORT_CLASS(tita_locomotion::HardwareBridge, hardware_interface::SystemInterface)
