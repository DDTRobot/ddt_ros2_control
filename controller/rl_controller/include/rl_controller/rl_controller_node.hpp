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

#ifndef RL_CONTROLLER__RL_CONTROLLER_NODE_HPP_
#define RL_CONTROLLER__RL_CONTROLLER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "fsm/FSM.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rl_controller/rl_controller_parameters.hpp"
#include "ros_utils/topic_names.hpp"
#include "semantic_components/imu_sensor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
namespace rl_controller
{
using LoanedCommandInterface =
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
using LoanedStateInterface =
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
struct Joint
{
  Joint() {}
  LoanedCommandInterface position_command_handle;
  LoanedCommandInterface velocity_command_handle;
  LoanedCommandInterface effort_command_handle;
  LoanedCommandInterface kp_command_handle;
  LoanedCommandInterface kd_command_handle;

  LoanedStateInterface position_handle;
  LoanedStateInterface velocity_handle;
  LoanedStateInterface effort_handle;
  std::string name;
};

class RlController : public controller_interface::ControllerInterface
{
public:
  ~RlController();
  RlController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void fsm_goal_cb(const std_msgs::msg::String::SharedPtr msg);
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_goal_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  sensor_msgs::msg::Joy::SharedPtr joy_msg_ = nullptr;

protected:
  std::vector<std::shared_ptr<Joint>> joints_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> sensor_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;
  std::shared_ptr<rl_controller::ParamListener> param_listener_;
  rl_controller::Params params_;

protected:
  void setup_controller();
  void setup_control_parameters();
  void update_control_parameters();

  int init_estimator_count_{0};
  std::shared_ptr<FSM> FSMController_;
  std::shared_ptr<ControlFSMData> controlData_;
  std::mutex mutex_;
  std::mutex cmd_vel_mutex_, pose_stamped_mutex_, fsm_goal_mutex_, joy_mutex_, rigid_body_mutex_;
};

}  // namespace rl_controller
#endif  // RL_CONTROLLER__RL_CONTROLLER_NODE_HPP_
