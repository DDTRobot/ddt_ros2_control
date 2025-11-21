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

#ifndef TELEOP_COMMAND__TELEOP_COMMAND_NODE_HPP_
#define TELEOP_COMMAND__TELEOP_COMMAND_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "ros_utils/topic_names.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "teleop_command/elrs_emwave_device.hpp"
#include "teleop_command/teleop_command_parameters.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
namespace teleop_command
{
typedef struct
{
  uint32_t version;
  uint32_t error_code;
  uint8_t quadruped_mode : 1;
  uint8_t unlock_twins_mode : 1;
  uint8_t button_mode : 5;
  uint8_t use_sdk : 1;
  uint8_t bat1;
  uint8_t bat2;
  uint8_t ip_end;
} TransferDataT;

typedef enum {
  QUADRUPED_MODE = 1,
  UNLOCK_TWINS = 2,
  RL_1 = 3,
  RL_2 = 4,
  RL_3 = 5,
  CART_MODE = 6,
  LOCK_JOINTS = 7,
  USE_SDK = 8,
} mode_switch_t;

class TeleopCmdNode : public rclcpp::Node
{
public:
  explicit TeleopCmdNode(const rclcpp::NodeOptions & options);
  void init();

private:
  void cmd_vel_cb();
  void posestamped_cb();
  void fsm_goal_cb();
  // publishers
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> joy_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> cmd_vel_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> posestamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> fsm_goal_publisher_;
  std::shared_ptr<sensor_msgs::msg::Joy> joy_msg_;
  std_msgs::msg::String joy_key_msg_, fsm_fed_msg_, transition_pre_msg_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Joy>> realtime_joy_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>
    realtime_cmd_vel_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
    realtime_posestamped_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>>
    realtime_fsm_goal_publisher_;

  // param listener
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  void crsf_moderead_cb(const uint8_t data[]);
  void crsf_dataread_cb(const uint16_t channels[]);
  std::unique_ptr<CRSF> crsf_;
  TransferDataT crsf_transfer_data_;
  int last_key;
};
}  // namespace teleop_command
#endif  // TELEOP_COMMAND__TELEOP_COMMAND_NODE_HPP_
