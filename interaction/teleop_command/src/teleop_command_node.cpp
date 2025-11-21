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

#include "teleop_command/teleop_command_node.hpp"
namespace teleop_command
{
TeleopCmdNode::TeleopCmdNode(const rclcpp::NodeOptions & options) : Node("teleop_command", options)
{
}
void TeleopCmdNode::init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(shared_from_this());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return;
  }
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(10);

  // publishers
  joy_publisher_ =
    this->create_publisher<sensor_msgs::msg::Joy>(ros_topic::joy, rclcpp::SystemDefaultsQoS());
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    ros_topic::manager_twist_command, rclcpp::SystemDefaultsQoS());
  posestamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    ros_topic::manager_pose_command, rclcpp::SystemDefaultsQoS());
  fsm_goal_publisher_ =
    this->create_publisher<std_msgs::msg::String>(ros_topic::manager_key_command, qos);

  realtime_joy_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Joy>>(joy_publisher_);
  realtime_cmd_vel_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
      cmd_vel_publisher_);
  realtime_posestamped_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      posestamped_publisher_);
  realtime_fsm_goal_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(fsm_goal_publisher_);
  realtime_joy_publisher_->msg_.header.frame_id = "joy";
  realtime_joy_publisher_->msg_.axes.resize(params_.joystick_axes_size, 0);
  realtime_joy_publisher_->msg_.buttons.resize(params_.joystick_buttons_size, 0);
  joy_msg_ = std::make_shared<sensor_msgs::msg::Joy>();
  *joy_msg_ = realtime_joy_publisher_->msg_;
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(1000 / params_.update_rate),
    std::bind(&TeleopCmdNode::fsm_goal_cb, this));

  // others
  this->crsf_transfer_data_.quadruped_mode = 0;
  this->crsf_transfer_data_.button_mode = 0;
  crsf_ = std::make_unique<CRSF>(params_.uart_interface);
  crsf_->onDataReceived(std::bind(&TeleopCmdNode::crsf_dataread_cb, this, std::placeholders::_1));
  crsf_->onModeReceived(std::bind(&TeleopCmdNode::crsf_moderead_cb, this, std::placeholders::_1));
  crsf_->begin();
}
void TeleopCmdNode::cmd_vel_cb()
{
  if (realtime_cmd_vel_publisher_ && realtime_cmd_vel_publisher_->trylock()) {
    auto & msg = realtime_cmd_vel_publisher_->msg_;
    int index = static_cast<int>(std::clamp(1 - joy_msg_->axes[7], 0.0f, 2.0f));
    auto speed_ratio = params_.speed_ratio[index];
    msg.linear.x = -joy_msg_->axes[2] * params_.max_twist_linear * speed_ratio;
    msg.angular.z = joy_msg_->axes[3] * params_.max_twist_angular * speed_ratio;
    if (joy_msg_->axes[5] == 0) {
      msg.linear.y = joy_msg_->axes[0] * 1.0 * speed_ratio;
      msg.linear.z = -joy_msg_->axes[1] * 1.0 * speed_ratio;
    } else {
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
    }

    realtime_cmd_vel_publisher_->unlockAndPublish();
  }
}

void TeleopCmdNode::posestamped_cb()
{
  if (realtime_posestamped_publisher_ && realtime_posestamped_publisher_->trylock()) {
    auto & msg = realtime_posestamped_publisher_->msg_;
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      msg.header.stamp = this->now();
    }
    // period_ = this->now() - rclcpp::Time(msg.header.stamp);
    // RCLCPP_INFO(this->get_logger(), "Duration is %f seconds", period_.seconds());
    msg.header.stamp = this->now();
    // double roll, pitch, yaw;
    // if (joy_msg_->axes[7] == 1) {  // 右switch低位
    //   roll_ = -joy_msg_->axes[0] * params_.orientation_ratio;
    //   pitch_ = -joy_msg_->axes[1] * params_.orientation_ratio;
    // } else if (joy_msg_->axes[7] == 0) {                // 右switch中位
    //   y_ = joy_msg_->axes[0] * params_.position_ratio;  // * params_.position_ratio;
    // if (joy_msg_->axes[5] == 0) {
    // z_ -= joy_msg_->axes[1] * period_.seconds() *
    //       params_.position_ratio;  //-joy_msg_->axes[1] * params_.position_ratio;
    // z_ = clamp<double>(z_, params_.position_z_limits[0], params_.position_z_limits[1]);
    // } else
    double roll{0}, pitch{0}, yaw{0}, x{0}, y{0}, z{0.1};
    if (joy_msg_->axes[5] == -1) {
      roll = -joy_msg_->axes[0] * params_.max_roll;
      pitch = -joy_msg_->axes[1] * params_.max_pitch;
    }
    // } else {      // 右switch高位
    //   yaw_ = joy_msg_->axes[0] * params_.orientation_ratio;
    // }

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    msg.pose.orientation.x = q.getX();
    msg.pose.orientation.y = q.getY();
    msg.pose.orientation.z = q.getZ();
    msg.pose.orientation.w = q.getW();
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    realtime_posestamped_publisher_->unlockAndPublish();
  }
}

void TeleopCmdNode::fsm_goal_cb()
{
  if (realtime_fsm_goal_publisher_ && realtime_fsm_goal_publisher_->trylock()) {
    // RCLCPP_WARN_THROTTLE(
    //   this->get_logger(), *this->get_clock(), 5000, "fsm goal in called");
    auto & msg = realtime_fsm_goal_publisher_->msg_;
    if (joy_msg_->axes[4] == 1) {
      msg.data = "transform_down";
    } else if (joy_msg_->axes[4] == -1) {
      msg.data = "transform_up";
    }

    if (joy_msg_->axes[4] == -1) {
      if (joy_msg_->axes[5] == 0 || joy_msg_->axes[5] == -1) {
        msg.data = "rl_0";
        if (joy_msg_->axes[6] == -1) {
          msg.data = "jump";
        } else if (joy_msg_->buttons[2] == 1) {
          msg.data = "rl_1";
        } else if (joy_msg_->buttons[3] == 1) {
          msg.data = "rl_2";
        } else if (joy_msg_->buttons[4] == 1) {
          msg.data = "rl_3";
        }
      }
    }
    if (joy_key_msg_.data != msg.data) {
      joy_key_msg_.data = msg.data;
      realtime_fsm_goal_publisher_->unlockAndPublish();  // only publish when data changed
    } else {
      realtime_fsm_goal_publisher_->unlock();
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Publish \"fsm_goal_cb\" lock failed");
  }
}

void TeleopCmdNode::crsf_moderead_cb(const uint8_t data[])
{
  switch (data[0]) {
    case CRSF_CUSTOMER_CMD_DISCONNECT:
      RCLCPP_INFO(this->get_logger(), "link disconnect");
      break;
    case CRSF_CUSTOMER_CMD_IS_BINGDING:
      RCLCPP_INFO(this->get_logger(), "elrs is in binding");
      break;
    case CRSF_CUSTOMER_CMD_UART_LINKED:
      RCLCPP_INFO(this->get_logger(), "elrs uart connect success");
      break;
    default:
      break;
  }
}

void TeleopCmdNode::crsf_dataread_cb(const uint16_t channels[])
{
  if (!params_.use_sdk) {
    if (realtime_joy_publisher_ && realtime_joy_publisher_->trylock()) {
      auto & msg = realtime_joy_publisher_->msg_;
      msg.header.stamp = this->now();
      auto normalize = [this](float input, float min, float range) -> float {
        return ((input - min) / range) * 2 - 1;
      };
      float normalized1 = -1.0 * normalize(channels[0], 172, 1638);
      float normalized2 = -1.0 * normalize(channels[1], 172, 1638);
      float normalized3 = -1.0 * normalize(channels[2], 172, 1638);
      float normalized4 = -1.0 * normalize(channels[3], 172, 1638);
      if (std::abs(normalized1) < params_.joystick_deadzone) normalized1 = 0.0;
      if (std::abs(normalized2) < params_.joystick_deadzone) normalized2 = 0.0;
      if (std::abs(normalized3) < params_.joystick_deadzone) normalized3 = 0.0;
      if (std::abs(normalized4) < params_.joystick_deadzone) normalized4 = 0.0;
      msg.axes[0] = (normalized1);
      msg.axes[1] = (normalized2);
      msg.axes[2] = (normalized3);
      msg.axes[3] = (normalized4);
      msg.axes[4] = (-(static_cast<int>(channels[4]) / 800 - 1));
      msg.axes[5] = (-(static_cast<int>(channels[5]) / 900 - 1));
      msg.axes[6] = (-(static_cast<int>(channels[6]) / 800 - 1));
      msg.axes[7] = (-(static_cast<int>(channels[7]) / 900 - 1));
      msg.buttons[0] = static_cast<int>(channels[8] - 988) / 4;  // debug, not use
      *joy_msg_ = msg;
      realtime_joy_publisher_->unlockAndPublish();
      cmd_vel_cb();
      posestamped_cb();
    }
  }
  // channels[8] process as button
  int axis_value = static_cast<int>(channels[8] - 988) / 4;
  // only master can use mode change when joined
  if (this->last_key == 0 && axis_value) {
    if (axis_value == USE_SDK) {
      if (!this->crsf_transfer_data_.use_sdk) {
        params_.use_sdk = true;
        this->crsf_transfer_data_.use_sdk = 1;
      } else {
        params_.use_sdk = false;
        this->crsf_transfer_data_.use_sdk = 0;
      }
    }
  }
  this->last_key = axis_value;

  crsf_->setTransferBackData(
    reinterpret_cast<uint8_t *>(&crsf_transfer_data_), 0, sizeof(crsf_transfer_data_));
}

}  // namespace teleop_command
