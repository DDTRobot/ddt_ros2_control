#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "ros_utils/topic_names.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"

#define STEP_ACCL_X 0.2
#define STEP_ACCL_W 0.2
#define STEP_ORIENTATION 0.01
#define STEP_POSITION 0.01
#define STEP_HEIGHT 0.1

#define MAX_VEL_X 3.0
#define MAX_VEL_W 10.0
#define MAX_ORIENTATION 0.6
#define MAX_POSITION 0.1
#define MIN_HEIGHT 0.1
#define MAX_HEIGHT 0.5

using namespace std::chrono_literals;

class KeyboardControllerNode : public rclcpp::Node
{
public:
  KeyboardControllerNode(const rclcpp::NodeOptions & options);

private:
  template <typename scalar_t>
  scalar_t clamp(scalar_t value, scalar_t min, scalar_t max)
  {
    return std::max(min, std::min(value, max));
  }
  void print_interface();
  int get_key();

  void ReadKeyThread();
  void PubCmdVelCallBack();

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> cmd_vel_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> posestamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> fsm_goal_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>
    realtime_cmd_vel_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
    realtime_posestamped_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>>
    realtime_fsm_goal_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist twist_;
  geometry_msgs::msg::PoseStamped pose_;
  std_msgs::msg::String fsm_goal_;
  double rpy_[3];
  double speed_scale_{1.0}, pose_scale_{1.0};
  static const std::vector<std::pair<char, std::string>> fsm_state_mapping;  // {key, state}
};