#include "keyboard_controller/keyboard_controller.hpp"

const std::vector<std::pair<char, std::string>> KeyboardControllerNode::fsm_state_mapping = {
  {'0', "rl_0"}, {'1', "rl_1"}, {'2', "rl_2"},         {'3', "rl_3"},           {'4', "rl_4"},
  {'5', "jump"}, {'6', "idle"}, {'7', "transform_up"}, {'8', "transform_down"}, {'9', "joint_pd"}};

KeyboardControllerNode::KeyboardControllerNode(const rclcpp::NodeOptions & options)
: Node("keyboard_controller", options)
{
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(10);
  this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    ros_topic::manager_twist_command, rclcpp::SystemDefaultsQoS());
  this->posestamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    ros_topic::manager_pose_command, rclcpp::SystemDefaultsQoS());
  this->fsm_goal_publisher_ =
    this->create_publisher<std_msgs::msg::String>(ros_topic::manager_key_command, qos);
  realtime_cmd_vel_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
      cmd_vel_publisher_);
  realtime_posestamped_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      posestamped_publisher_);
  realtime_fsm_goal_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(fsm_goal_publisher_);

  this->timer_ =
    this->create_wall_timer(10ms, std::bind(&KeyboardControllerNode::PubCmdVelCallBack, this));
  std::shared_ptr<std::thread> read_key_thread =
    std::make_shared<std::thread>(&KeyboardControllerNode::ReadKeyThread, this);
  (*read_key_thread).detach();
  pose_.pose.position.z = MIN_HEIGHT;
  for (size_t i = 0; i < 3; i++) {
    rpy_[i] = 0.0;
  }
  fsm_goal_.data = "idle";
  print_interface();
}
void KeyboardControllerNode::print_interface()
{
  system("clear");
  std::cout << R"(
  -------------------------------------------------------
                      State Machines)"
            << std::endl;
  int count = 0;
  for (const auto & pair : fsm_state_mapping) {
    std::cout << "  " << pair.first << ": " << pair.second;
    count++;
    if (count % 3 == 0) {
      std::cout << std::endl;
    } else {
      std::cout << "  ";
    }
  }
  if (count % 3 != 0) {
    std::cout << std::endl;
  }
  // std::cout << R"(
  // -------------------------------------------------------
  // Moving Around    Postion Control    Orientation Control
  //       w                 ↑                u i o
  //     a s d             ← ↓ →              j k l
  // -------------------------------------------------------
  // )";
  std::cout << R"(
  -------------------------------------------------------
  )" << std::endl;
  std::cout << std::fixed << std::setprecision(2);
  std::cout << std::setw(36) << "state machine now: " << std::setw(6) << GREEN << fsm_goal_.data
            << RESET << std::endl;
  std::cout << R"(
  -------------------------------------------------------
       "r": reset velocity, "y" : reset orientation
  )" << std::endl;
  // std::cout << std::setw(20) << "  speed scale:" << std::setw(6) << RED << speed_scale_ << RESET
  //           << "  pose scale:" << std::setw(6) << RED << pose_scale_ << RESET << std::endl
  //           << std::endl;
  std::cout << "  x_vel(ws):" << std::setw(6) << MAGENTA << twist_.linear.x
            << RESET
            // << "  y_vel(ad):" << std::setw(6) << MAGENTA << twist_.linear.y << RESET
            << " wz_vel(ad):" << std::setw(6) << MAGENTA << twist_.angular.z << RESET << std::endl;
  // std::cout << "  z_vel(zc):" << std::setw(6) << MAGENTA << twist_.linear.x << RESET
  std::cout << "  y_vel(←→):" << std::setw(6) << YELLOW << twist_.linear.y << RESET
            << "  z_vel(↑↓):" << std::setw(6) << YELLOW << twist_.linear.z << RESET << std::endl;

  std::cout << "  pitch(jl):" << std::setw(6) << CYAN << rpy_[0] << RESET
            << "   roll(ik):" << std::setw(6) << CYAN << rpy_[1] << RESET
            << "   yaw(uo): " << std::setw(6) << CYAN << rpy_[2] << RESET << std::endl;
}

int KeyboardControllerNode::get_key()
{
  int ch;
  struct termios oldt;
  struct termios newt;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  // std::cout << "ch : " << ch << std::endl;
  return ch;
}

void KeyboardControllerNode::ReadKeyThread()
{
  while (rclcpp::ok()) {
    int key = get_key();
    // 遍历映射表查找匹配的键
    for (const auto & pair : fsm_state_mapping) {
      if (key == pair.first) {
        fsm_goal_.data = pair.second;
        break;
      }
    }
    switch (key) {
      case 'w':
        twist_.linear.x += STEP_ACCL_X * speed_scale_;
        break;
      case 's':
        twist_.linear.x -= STEP_ACCL_X * speed_scale_;
        break;
      // case 'a':
      //   twist_.linear.y += STEP_ACCL_X * speed_scale_;
      //   break;
      // case 'd':
      //   twist_.linear.y -= STEP_ACCL_X * speed_scale_;
      //   break;
      case 'a':
        twist_.angular.z += STEP_ACCL_W * speed_scale_;
        break;
      case 'd':
        twist_.angular.z -= STEP_ACCL_W * speed_scale_;
        break;
      case 'r':
        twist_.linear.x = 0.0;
        twist_.linear.y = 0.0;
        twist_.linear.z = 0.0;
        twist_.angular.x = 0.0;
        twist_.angular.y = 0.0;
        twist_.angular.z = 0.0;
        break;
      // case '+':
      //   speed_scale_ += 0.1;
      //   break;
      // case '-':
      //   speed_scale_ -= 0.1;
      //   break;
      // case '*':
      //   pose_scale_ += 0.1;
      //   break;
      // case '/':
      //   pose_scale_ -= 0.1;
      //   break;
      case '\033':
        if (get_key() == '[') {
          switch (get_key()) {
            case 'A':  // Up
              twist_.linear.z += STEP_ACCL_X * speed_scale_ * 0.1;
              break;
            case 'B':  // Down
              twist_.linear.z -= STEP_ACCL_X * speed_scale_ * 0.1;
              break;
            case 'C':  // Right
              twist_.linear.y -= STEP_ACCL_X * speed_scale_;
              break;
            case 'D':  // Left
              twist_.linear.y += STEP_ACCL_X * speed_scale_;
              break;
            default:
              break;
          }
        }
        break;
      case 'i':
        rpy_[1] += STEP_ORIENTATION * pose_scale_;
        break;
      case 'k':
        rpy_[1] -= STEP_ORIENTATION * pose_scale_;
        break;
      case 'j':
        rpy_[0] -= STEP_ORIENTATION * pose_scale_;
        break;
      case 'l':
        rpy_[0] += STEP_ORIENTATION * pose_scale_;
        break;
      case 'u':
        rpy_[2] -= STEP_ORIENTATION * pose_scale_;
        break;
      case 'o':
        rpy_[2] += STEP_ORIENTATION * pose_scale_;
        break;
      case 'y':
        rpy_[0] = rpy_[1] = rpy_[2] = 0;
        break;
      case '\x03':  // Ctrl+C
        std::cout << "KEYBOARD WILL BE BACK..." << std::endl;
        rclcpp::shutdown();
        break;
      default:
        break;
    }
    speed_scale_ = clamp(speed_scale_, 0.1, 4.0);
    pose_scale_ = clamp(pose_scale_, 0.1, 4.0);
    twist_.linear.x = clamp(twist_.linear.x, -MAX_VEL_X, MAX_VEL_X);
    twist_.linear.y = clamp(twist_.linear.y, -MAX_VEL_X, MAX_VEL_X);
    twist_.linear.z = clamp(twist_.linear.z, -MAX_VEL_X, MAX_VEL_X);
    twist_.angular.z = clamp(twist_.angular.z, -MAX_VEL_W, MAX_VEL_W);
    for (size_t i = 0; i < 3; i++) {
      rpy_[i] = clamp(rpy_[i], -MAX_ORIENTATION, MAX_ORIENTATION);
    }
    tf2::Quaternion q;
    q.setRPY(rpy_[0], rpy_[1], rpy_[2]);
    pose_.pose.orientation.x = q.x();
    pose_.pose.orientation.y = q.y();
    pose_.pose.orientation.z = q.z();
    pose_.pose.orientation.w = q.w();
    pose_.pose.position.y = clamp(pose_.pose.position.y, -MAX_POSITION, MAX_POSITION);
    pose_.pose.position.z = clamp(pose_.pose.position.z, MIN_HEIGHT, MAX_HEIGHT);
    print_interface();
  }
}

void KeyboardControllerNode::PubCmdVelCallBack()
{
  if (realtime_cmd_vel_publisher_ && realtime_cmd_vel_publisher_->trylock()) {
    realtime_cmd_vel_publisher_->msg_ = twist_;
    realtime_cmd_vel_publisher_->unlockAndPublish();
  }
  if (realtime_posestamped_publisher_ && realtime_posestamped_publisher_->trylock()) {
    realtime_posestamped_publisher_->msg_.header.stamp = this->now();
    realtime_posestamped_publisher_->msg_ = pose_;
    realtime_posestamped_publisher_->unlockAndPublish();
  }
  if (realtime_fsm_goal_publisher_ && realtime_fsm_goal_publisher_->trylock()) {
    realtime_fsm_goal_publisher_->msg_ = fsm_goal_;
    realtime_fsm_goal_publisher_->unlockAndPublish();
  }
}
