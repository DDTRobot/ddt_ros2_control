#include "keyboard_controller/keyboard_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<KeyboardControllerNode>(options);
  rclcpp::spin(node);
  return 0;
}