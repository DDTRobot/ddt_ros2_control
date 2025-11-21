cmake_minimum_required(VERSION 3.5)
project(gazebo_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gazebo_bridge.cpp
)

pluginlib_export_plugin_description_file(gazebo_ros2_control gazebo_bridge_plugin.xml)

## Install
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
