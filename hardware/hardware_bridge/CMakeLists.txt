cmake_minimum_required(VERSION 3.8)
project(hardware_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hardware_bridge_node.cpp
)

# ament_auto_add_executable(${PROJECT_NAME}_node
    # src/main.cpp
# )

pluginlib_export_plugin_description_file(hardware_interface hardware_bridge_plugin.xml)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
