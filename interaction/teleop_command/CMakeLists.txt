cmake_minimum_required(VERSION 3.8)
project(teleop_command)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories()

generate_parameter_library(teleop_command_parameters
  src/teleop_command_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    src/teleop_command_node.cpp
    src/elrs_emwave_device.cpp
)

target_link_libraries(${PROJECT_NAME}_lib teleop_command_parameters)

ament_auto_add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

