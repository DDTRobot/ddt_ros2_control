cmake_minimum_required(VERSION 3.8)
project(canfd_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

include_directories(SYSTEM /opt/d1_ros2/include)
find_library(CANFD_API_LIB canfd_api PATHS /opt/d1_ros2/lib REQUIRED NO_DEFAULT_PATH)

set(TEST_TARGETS
  read_joints_imu_status
  read_battery_info
  read_dock_status
  write_joints
)

foreach(target ${TEST_TARGETS})
  add_executable(${target} test/${target}.cpp)
  target_link_libraries(${target} ${CANFD_API_LIB})
endforeach()

install(TARGETS ${TEST_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)
