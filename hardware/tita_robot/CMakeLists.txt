cmake_minimum_required(VERSION 3.8)
project(tita_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include_directories(include)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH})

add_executable(read_write_example test/read_write_example.cpp)

target_link_libraries(read_write_example ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}/libtita_robot.so
  DESTINATION lib
)

install(
  TARGETS read_write_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(tita_robot)
ament_auto_package()
