cmake_minimum_required(VERSION 3.8)
project(rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wno-unused-parameter -Wno-deprecated-declarations -Wno-unused-variable -Wno-sign-compare -Wno-dev)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(CUDA REQUIRED)

include_directories()

# generate parameter listener hpp file
generate_parameter_library(rl_controller_parameters
  src/rl_controller_parameters.yaml
)

pluginlib_export_plugin_description_file(controller_interface rl_controller_plugin.xml)

set(FSM_FILES
  src/fsm/FSM.cpp
  src/fsm/FSMState.cpp
  src/fsm/FSMState_JointPD.cpp
  src/fsm/FSMState_Passive.cpp
  src/fsm/FSMState_TransformUp.cpp
  src/fsm/FSMState_TransformDown.cpp
  src/fsm/FSMState_RL.cpp
  # src/fsm/FSMState_RL_Hopturn.cpp
  src/fsm/FSMState_RLPPO.cpp

)

if(DEBUG)
  target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g)
endif()

option(USE_ENGINE "Use TensorRT engine instead of ONNX runtime" OFF)
if(USE_ENGINE)
  find_package(CUDA REQUIRED)
  include_directories(/usr/local/cuda/include)
  link_directories(/usr/local/cuda/lib64)
  include_directories(/usr/local/TensorRT-8.6.1.6/include)
  link_directories(/usr/local/TensorRT-8.6.1.6/lib)

  ament_auto_add_library(${PROJECT_NAME} SHARED
    ${FSM_FILES} src/inferrer/engine_inferrer.cpp
    src/rl_controller_node.cpp
  )
  target_compile_definitions(rl_controller PRIVATE USE_ENGINE) 
  target_link_libraries(rl_controller rl_controller_parameters cudart nvinfer)
else()
  ament_auto_add_library(${PROJECT_NAME} SHARED
    ${FSM_FILES} src/inferrer/onnx_inferrer.cpp
    src/rl_controller_node.cpp
  )
  target_link_libraries(rl_controller rl_controller_parameters onnxruntime)
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
