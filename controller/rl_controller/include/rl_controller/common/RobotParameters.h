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

#ifndef RL_CONTROLLER__COMMON__ROBOTPARAMETERS_H_
#define RL_CONTROLLER__COMMON__ROBOTPARAMETERS_H_
#include <string>
#include <vector>

#include "rl_controller/common/cppTypes.h"

struct CarParameters
{
  bool car_mode{false};
  scalar_t wheel_radius{0.095}, wheel_distance{0.50};
  std::vector<scalar_t> car_lin_vel{-2.0, 2.0}, car_ang_vel{-5.0, 5.0};
  std::vector<scalar_t> car_lin_accl{-1.5, 5.0}, car_ang_accl{-20.0, 20.0};
  // std::vector<scalar_t> fold_jpos, stand_jpos;
  // scalar_t fold_timer, stand_timer;
  // std::vector<scalar_t> ff_torque, joint_kp, joint_kd;
};

struct TransformUpParameters
{
  // bool car_mode{false};
  // std::vector<scalar_t> car_lin_vel{-2.0, 2.0}, car_ang_vel{-5.0, 5.0};
  // std::vector<scalar_t> car_lin_accl{-1.5, 5.0}, car_ang_accl{-20.0, 20.0};
  std::string stand_up_policy{""};
  std::string roll_over_policy{""};
  std::vector<scalar_t> fold_jpos, stand_jpos, roll_jpos, roll_jpos2, roll_jpos3;
  scalar_t fold_timer, stand_timer, roll_timer, roll_timer2, roll_timer3;
  std::vector<scalar_t> ff_torque, joint_kp, joint_kd;
};
struct TransformDownParameters
{
  std::string policy_name{""};
  std::vector<scalar_t> fold_jpos;
  scalar_t rl_down_timer{4.0};
  scalar_t fold_timer;
  std::vector<scalar_t> joint_kp, joint_kd;
};

struct JointPDParameters
{
  std::vector<scalar_t> joint_kp, joint_kd;
};

struct RLParameters
{
  std::string policy_path;
  std::string output_name;          // output of the policy name
  std::string policy_type{"np3o"};  // ppo or np3o
  int num_obs;
  int num_actions;
  int history_len{1};
  std::vector<std::string> observations_name{"ang_vel", "gravity", "commands",
                                             "dof_pos", "dof_vel", "last_actions"};
  std::vector<std::string> commands_name{"lin_vel_x", "lin_vel_y", "ang_vel_z"};
  std::vector<scalar_t> commands_scale{2.0, 2.0, 0.25};
  std::vector<scalar_t> max_commands{1.0, 1.0, 1.0};
  std::vector<scalar_t> min_commands{-1.0, -1.0, -1.0};
  std::vector<scalar_t> commands_comp{0.0, 0.0, 0.0};
  std::vector<scalar_t> commands_gain{1.0, 1.0, 1.0};
  scalar_t episode_length{0};  // 0 means no limit, units: seconds

  scalar_t time_interval{0.02};
  // "P" means all joints use position control,
  // "P_V" mean wheel use velocity control, other use position control
  std::string control_type{"P"};
  std::vector<scalar_t> default_joint_angles;
  std::vector<scalar_t> joint_kp;
  std::vector<scalar_t> joint_kd;
  std::vector<scalar_t> action_scales{
    0.25};  // size equal to num_actions, if size is 1, then all actions use the same scale

  scalar_t lin_vel_scale;
  scalar_t ang_vel_scale;
  scalar_t dof_pos_scale;
  scalar_t dof_vel_scale;

  // scalar_t lin_vel_x_comp{0}, ang_vel_z_comp{0};
  scalar_t output_torque_scale{1.0};
  std::vector<long int> reindex;
  std::vector<scalar_t> re_sign;
  // scalar_t max_lin_vel_x{1.0}, max_lin_vel_y{1.0}, max_ang_vel_z{1.0};
};
struct RobotControlParameters
{
  RobotControlParameters() { torque_limit = {80.0, 20.0, 40.0, 5.0, 80.0, 20.0, 40.0, 5.0}; }
  std::vector<long int> wheel_indices, hip_indices;
  // control param
  // scalar_t dt;           // actual control period
  int update_rate{500};  // control loop update rate
  std::vector<scalar_t> torque_limit;

  CarParameters car_params;
  TransformUpParameters transform_up_params;
  TransformDownParameters transform_down_params;
  JointPDParameters joint_pd_params;
  std::vector<RLParameters> rl_params;

  std::vector<std::string> rl_policy_names;
  std::string rl_policy_jump_name;
  bool locomotion_use_lqr{false};
};

#endif  // RL_CONTROLLER__COMMON__ROBOTPARAMETERS_H_
