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

#include "rl_controller/fsm/FSMState_RL.h"

#include <filesystem>

#ifdef USE_ENGINE
#include "rl_controller/inferrer/engine_inferrer.hpp"
#else
#include "rl_controller/inferrer/onnx_inferrer.hpp"
#endif

FSMState_RL::FSMState_RL(
  std::shared_ptr<ControlFSMData> data, RLParameters * rl_params, std::string stateName)
: FSMState(data, stateName)
{
  rl_params_ = rl_params;
  printf("Get policy: %s in \"%s\" fsm\n", rl_params_->policy_path.c_str(), stateName.c_str());
#ifdef USE_ENGINE
  // printf("Use TensorRT engine\n");
  inferrer_ = std::make_unique<EngineInferrer>();
#else
  // printf("Use ONNX runtime\n");
  inferrer_ = std::make_unique<ONNXInferrer>();
#endif
  // std::filesystem::path policy_path(rl_params_->policy_path);
  // std::string ext = policy_path.extension().string();  // 获取扩展名（含点，如 ".onnx"）
  // if (ext == ".onnx") {
  //   inferrer_ = std::make_unique<ONNXInferrer>();
  // } else if (ext == ".engine") {
  //   inferrer_ = std::make_unique<EngineInferrer>();
  // } else {
  //   throw std::runtime_error("Unsupported file type");
  // }
  inferrer_->loadModel(rl_params_->policy_path);
  inferrer_->setOutput(rl_params_->output_name, rl_params_->num_actions);

  obs_vec_.setZero(rl_params_->num_obs);
  obs_history_vec_.setZero(rl_params_->num_obs * rl_params_->history_len);
  action_vec_.setZero(rl_params_->num_actions);
  obs_.commands.setZero(rl_params_->commands_name.size());
  obs_.dof_pos.setZero(rl_params_->num_actions);
  obs_.dof_vel.setZero(rl_params_->num_actions);
  obs_.last_actions.setZero(rl_params_->num_actions);
  if (rl_params_->action_scales.size() == 1) {
    scalar_t base_scale = rl_params_->action_scales[0];
    rl_params_->action_scales.resize(rl_params_->num_actions, base_scale);
  }
}

void FSMState_RL::enter()
{
  obs_.reset();
  update_observations();
  for (int i = 0; i < rl_params_->history_len; i++) {
    obs_history_vec_.segment(i * rl_params_->num_obs, rl_params_->num_obs) = obs_vec_;
  }
  threadRunning = true;
  if (thread_first_) {
    forward_thread = std::thread(&FSMState_RL::update_forward, this);
    thread_first_ = false;
  }
  stop_update_ = false;
  iter_ = 0;
}

void FSMState_RL::run()
{
  _data->low_cmd->zero();
  DVec<tensor_element_t> pos = d2f(_data->low_state->q);
  DVec<tensor_element_t> vel = d2f(_data->low_state->dq);
  for (auto i : _data->params->wheel_indices) {
    pos[i] = .0f;
  }

  // compute command
  std::vector<tensor_element_t> torques;
  for (int i = 0; i < rl_params_->num_actions; i++) {
    tensor_element_t action_scaled = action_vec_[i] * rl_params_->action_scales[i];
    // printf("sad%f" , rl_params_->action_scales[i]);
    // tensor_element_t torque = 0.0;
    if (rl_params_->control_type == "P") {
      bool is_wheel_joint =
        std::find(_data->params->wheel_indices.begin(), _data->params->wheel_indices.end(), i) !=
        _data->params->wheel_indices.end();
      tensor_element_t command =
        action_scaled + (tensor_element_t)rl_params_->default_joint_angles[i];
      _data->low_cmd->kp(i) = is_wheel_joint ? 0.0 : rl_params_->joint_kp[i];
      _data->low_cmd->kd(i) = is_wheel_joint ? 0.0 : rl_params_->joint_kd[i];
      _data->low_cmd->qd(i) = is_wheel_joint ? 0.0 : command;
      _data->low_cmd->qd_dot(i) = 0.0;
      _data->low_cmd->tau_cmd(i) =
        is_wheel_joint ? rl_params_->joint_kp[i] * command - rl_params_->joint_kd[i] * vel[i] : 0.0;
    } else if (rl_params_->control_type == "P_V") {
      bool is_wheel_joint =
        std::find(_data->params->wheel_indices.begin(), _data->params->wheel_indices.end(), i) !=
        _data->params->wheel_indices.end();
      tensor_element_t command =
        action_scaled + (tensor_element_t)rl_params_->default_joint_angles[i];
      _data->low_cmd->kp(i) = is_wheel_joint ? 0.0 : rl_params_->joint_kp[i];
      _data->low_cmd->kd(i) = rl_params_->joint_kd[i];
      _data->low_cmd->qd(i) = is_wheel_joint ? 0.0 : command;
      _data->low_cmd->qd_dot(i) = is_wheel_joint ? action_scaled : 0.0;
      _data->low_cmd->tau_cmd(i) = 0.0;
    } else {
      throw std::runtime_error("[FSMState_RL] Unknown control type");
    }
    // torque *= rl_params_->output_torque_scale;
    // torques.push_back(torque);
  }
  // _data->low_cmd->tau_cmd = f2d(vectorToEigen(torques));
}

void FSMState_RL::exit()
{
  stop_update_ = true;
  // std::cout << "exit RL" << std::endl;
}

std::string FSMState_RL::checkTransition()
{
  this->_nextStateName = this->_stateName;
  auto desiredState = _data->rc_data->fsm_name_;
  // 将 switch 替换为 if-else 结构
  if (desiredState.find("rl_") == 0) {
    try {
      size_t number = std::stoi(desiredState.substr(3));
      if (number < _data->params->rl_policy_names.size()) {
        this->_nextStateName = _data->params->rl_policy_names[number];
      }
    } catch (const std::exception & e) {
      std::cerr << "Invalid number format in " << desiredState << std::endl;
    }
  } else if (desiredState == "transform_down") {
    this->_nextStateName = "transform_down";
    // } else if (desiredState == "idle") {
    //   // normal c
    //   this->_nextStateName = "idle";
  }
  return this->_nextStateName;
}

bool FSMState_RL::transition()
{
  run();
  if (
    rl_params_->episode_length > 1e-4 &&
    getTimeSecond() - obs_.phase_start_time < rl_params_->episode_length) {
    return false;
  }  // For some phase policy, we need to wait for the episode to finish
  // You can add the height decend for 8dof robots to get a smooth landing
  return true;
}

void FSMState_RL::update_observations()
{
  obs_.dof_pos = d2f(_data->low_state->q);
  obs_.dof_vel = d2f(_data->low_state->dq);
  for (auto i : _data->params->wheel_indices) {
    obs_.dof_pos[i] = .0f;
  }

  obs_.ang_vel = 0.97 * d2f(this->_data->low_state->gyro) + 0.03 * obs_.ang_vel;

  // compute gravity
  auto rBody = d2f(ori::quaternionToRotationMatrix(_data->low_state->quat));
  obs_.gravity = rBody * Vec3<tensor_element_t>(0.0, 0.0, -1.0);

  // command
  for (size_t i = 0; i < rl_params_->commands_name.size(); i++) {
    scalar_t command;
    if (rl_params_->commands_name[i] == "lin_vel_x") {
      command = rl_params_->commands_gain[i] * _data->rc_data->twist_linear[point::X] +
                rl_params_->commands_comp[i];
    } else if (rl_params_->commands_name[i] == "lin_vel_y") {
      command = rl_params_->commands_gain[i] * _data->rc_data->twist_linear[point::Y] +
                rl_params_->commands_comp[i];
    } else if (rl_params_->commands_name[i] == "ang_vel_z") {
      command = rl_params_->commands_gain[i] * _data->rc_data->twist_angular[point::Z] +
                rl_params_->commands_comp[i];
    } else if (rl_params_->commands_name[i] == "base_height") {
      command = rl_params_->commands_gain[i] * _data->rc_data->pose_position[point::Z] +
                rl_params_->commands_comp[i];
    } else {
      throw std::runtime_error("Unknown command name: " + rl_params_->commands_name[i]);
    }
    command = std::max(std::min(command, rl_params_->max_commands[i]), rl_params_->min_commands[i]);
    obs_.commands[i] = command * rl_params_->commands_scale[i];
  }
  // phase
  double phase = (getTimeSecond() - obs_.phase_start_time) * M_PI / 2.0f;
  if (getTimeSecond() - obs_.phase_start_time > rl_params_->episode_length) {
    phase = rl_params_->episode_length * M_PI / 2.0f;
  }
  // clang-format off
  obs_.phases << std::sin(phase),
                 std::cos(phase),
                 std::sin(phase/2.0),
                 std::cos(phase/2.0),
                 std::sin(phase/4.0),
                 std::cos(phase/4.0);
  // clang-format on
  // obs_buf
  DVec<tensor_element_t> pos = obs_.dof_pos - d2f(vectorToEigen(rl_params_->default_joint_angles));
  DVec<tensor_element_t> vel = obs_.dof_vel;
  pos = reindex(pos);
  pos = re_sign(pos);
  vel = reindex(vel);
  vel = re_sign(vel);
  std::vector<DVec<tensor_element_t>> observations;
  for (size_t i = 0; i < rl_params_->observations_name.size(); i++) {
    if (rl_params_->observations_name[i] == "ang_vel") {
      observations.push_back(obs_.ang_vel * rl_params_->ang_vel_scale);
    } else if (rl_params_->observations_name[i] == "gravity") {
      observations.push_back(obs_.gravity);
    } else if (rl_params_->observations_name[i] == "commands") {
      observations.push_back(obs_.commands);
    } else if (rl_params_->observations_name[i] == "dof_pos") {
      observations.push_back(pos * rl_params_->dof_pos_scale);
    } else if (rl_params_->observations_name[i] == "dof_pos_nwp") {
      std::vector<int> indices = {0, 1, 2, 4, 5, 6};
      DVec<tensor_element_t> pos_sliced(indices.size());
      for (size_t j = 0; j < indices.size(); ++j) {
        pos_sliced[j] = pos[indices[j]];
      }
      observations.push_back(pos_sliced * static_cast<tensor_element_t>(rl_params_->dof_pos_scale));
    } else if (rl_params_->observations_name[i] == "dof_vel") {
      observations.push_back(vel * rl_params_->dof_vel_scale);
    } else if (rl_params_->observations_name[i] == "last_actions") {
      observations.push_back(obs_.last_actions);
    } else if (rl_params_->observations_name[i] == "phases") {
      observations.push_back(obs_.phases);
    } else {
      throw std::runtime_error("Unknown observation name: " + rl_params_->observations_name[i]);
    }
  }
  int obs_size = 0;
  for (const auto & obs : observations) obs_size += obs.size();
  if (obs_vec_.size() != obs_size) {
    throw std::runtime_error("Obs size does not correct");
  }
  int offset = 0;
  for (const auto & obs : observations) {
    obs_vec_.segment(offset, obs.size()) = obs;
    offset += obs.size();
  }
  // // clang-format off
  // obs_vec_ << obs_.ang_vel * rl_params_->ang_vel_scale,
  //             obs_.gravity,
  //             obs_.commands,
  //             pos * rl_params_->dof_pos_scale,
  //             vel * rl_params_->dof_vel_scale,
  //             obs_.last_actions;
  // // clang-format on
}

void FSMState_RL::update_forward()
{
  const long long interval = static_cast<long long>(rl_params_->time_interval * 1000000);
  while (threadRunning) {
    long long _start_time = getSystemTime();

    if (!stop_update_) {
      update_observations();
      std::vector<std::vector<tensor_element_t>> input_datas;
      std::vector<tensor_element_t> input_data_1 = eigenToVector(obs_vec_);
      std::vector<tensor_element_t> input_data_2 = eigenToVector(obs_history_vec_);
      input_datas.push_back(input_data_1);
      input_datas.push_back(input_data_2);
      action_vec_ = vectorToEigen(inferrer_->computeActions(input_datas));
      obs_.last_actions = action_vec_;
      action_vec_ = reindex(action_vec_);
      action_vec_ = re_sign(action_vec_);
      obs_history_vec_.head(obs_history_vec_.size() - obs_vec_.size()) =
        obs_history_vec_.tail(obs_history_vec_.size() - obs_vec_.size());
      obs_history_vec_.tail(obs_vec_.size()) = obs_vec_;
    }
    absoluteWait(_start_time, interval);
  }
  threadRunning = false;
}
