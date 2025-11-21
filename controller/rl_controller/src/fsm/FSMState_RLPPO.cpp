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

#include "rl_controller/fsm/FSMState_RLPPO.h"

#include <filesystem>

#include "rl_controller/common/timeMarker.h"

FSMState_RLPPO::FSMState_RLPPO(
  std::shared_ptr<ControlFSMData> data, RLParameters * rl_params, std::string stateName)
: FSMState_RL(data, rl_params, stateName)
{
}

void FSMState_RLPPO::update_forward()
{
  const long long interval = static_cast<long long>(rl_params_->time_interval * 1000000);
  while (threadRunning) {
    long long _start_time = getSystemTime();

    if (!stop_update_) {
      update_observations();
      obs_history_vec_.head(obs_history_vec_.size() - obs_vec_.size()) =
        obs_history_vec_.tail(obs_history_vec_.size() - obs_vec_.size());
      obs_history_vec_.tail(obs_vec_.size()) = obs_vec_;

      std::vector<std::vector<tensor_element_t>> input_datas;
      std::vector<tensor_element_t> input_data_1 = eigenToVector(obs_history_vec_);
      input_datas.push_back(input_data_1);
      action_vec_ = vectorToEigen(inferrer_->computeActions(input_datas));
      obs_.last_actions = action_vec_;

      // DVec<tensor_element_t> action_scaled = action_vec_ * rl_params_->action_scale;
      // command_position_ = action_scaled + d2f(vectorToEigen(rl_params_->default_joint_angles));
      // for(auto i : _data->params->wheel_indices){
      //   torque_[i] = rl_params_->joint_kd[i] * (8.0f*action_vec_[i] - obs_.dof_vel[i]);
      // }
    }
    absoluteWait(_start_time, interval);
  }
  threadRunning = false;
}
