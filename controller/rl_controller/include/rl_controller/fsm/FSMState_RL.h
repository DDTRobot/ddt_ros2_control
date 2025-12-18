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

#ifndef RL_CONTROLLER__FSM__FSMSTATE_RL_H_
#define RL_CONTROLLER__FSM__FSMSTATE_RL_H_

#include <memory>
#include <string>
#include <thread>

#include "FSMState.h"
#include "rl_controller/common/timeMarker.h"
#include "rl_controller/inferrer/inferrer_base.hpp"

struct Observations
{
  Vec3<tensor_element_t> lin_vel;
  Vec3<tensor_element_t> ang_vel;
  Vec3<tensor_element_t> gravity;
  DVec<tensor_element_t> commands;
  DVec<tensor_element_t> dof_pos;
  DVec<tensor_element_t> dof_vel;
  DVec<tensor_element_t> last_actions;
  DVec<tensor_element_t> phases{6};  //
  double phase_start_time;

  void reset()
  {
    lin_vel.setZero();
    ang_vel.setZero();
    gravity.setZero();
    if (
      commands.size() == 0 || dof_pos.size() == 0 || dof_vel.size() == 0 ||
      last_actions.size() == 0) {
      throw std::runtime_error("Observations vectors must be initialized before reset");
    } else {
      commands.setZero();
      dof_pos.setZero();
      dof_vel.setZero();
      last_actions.setZero();
    }
    phase_start_time = getTimeSecond();
  }
};
class FSMState_RL : public FSMState
{
public:
  FSMState_RL(
    std::shared_ptr<ControlFSMData> data, RLParameters * rl_params, std::string stateName);
  virtual ~FSMState_RL() {}

  // Behavior to be carried out when entering a state
  void enter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  std::string checkTransition();

  bool transition();

  // Manages state specific transitions
  //   TransitionData transition();

  // Behavior to be carried out when exiting a state
  void exit();

  //   TransitionData testTransition();

protected:
  virtual void update_observations();
  virtual void update_forward();

  DVec<tensor_element_t> reindex(DVec<tensor_element_t> & vec)
  {
    if (rl_params_->reindex.empty()) {
      return vec;
    } else {
      DVec<tensor_element_t> vec_reindex = vec;
      for (auto i = 0UL; i < rl_params_->reindex.size(); i++) {
        vec_reindex[i] = vec[rl_params_->reindex[i]];
      }
      return vec_reindex;
    }
  };

  DVec<tensor_element_t> re_sign(DVec<tensor_element_t> & vec)
  {
    if (rl_params_->re_sign.empty()) {
      return vec;
    } else {
      DVec<tensor_element_t> vec_re_sign = vec;
      for (auto i = 0UL; i < rl_params_->re_sign.size(); i++) {
        vec_re_sign[i] = rl_params_->re_sign[i] * vec[i];
      }
      return vec_re_sign;
    }
  };

  RLParameters * rl_params_;
  Observations obs_;

  DVec<tensor_element_t> obs_vec_;
  DVec<tensor_element_t> obs_history_vec_;
  DVec<tensor_element_t> action_vec_;

  std::unique_ptr<InferrerBase> inferrer_;
  std::thread forward_thread;
  bool threadRunning;
  bool stop_update_ = false;
  bool thread_first_ = true;

private:
  int iter_ = 0;
};

#endif  // RL_CONTROLLER__FSM__FSMSTATE_RL_H_
