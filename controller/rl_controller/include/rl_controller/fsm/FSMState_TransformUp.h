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

#ifndef RL_CONTROLLER__FSM__FSMSTATE_TRANSFORMUP_H_
#define RL_CONTROLLER__FSM__FSMSTATE_TRANSFORMUP_H_
#include <memory>
#include <string>

#include "FSMState.h"
/**
 *
 */
class FSMState_TransformUp : public FSMState
{
public:
  FSMState_TransformUp(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_TransformUp() {}

  // Behavior to be carried out when entering a state
  void enter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  std::string checkTransition();

  // Manages state specific transitions
  //   TransitionData transition();

  // Behavior to be carried out when exiting a state
  void exit();

  //   TransitionData testTransition();

private:
  TransformUpParameters * tu_params_;
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  static constexpr int StandUp = 0;
  static constexpr int FoldLegs = 1;
  static constexpr int RollOver = 2;

  unsigned long long _state_iter;
  int _flag = FoldLegs;

  // JPos
  DVec<scalar_t> initial_jpos, fold_jpos, stand_jpos, f_ff;

  int fold_ramp_iter, rollover_ramp_iter, standup_ramp_iter;

  void _RollOver(const int & iter);
  void _StandUp(const int & iter);
  void _FoldLegs(const int & iter);

  bool _UpsideDown();
  DVec<scalar_t> _SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini,
    const DVec<scalar_t> & fin);
};

#endif  // RL_CONTROLLER__FSM__FSMSTATE_TRANSFORMUP_H_
