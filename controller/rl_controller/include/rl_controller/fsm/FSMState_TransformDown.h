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

#ifndef RL_CONTROLLER__FSM__FSMSTATE_TRANSFORMDOWN_H_
#define RL_CONTROLLER__FSM__FSMSTATE_TRANSFORMDOWN_H_

#include <memory>
#include <string>

#include "FSMState.h"

class FSMState_TransformDown : public FSMState
{
public:
  FSMState_TransformDown(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_TransformDown() {}

  void enter();
  void run();
  void exit();
  std::string checkTransition();

private:
  void _FoldLegs(const int & iter);
  DVec<scalar_t> _SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini,
    const DVec<scalar_t> & fin);
  TransformDownParameters * td_params_;
  unsigned long long _state_iter;
  // JPos
  DVec<scalar_t> fold_jpos, initial_jpos;
  int fold_ramp_iter;
  const float timer_fold = 2.0;  // timer unit : second
  bool transform_finish_ = false;
};

#endif  // RL_CONTROLLER__FSM__FSMSTATE_TRANSFORMDOWN_H_
