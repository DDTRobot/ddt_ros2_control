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

#ifndef RL_CONTROLLER__FSM__FSMSTATE_RLPPO_H_
#define RL_CONTROLLER__FSM__FSMSTATE_RLPPO_H_

#include <memory>
#include <string>
#include <thread>

#include "FSMState.h"
#include "FSMState_RL.h"
#include "rl_controller/inferrer/inferrer_base.hpp"
class FSMState_RLPPO : public FSMState_RL
{
public:
  FSMState_RLPPO(
    std::shared_ptr<ControlFSMData> data, RLParameters * rl_params, std::string stateName);
  virtual ~FSMState_RLPPO() {}

protected:
  void update_forward() override;
};

#endif  // RL_CONTROLLER__FSM__FSMSTATE_RLPPO_H_
