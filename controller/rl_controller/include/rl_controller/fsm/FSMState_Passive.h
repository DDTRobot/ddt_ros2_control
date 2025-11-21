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

#ifndef RL_CONTROLLER__FSM__FSMSTATE_PASSIVE_H_
#define RL_CONTROLLER__FSM__FSMSTATE_PASSIVE_H_

#include <memory>
#include <string>

#include "FSMState.h"

class FSMState_Passive : public FSMState
{
public:
  FSMState_Passive(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_Passive() {}

  void enter();
  void run();
  void exit();
  std::string checkTransition();
};

#endif  // RL_CONTROLLER__FSM__FSMSTATE_PASSIVE_H_