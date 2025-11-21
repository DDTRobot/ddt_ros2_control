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

#ifndef RL_CONTROLLER__FSM__FSMSTATE_H_
#define RL_CONTROLLER__FSM__FSMSTATE_H_

#include <iostream>
#include <memory>
#include <string>

#include "rl_controller/common/cppTypes.h"
#include "rl_controller/common/enumClass.h"
#include "rl_controller/fsm/ControlFSMData.h"

class FSMState
{
public:
  FSMState(std::shared_ptr<ControlFSMData> data, std::string stateName)
  : _stateName(stateName),
    _data(data){

    };
  virtual ~FSMState() = 0;

  virtual void enter() = 0;
  virtual void run() = 0;
  virtual void exit() = 0;
  virtual std::string checkTransition() { return "invalid"; }

  std::string _stateName;

protected:
  std::shared_ptr<ControlFSMData> _data;
  std::string _nextStateName;
};

#endif  // RL_CONTROLLER__FSM__FSMSTATE_H_
