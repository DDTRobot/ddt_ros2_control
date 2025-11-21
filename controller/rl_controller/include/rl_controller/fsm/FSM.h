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

#ifndef RL_CONTROLLER__FSM__FSM_H_
#define RL_CONTROLLER__FSM__FSM_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "FSMState.h"
#include "FSMState_JointPD.h"
#include "FSMState_Passive.h"
#include "FSMState_RL.h"
#include "FSMState_TransformDown.h"
#include "FSMState_TransformUp.h"
#include "rl_controller/common/enumClass.h"

struct FSMStateList
{
  FSMState_Passive * passive;
  // FSMState_BalanceStand * balanceStand;
  FSMState_TransformUp * transformUp;
  FSMState_JointPD * jointPD;
  FSMState_TransformDown * transformDown;
  std::vector<FSMState *> rlLists;

  void deletePtr()
  {
    delete passive;
    // delete balanceStand;
    delete transformUp;
    delete jointPD;
    delete transformDown;
    // delete rl;
    for (auto ptr : rlLists) {
      delete ptr;
    }
    rlLists.clear();
  }
};

class FSM
{
public:
  FSM(std::shared_ptr<ControlFSMData> data);
  ~FSM();
  void initialize();
  void run();
  std::string getCurrentStateName() { return _currentState->_stateName; };

private:
  FSMState * getNextState(std::string stateName);
  bool checkSafty();
  std::unordered_map<std::string, FSMState *> _stateMap;
  std::shared_ptr<ControlFSMData> _data;
  FSMState * _currentState;
  FSMState * _nextState;
  std::string _nextStateName;
  FSMStateList _stateList;
  FSMMode _mode;
  long long _startTime;
  int count;
};

#endif  // RL_CONTROLLER__FSM__FSM_H_
