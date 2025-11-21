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

#include "rl_controller/fsm/FSM.h"

#include <iostream>

#include "rl_controller/fsm/FSMState_RLPPO.h"

FSM::FSM(std::shared_ptr<ControlFSMData> data) : _data(data)
{
  _stateList.passive = new FSMState_Passive(_data);
  // _stateList.balanceStand = new FSMState_BalanceStand(_data);
  _stateList.transformUp = new FSMState_TransformUp(_data);
  _stateList.jointPD = new FSMState_JointPD(_data);
  _stateList.transformDown = new FSMState_TransformDown(_data);
  // _stateList.rlHopturn = new FSMState_RL_Hopturn(_data, &_data->params->rl_hopturn_params, "hopturn");
  _stateMap = {
    {"idle", _stateList.passive},
    // {FSMStateName::BALANCE_STAND, nullptr},
    {"transform_up", _stateList.transformUp},
    {"joint_pd", _stateList.jointPD},
    {"transform_down", _stateList.transformDown},
    // {"hopturn", _stateList.rlHopturn}
  };

  for (size_t i = 0; i < _data->params->rl_params.size(); i++) {
    auto policy_name = _data->params->rl_policy_names[i];
    FSMState * rl_fsm;
    if (_data->params->rl_params[i].policy_type == "ppo") {
      rl_fsm = new FSMState_RLPPO(_data, &_data->params->rl_params[i], policy_name);
    } else if (_data->params->rl_params[i].policy_type == "np3o") {
      rl_fsm = new FSMState_RL(_data, &_data->params->rl_params[i], policy_name);
    } else {
      throw std::runtime_error("RL policy type not supported");
    }
    _stateList.rlLists.push_back(rl_fsm);
    _stateMap.insert({policy_name, rl_fsm});
  }
  std::cout << "All avaliable FSM states:" << std::endl;
  for (const auto & pair : _stateMap) {
    std::cout << "\t\t" << pair.first << std::endl;
  }
  initialize();
}

FSM::~FSM() { _stateList.deletePtr(); }

void FSM::initialize()
{
  count = 0;
  _currentState = _stateList.passive;
  _currentState->enter();
  _nextState = _currentState;
  _mode = FSMMode::NORMAL;
  std::cout << "FSM state now: " << _currentState->_stateName.c_str() << std::endl;
}

void FSM::run()
{
  if (!checkSafty()) {
    _currentState = _stateList.passive;
    _currentState->run();
    return;
  }

  if (_mode == FSMMode::NORMAL) {
    _currentState->run();
    _nextStateName = _currentState->checkTransition();
    if (_nextStateName != _currentState->_stateName) {
      _mode = FSMMode::CHANGE;
      _nextState = getNextState(_nextStateName);
    }
  } else if (_mode == FSMMode::CHANGE) {
    std::cout << "FSM state changed from \"" << _currentState->_stateName.c_str();
    _currentState->exit();
    _currentState = _nextState;
    _currentState->enter();
    _mode = FSMMode::NORMAL;
    _currentState->run();
    std::cout << "\" to \"" << _currentState->_stateName.c_str() << "\"" << std::endl;
  }
  // limit torque out put
  for (Eigen::Index i = 0; i < _data->low_cmd->tau_cmd.size(); i++) {
    std::clamp(
      _data->low_cmd->tau_cmd[i], _data->params->torque_limit[i], -_data->params->torque_limit[i]);
  }

  count++;
}

FSMState * FSM::getNextState(std::string stateName)
{
  auto it = _stateMap.find(stateName);
  if (it != _stateMap.end()) {
    return it->second;
  }
  return _stateList.passive;
}

bool FSM::checkSafty()
{
  bool isSafe = true;
  //
  // auto rBody = ori::quaternionToRotationMatrix(_data->low_state->quat);
  // if (rBody(2, 2) < -0.5) isSafe = false;
  // if (_currentState->_stateName == FSMStateName::BALANCE_STAND) {
  //   if (std::abs(_data->wheel_legged_data->com_position_relative(point::X)) > 0.15) isSafe = false; //TODO:
  // }
  return isSafe;
}