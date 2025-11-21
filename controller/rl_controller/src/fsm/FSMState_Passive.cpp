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

#include "rl_controller/fsm/FSMState_Passive.h"

FSMState_Passive::FSMState_Passive(std::shared_ptr<ControlFSMData> data) : FSMState(data, "idle") {}

void FSMState_Passive::enter() {}

void FSMState_Passive::run()
{
  //   std::make_shared<DesiredStateCommand::RemoteControlData>());
  _data->low_cmd->zero();
  for (int i = 0; i < _data->low_cmd->kd.size(); i++) {
    _data->low_cmd->kd(i) = 5;
  }
  for (auto i : _data->params->wheel_indices) {
    _data->low_cmd->kd(i) = 0;
  }
  // if(!_data->params_->ee_name_.empty()){
  //   Eigen::VectorXd kp_joint, kd_joint;
  //   kp_joint.setZero(6);
  //   kd_joint.setZero(6);
  //   kp_joint << 100, 100, 10, 1, 1, 1;
  //   // kd_joint << 1, 1, 1, 0.1, 0.1, 0.1;
  //   _data->low_cmd_->tau_cmd.tail(6) = kp_joint.cwiseProduct(-_data->low_state_->q.tail(6)) + kd_joint.cwiseProduct(-_data->low_state_->dq.tail(6));
  //   PRINT_MAT(_data->low_state_->dq.tail(6));
  // }
  // _data->_stateEstimator->run();
}

void FSMState_Passive::exit() {}

std::string FSMState_Passive::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  auto desire_state = _data->rc_data->fsm_name_;
  if (desire_state == "transform_up") {
    this->_nextStateName = "transform_up";
  } else if (desire_state == "joint_pd") {
    this->_nextStateName = "joint_pd";
  }
  return this->_nextStateName;
}
