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

#include "rl_controller/fsm/FSMState_JointPD.h"
FSMState_JointPD::FSMState_JointPD(std::shared_ptr<ControlFSMData> data)
: FSMState(data, "joint_pd")
{
  // Do nothing
  // Set the pre controls safety checks
  // this->checkSafeOrientation = false;

  // Post control safety checks
  // this->checkPDesFoot = false;
  // this->checkForceFeedForward = false;
  // size_t dof = _data->params->dof_chassis;
  // initial_jpos.setZero(dof);
  jp_params_ = &_data->params->joint_pd_params;
}

void FSMState_JointPD::enter()
{
  // Default is to not transition
  this->_nextStateName = this->_stateName;
  initial_jpos = this->_data->low_state->q;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */

void FSMState_JointPD::run()
{
  _data->low_cmd->zero();
  // scalar_t kp_joint = 10, kd_joint = 0.2;
  auto kp_joint = vectorToEigen(jp_params_->joint_kp);
  auto kd_joint = vectorToEigen(jp_params_->joint_kd);

  DVec<scalar_t> initial_djpos(initial_jpos.size());
  initial_djpos.setZero();
  _data->low_cmd->qd = initial_jpos;
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp = kp_joint;
  _data->low_cmd->kd = kd_joint;
  _data->low_cmd->tau_cmd.setZero();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */

std::string FSMState_JointPD::checkTransition()
{
  this->_nextStateName = this->_stateName;
  auto desire_state = _data->rc_data->fsm_name_;
  // Switch FSM control mode
  if (desire_state.find("rl_") == 0) {
    try {
      size_t number = std::stoi(desire_state.substr(3));
      if (number < _data->params->rl_policy_names.size()) {
        this->_nextStateName = _data->params->rl_policy_names[number];
      }
    } catch (const std::exception & e) {
      std::cerr << "Invalid number format in " << desire_state << std::endl;
    }
  } else if (desire_state == "idle") {  // normal c
    this->_nextStateName = "idle";
  } else if (desire_state == "transform_down") {
    this->_nextStateName = "transform_down";
  }
  // Get the next state
  return this->_nextStateName;
}

/**
 * Cleans up the state information on exiting the state.
 */

void FSMState_JointPD::exit()
{
  // Nothing to clean up when exiting
}
