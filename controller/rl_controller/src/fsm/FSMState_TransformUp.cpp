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

#include "rl_controller/fsm/FSMState_TransformUp.h"

FSMState_TransformUp::FSMState_TransformUp(std::shared_ptr<ControlFSMData> data)
: FSMState(data, "transform_up")
{
  // Do nothing
  // Set the pre controls safety checks
  // this->checkSafeOrientation = false;

  // Post control safety checks
  // this->checkPDesFoot = false;
  // this->checkForceFeedForward = false;
  tu_params_ = &_data->params->transform_up_params;
  fold_jpos = vectorToEigen(tu_params_->fold_jpos);
  stand_jpos = vectorToEigen(tu_params_->stand_jpos);
  f_ff = vectorToEigen(tu_params_->ff_torque);
}

void FSMState_TransformUp::enter()
{
  // Default is to not transition
  this->_nextStateName = this->_stateName;
  fold_ramp_iter = tu_params_->fold_timer * _data->params->update_rate;
  standup_ramp_iter = tu_params_->stand_timer * _data->params->update_rate;

  // // Reset the transition data
  // this->transitionData.zero();

  // // Reset iteration counter
  iter = 0;
  _state_iter = 0;

  // initial configuration, position
  initial_jpos = this->_data->low_state->q;
  for (auto index : _data->params->wheel_indices) {
    fold_jpos(index) = initial_jpos(index);
  }

  fold_ramp_iter = fold_ramp_iter * (initial_jpos - fold_jpos).cwiseAbs().maxCoeff();  //
  // PRINT_MAT(initial_jpos - fold_jpos);
  // printf("fold_ramp_iter:%d\n", fold_ramp_iter);
  // printf("maxCoeff      :%f\n", (initial_jpos - fold_jpos).cwiseAbs().maxCoeff());

  // scalar_t body_height = this->_data->_stateEstimator->getResult().position[2];

  _flag = FoldLegs;
  if (!_UpsideDown()) {
    // Proper orientation
    // if (  (0.2 < body_height) && (body_height < 0.45) ){
    //   printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
    //   _flag = StandUp;
    // }else{
    //   printf("[Recovery Balance] body height is %f; Folding legs \n", body_height);
    // }
  } else {
    printf("[Recovery Balance] UpsideDown (%d) \n", _UpsideDown());
    _flag = RollOver;
  }
  _motion_start_iter = 0;
}

bool FSMState_TransformUp::_UpsideDown()
{
  auto rBody = ori::quaternionToRotationMatrix(_data->low_state->quat);
  //pretty_print(this->_data->_stateEstimator->getResult().rBody, std::cout, "Rot");
  //if(this->_data->_stateEstimator->getResult().aBody[2] < 0){
  if (rBody(2, 2) < 0) {  //TODO:
    return true;
  }
  return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */

void FSMState_TransformUp::run()
{
  // printf("flag: %d, _state_iter: %d, _motion_start_iter: %d\n", _flag, _state_iter, _motion_start_iter);
  _data->low_cmd->zero();
  switch (_flag) {
    case StandUp:
      _StandUp(_state_iter - _motion_start_iter);
      // std::cout << "Stand Up" << std::endl;
      break;
    case FoldLegs:
      _FoldLegs(_state_iter - _motion_start_iter);
      // std::cout << "Fold Legs" << std::endl;
      break;
    // case RollOver:
    //   _RollOver(_state_iter - _motion_start_iter);
    //   // std::cout << "Roll Over" << std::endl;
    //   break;
    default:
      std::cout << "ERROR: Transform Up State not defined" << std::endl;
      break;
  }
  ++_state_iter;
}

DVec<scalar_t> FSMState_TransformUp::_SetJPosInterPts(
  const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini, const DVec<scalar_t> & fin)
{
  if (ini.size() != fin.size()) {
    std::cerr << "[FSMState_TransformUp] ERROR: ini and fin must have the same size" << std::endl;
  }

  float a(0.f);
  float b(1.f);

  // if we're done interpolating
  if (curr_iter <= max_iter) {
    b = (float)curr_iter / (float)max_iter;
    a = 1.f - b;
  }

  // compute setpoints
  DVec<scalar_t> inter_pos = a * ini + b * fin;
  return inter_pos;
}

// void FSMState_TransformUp::_RollOver(const int & curr_iter)
// {
//   // _SetJPosInterPts(curr_iter, rollover_ramp_iter, initial_jpos, rolling_jpos);

//   // if(curr_iter > rollover_ramp_iter + rollover_settle_iter){
//   //   _flag = FoldLegs;
//   //   initial_jpos = rolling_jpos;
//   //   _motion_start_iter = _state_iter+1;
//   // }

//   DVec<scalar_t> inter_jpos;
//   Eigen::Map<DVec<scalar_t>> kp_joint(
//     _data->params->joint_kp.data(), _data->params->joint_kp.size()),
//     kd_joint(_data->params->joint_kd.data(), _data->params->joint_kd.size());

//   inter_jpos = _SetJPosInterPts(curr_iter, rollover_ramp_iter, initial_jpos, rolling_jpos);
//   _data->low_cmd->qd = inter_jpos;
//   _data->low_cmd->qd_dot.setZero();
//   _data->low_cmd->kp.setZero();
//   _data->low_cmd->kd.setZero();
//   _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(inter_jpos - _data->low_state->q) +
//                             kd_joint.cwiseProduct(-_data->low_state->dq);
//   _data->low_cmd->tau_cmd(3) = _data->low_cmd->tau_cmd(7) = 0.0;
//   for (Eigen::Index i(0); i < initial_jpos.size(); ++i) {
//     bound(_data->low_cmd->tau_cmd(i), _data->params->torque_limit[i]);
//   }
// }

void FSMState_TransformUp::_StandUp(const int & curr_iter)
{
  auto kp_joint = vectorToEigen(tu_params_->joint_kp);
  auto kd_joint = vectorToEigen(tu_params_->joint_kd);
  DVec<scalar_t> inter_jpos =
    _SetJPosInterPts(curr_iter, standup_ramp_iter, initial_jpos, stand_jpos);
  _data->low_cmd->qd = inter_jpos;
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(inter_jpos - _data->low_state->q) +
                            kd_joint.cwiseProduct(-_data->low_state->dq) + f_ff;
  for (auto index : _data->params->wheel_indices) {
    _data->low_cmd->tau_cmd(index) = kd_joint(index) * -_data->low_state->dq(index);
  }
}

void FSMState_TransformUp::_FoldLegs(const int & curr_iter)
{
  auto kp_joint = vectorToEigen(tu_params_->joint_kp);
  auto kd_joint = vectorToEigen(tu_params_->joint_kd);

  DVec<scalar_t> inter_jpos = _SetJPosInterPts(curr_iter, fold_ramp_iter, initial_jpos, fold_jpos);
  _data->low_cmd->qd = inter_jpos;
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(inter_jpos - _data->low_state->q) +
                            kd_joint.cwiseProduct(-_data->low_state->dq) + f_ff;
  for (auto index : _data->params->wheel_indices) {
    _data->low_cmd->tau_cmd(index) = kd_joint(index) * -_data->low_state->dq(index);
  }

  if (curr_iter >= fold_ramp_iter) {
    if (_UpsideDown()) {
      _flag = RollOver;
      initial_jpos = fold_jpos;
    } else {
      _flag = StandUp;
      initial_jpos = fold_jpos;
    }
    _motion_start_iter = _state_iter + 1;
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */

std::string FSMState_TransformUp::checkTransition()
{
  this->_nextStateName = this->_stateName;
  iter++;
  auto desiredState = _data->rc_data->fsm_name_;
  if (desiredState == "transform_up") {
    // 无操作
    // } else if (desiredState == "idle") {  // normal c
    //   this->_nextStateName = "idle";
  } else if (desiredState == "joint_pd") {
    if ((int)(_state_iter - fold_ramp_iter - standup_ramp_iter) >= 100) {
      this->_nextStateName = "joint_pd";
    }
    // } else if (desiredState == "rl") {  // normal c
    //   if ((int)(_state_iter - fold_ramp_iter - standup_ramp_iter) >= 100) {
    //     this->_nextStateName = "rl";
    //   }
  } else if (desiredState.find("rl_") == 0) {
    if ((int)(_state_iter - fold_ramp_iter - standup_ramp_iter) >= 100) {
      try {
        size_t number = std::stoi(desiredState.substr(3));
        if (number < _data->params->rl_policy_names.size()) {
          this->_nextStateName = _data->params->rl_policy_names[number];
        }
      } catch (const std::invalid_argument & e) {
        std::cerr << "Invalid number format in \"" << desiredState << "\"" << std::endl;
      }
    }
  } else if (desiredState == "transform_down") {  // normal c
    if ((int)(_state_iter - fold_ramp_iter - standup_ramp_iter) >= 100) {
      this->_nextStateName = "transform_down";
    }
  }

  // Get the next state
  return this->_nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */

// TransitionData FSMState_TransformUp::transition() {
//   // Finish Transition
//   switch (this->_nextStateName) {
//     case FSMStateName::PASSIVE:  // normal
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::BALANCE_STAND:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::LOCOMOTION:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::BACKFLIP:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::FRONTJUMP:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::VISION:
//       this->transitionData.done = true;
//       break;

//     default:
//       std::cout << "[CONTROL FSM] Something went wrong in transition"
//                 << std::endl;
//   }

//   // Return the transition data to the FSM
//   return this->transitionData;
// }

/**
 * Cleans up the state information on exiting the state.
 */

void FSMState_TransformUp::exit()
{
  // Nothing to clean up when exiting
}
