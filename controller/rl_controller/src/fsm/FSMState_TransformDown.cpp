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

#include "rl_controller/fsm/FSMState_TransformDown.h"

FSMState_TransformDown::FSMState_TransformDown(std::shared_ptr<ControlFSMData> data)
: FSMState(data, "transform_down")
{
  td_params_ = &_data->params->transform_down_params;
  fold_jpos = vectorToEigen(td_params_->fold_jpos);
}

void FSMState_TransformDown::enter()
{
  this->_nextStateName = this->_stateName;
  fold_ramp_iter = td_params_->fold_timer * _data->params->update_rate;
  _state_iter = 0;
  transform_finish_ = false;
  initial_jpos = this->_data->low_state->q;
  for (auto index : _data->params->wheel_indices) {
    fold_jpos(index) = initial_jpos(index);
  }
  fold_ramp_iter = fold_ramp_iter * (initial_jpos - fold_jpos).cwiseAbs().maxCoeff();  //
}

void FSMState_TransformDown::run()
{
  _data->low_cmd->zero();

  _FoldLegs(_state_iter);
  ++_state_iter;
}

DVec<scalar_t> FSMState_TransformDown::_SetJPosInterPts(
  const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini, const DVec<scalar_t> & fin)
{
  if (ini.size() != fin.size()) {
    std::cerr << "[FSMState_TransformDown] ERROR: ini and fin must have the same size" << std::endl;
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

void FSMState_TransformDown::_FoldLegs(const int & curr_iter)
{
  auto kp_joint = vectorToEigen(td_params_->joint_kp);
  auto kd_joint = vectorToEigen(td_params_->joint_kd);
  DVec<scalar_t> inter_jpos = _SetJPosInterPts(curr_iter, fold_ramp_iter, initial_jpos, fold_jpos);
  _data->low_cmd->qd = inter_jpos;
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();

  _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(inter_jpos - _data->low_state->q) +
                            kd_joint.cwiseProduct(-_data->low_state->dq);
  for (auto index : _data->params->wheel_indices) {
    _data->low_cmd->tau_cmd(index) = kd_joint(index) * -_data->low_state->dq(index);
  }
  if (curr_iter >= fold_ramp_iter + 100) {
    //   if (_UpsideDown()) {
    //     _flag = RollOver;
    //     initial_jpos = fold_jpos;
    //   } else {
    //     _flag = StandUp;
    //     initial_jpos = fold_jpos;
    //   }
    transform_finish_ = true;
  }
}

std::string FSMState_TransformDown::checkTransition()
{
  this->_nextStateName = this->_stateName;
  auto fsm_state_name = _data->rc_data->fsm_name_;
  // if (fsm_state_name == "transform_up") {
  //   this->_nextStateName = "transform_up";
  // } else if (fsm_state_name == "idle") {  // normal c
  //   this->_nextStateName = "idle";
  // }

  if (transform_finish_) {
    this->_nextStateName = "idle";
  }

  return this->_nextStateName;
}

void FSMState_TransformDown::exit()
{
  transform_finish_ = false;
  // Nothing to clean up when exiting
}
