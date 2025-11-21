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

#ifndef RL_CONTROLLER__COMMON__ENUMCLASS_H_
#define RL_CONTROLLER__COMMON__ENUMCLASS_H_
#include <string>

#include "cppTypes.h"

enum point { X, Y, Z };

enum rpy { ROLL, PITCH, YAW };

enum quat { QW, QX, QY, QZ };

struct LowlevelCmd
{
  explicit LowlevelCmd(size_t size)
  {
    tau_cmd.setZero(size);
    qd.setZero(size);
    qd_dot.setZero(size);
    kp.setZero(size);
    kd.setZero(size);
  }
  void zero()
  {
    tau_cmd.setZero();
    qd.setZero();
    qd_dot.setZero();
    kp.setZero();
    kd.setZero();
  }

  DVec<scalar_t> tau_cmd;
  DVec<scalar_t> qd;
  DVec<scalar_t> qd_dot;
  DVec<scalar_t> kp;
  DVec<scalar_t> kd;
};

struct LowlevelState
{
  explicit LowlevelState(size_t size) { zero(size); }
  void zero(size_t size)
  {
    accelerometer = Vec3<scalar_t>::Zero();
    gyro = Vec3<scalar_t>::Zero();
    quat << 1, 0, 0, 0;
    q.setZero(size);
    dq.setZero(size);
    tau_est.setZero(size);
  }
  Vec3<scalar_t> accelerometer;
  Vec3<scalar_t> gyro;
  Quat<scalar_t> quat;
  DVec<scalar_t> q, dq, tau_est;
};

struct RemoteControlData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RemoteControlData() { zero(); }
  std::string fsm_name_;
  Vec3<scalar_t> twist_linear;
  Vec3<scalar_t> twist_angular;
  Vec3<scalar_t> pose_position;
  Quat<scalar_t> pose_orientation;
  double two_wheel_distance;
  void zero()
  {
    fsm_name_ = "idle";
    twist_linear.Zero();
    twist_angular.Zero();
    pose_position.Zero();
    pose_orientation.Zero();
    pose_orientation << 1, 0, 0, 0;
    two_wheel_distance = 0;
  }
};

enum class FSMMode { NORMAL, CHANGE };

#endif  // RL_CONTROLLER__COMMON__ENUMCLASS_H_
