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

#ifndef RL_CONTROLLER__FSM__CONTROLFSMDATA_H_
#define RL_CONTROLLER__FSM__CONTROLFSMDATA_H_
#include <memory>

#include "rl_controller/common/Math/MathUtilities.h"
#include "rl_controller/common/Math/orientation_tools.h"
#include "rl_controller/common/RobotParameters.h"
#include "rl_controller/common/enumClass.h"
struct ControlFSMData
{
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ControlFSMData(size_t size)
  {
    params = std::make_shared<RobotControlParameters>();
    low_state = std::make_shared<LowlevelState>(size);
    low_cmd = std::make_shared<LowlevelCmd>(size);
    rc_data = std::make_shared<RemoteControlData>();
  };

  std::shared_ptr<RobotControlParameters> params;
  std::shared_ptr<LowlevelCmd> low_cmd;
  std::shared_ptr<LowlevelState> low_state;
  std::shared_ptr<RemoteControlData> rc_data;
};

#endif  // RL_CONTROLLER__FSM__CONTROLFSMDATA_H_