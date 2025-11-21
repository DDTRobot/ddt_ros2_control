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

#include "rl_controller/rl_controller_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pluginlib/class_list_macros.hpp"
namespace rl_controller
{
RlController::RlController() {}

controller_interface::CallbackReturn RlController::on_init()
{
  try {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    command_interface_types_ =
      auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    state_interface_types_ =
      auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    sensor_names_ = auto_declare<std::vector<std::string>>("sensors", sensor_names_);
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(sensor_names_[0]));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // param list init
  param_listener_ = std::make_shared<rl_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();
  setup_controller();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RlController::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (std::string & joint_name : joint_names_) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Get joint name : %s", joint_name.c_str());
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();
    joint->name = joint_name;
    joints_.emplace_back(joint);
  }
  // topics QoS
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(10);
  cmd_vel_subscription_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    ros_topic::manager_twist_command, rclcpp::SystemDefaultsQoS(),
    std::bind(&RlController::cmd_vel_cb, this, std::placeholders::_1));
  posestamped_subscription_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    ros_topic::manager_pose_command, rclcpp::SystemDefaultsQoS(),
    std::bind(&RlController::posestamped_cb, this, std::placeholders::_1));
  fsm_goal_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
    ros_topic::manager_key_command, qos,
    std::bind(&RlController::fsm_goal_cb, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RlController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : command_interface_types_) {
      conf_names.push_back(joint->name + "/" + interface_type);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration RlController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (std::shared_ptr<Joint> joint : joints_) {
    for (const auto & interface_type : state_interface_types_)
      conf_names.push_back(joint->name + "/" + interface_type);
  }
  for (auto name : imu_sensor_->get_state_interface_names()) conf_names.push_back(name);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type RlController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    update_control_parameters();
  }
  // controlData_->params->dt = period.seconds();
  // State Update
  int id = 0;
  auto state = controlData_->low_state;
  // clang-format off
  for (std::shared_ptr<Joint> joint : joints_) {
    state->q(id) = joint->position_handle->get().get_value();
    state->dq(id) = joint->velocity_handle->get().get_value();
    state->tau_est(id) = joint->effort_handle->get().get_value();
    id++;
  }
  state->accelerometer = Vec3<scalar_t>(
    imu_sensor_->get_linear_acceleration()[0], 
    imu_sensor_->get_linear_acceleration()[1],
    imu_sensor_->get_linear_acceleration()[2]);
  state->gyro = Vec3<scalar_t>(
    imu_sensor_->get_angular_velocity()[0], 
    imu_sensor_->get_angular_velocity()[1],
    imu_sensor_->get_angular_velocity()[2]);
  state->quat = Quat<scalar_t>(
    imu_sensor_->get_orientation()[3], imu_sensor_->get_orientation()[0],
    imu_sensor_->get_orientation()[1], imu_sensor_->get_orientation()[2]);
  // clang-format on
  // Control Update
  FSMController_->run();
  // Update torque
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->position_command_handle->get().set_value(controlData_->low_cmd->qd[id]);
    joints_[id]->velocity_command_handle->get().set_value(controlData_->low_cmd->qd_dot[id]);
    joints_[id]->effort_command_handle->get().set_value(controlData_->low_cmd->tau_cmd[id]);
    joints_[id]->kp_command_handle->get().set_value(controlData_->low_cmd->kp[id]);
    joints_[id]->kd_command_handle->get().set_value(controlData_->low_cmd->kd[id]);
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn RlController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_activate");
  for (std::shared_ptr<Joint> joint : joints_) {
    // Position command
    const auto position_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_command_handle = std::ref(*position_command_handle);

    // Velocity command
    const auto velocity_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_command_handle = std::ref(*velocity_command_handle);

    // Effort command
    const auto effort_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain effort command handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_command_handle = std::ref(*effort_command_handle);

    // kp command
    const auto kp_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name && interface.get_interface_name() == "kp";
      });
    if (kp_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain kp command handle for %s", joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->kp_command_handle = std::ref(*kp_command_handle);

    // kd command
    const auto kd_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name && interface.get_interface_name() == "kd";
      });
    if (kd_command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain kd command handle for %s", joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->kd_command_handle = std::ref(*kd_command_handle);

    // Position state
    const auto position_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (position_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->position_handle = std::ref(*position_handle);
    // Velocity state
    const auto velocity_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });
    if (velocity_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->velocity_handle = std::ref(*velocity_handle);
    // Effort state
    const auto effort_handle = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(), [&joint](const auto & interface) {
        return interface.get_prefix_name() == joint->name &&
               interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
      });
    if (effort_handle == state_interfaces_.end()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Unable to obtain joint state handle for %s",
        joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    joint->effort_handle = std::ref(*effort_handle);
  }
  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RlController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate ");
  // for (uint id = 0; id < joints_.size(); id++) {
  //   joints_[id]->effort_command_handle->get().set_value(0);
  // }
  release_interfaces();
  imu_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RlController::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->position_command_handle->get().set_value(0);
    joints_[id]->velocity_command_handle->get().set_value(0);
    joints_[id]->kp_command_handle->get().set_value(0);
    joints_[id]->kd_command_handle->get().set_value(0);
    joints_[id]->effort_command_handle->get().set_value(0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "on_cleanup ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RlController::on_error(const rclcpp_lifecycle::State &)
{
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->position_command_handle->get().set_value(0);
    joints_[id]->velocity_command_handle->get().set_value(0);
    joints_[id]->kp_command_handle->get().set_value(0);
    joints_[id]->kd_command_handle->get().set_value(0);
    joints_[id]->effort_command_handle->get().set_value(0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "on_error ");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RlController::on_shutdown(const rclcpp_lifecycle::State &)
{
  for (uint id = 0; id < joints_.size(); id++) {
    joints_[id]->position_command_handle->get().set_value(0);
    joints_[id]->velocity_command_handle->get().set_value(0);
    joints_[id]->kp_command_handle->get().set_value(0);
    joints_[id]->kd_command_handle->get().set_value(0);
    joints_[id]->effort_command_handle->get().set_value(0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "on_shutdown ");
  return controller_interface::CallbackReturn::SUCCESS;
}
// TODO:
void RlController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  auto cmd = controlData_->rc_data;
  cmd->twist_linear[point::X] = msg->linear.x;
  cmd->twist_linear[point::Y] = msg->linear.y;
  cmd->twist_linear[point::Z] = msg->linear.z;
  cmd->twist_angular[point::X] = msg->angular.x;
  cmd->twist_angular[point::Y] = msg->angular.y;
  cmd->twist_angular[point::Z] = msg->angular.z;
}

void RlController::posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_stamped_mutex_);
  auto cmd = controlData_->rc_data;
  cmd->pose_position[point::Y] = msg->pose.position.y;
  cmd->pose_position[point::Z] = msg->pose.position.z;
  cmd->pose_orientation[quat::QX] = msg->pose.orientation.x;
  cmd->pose_orientation[quat::QY] = msg->pose.orientation.y;
  cmd->pose_orientation[quat::QZ] = msg->pose.orientation.z;
  cmd->pose_orientation[quat::QW] = msg->pose.orientation.w;
}

void RlController::fsm_goal_cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(fsm_goal_mutex_);
  auto cmd = controlData_->rc_data;
  cmd->fsm_name_ = msg->data;
}

void RlController::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joy_mutex_);
  if (joy_msg_ == nullptr) {
    joy_msg_ = std::make_shared<sensor_msgs::msg::Joy>();
    *joy_msg_ = *msg;
  }
  if (msg->axes[4] != joy_msg_->axes[4] || msg->axes[6] != joy_msg_->axes[6]) {
    auto cmd = controlData_->rc_data;
    if (msg->axes[4] == 1) {
      cmd->fsm_name_ = "idle";
    } else if (msg->axes[4] == -1 && msg->axes[6] == 1) {
      cmd->fsm_name_ = "transform_up";
    } else if (msg->axes[4] == -1 && msg->axes[6] == -1) {
      cmd->fsm_name_ = "rl";
    }
    *joy_msg_ = std::move(*msg);
  }
}

void RlController::setup_controller()
{
  controlData_ = std::make_shared<ControlFSMData>(joint_names_.size());
  setup_control_parameters();
  FSMController_ = std::make_shared<FSM>(controlData_);
}

void RlController::setup_control_parameters() { update_control_parameters(); }

void RlController::update_control_parameters()
{
  // Basic parameters
  auto param = controlData_->params;
  get_node()->get_parameter<int>("update_rate", param->update_rate);
  get_node()->get_parameter<std::vector<long int>>("wheel_indices", param->wheel_indices);
  get_node()->get_parameter<std::vector<long int>>("hip_indices", param->hip_indices);
  get_node()->get_parameter<std::vector<scalar_t>>("torque_limit", param->torque_limit);
  get_node()->get_parameter<std::vector<std::string>>("rl_policy_names", param->rl_policy_names);

  // TransformUpParameters
  auto & rs_params = controlData_->params->transform_up_params;
  get_node()->get_parameter<std::vector<scalar_t>>("transform_up.fold_jpos", rs_params.fold_jpos);
  get_node()->get_parameter<std::vector<scalar_t>>("transform_up.stand_jpos", rs_params.stand_jpos);
  get_node()->get_parameter<std::vector<scalar_t>>("transform_up.ff_torque", rs_params.ff_torque);
  get_node()->get_parameter<scalar_t>("transform_up.fold_timer", rs_params.fold_timer);
  get_node()->get_parameter<scalar_t>("transform_up.stand_timer", rs_params.stand_timer);
  get_node()->get_parameter<std::vector<scalar_t>>("transform_up.joint_kp", rs_params.joint_kp);
  get_node()->get_parameter<std::vector<scalar_t>>("transform_up.joint_kd", rs_params.joint_kd);
  // TransformDownParameters
  auto & td_params = controlData_->params->transform_down_params;
  get_node()->get_parameter<std::vector<scalar_t>>("transform_down.fold_jpos", td_params.fold_jpos);
  get_node()->get_parameter<scalar_t>("transform_down.fold_timer", td_params.fold_timer);
  get_node()->get_parameter<std::vector<scalar_t>>("transform_down.joint_kp", td_params.joint_kp);
  get_node()->get_parameter<std::vector<scalar_t>>("transform_down.joint_kd", td_params.joint_kd);
  // JointPDParameters
  auto & jp_params = controlData_->params->joint_pd_params;
  get_node()->get_parameter<std::vector<scalar_t>>("joint_pd.joint_kp", jp_params.joint_kp);
  get_node()->get_parameter<std::vector<scalar_t>>("joint_pd.joint_kd", jp_params.joint_kd);
  // RLParameters
  auto get_policy_params = [this](const std::string & policy_name, RLParameters & rl_params) {
    get_node()->get_parameter<std::string>(policy_name + ".policy_path", rl_params.policy_path);
    rl_params.policy_path =
      ament_index_cpp::get_package_share_directory("rl_controller") + "/" + rl_params.policy_path;
    get_node()->get_parameter<std::string>(policy_name + ".output_name", rl_params.output_name);
    get_node()->get_parameter<std::string>(policy_name + ".policy_type", rl_params.policy_type);
    // env
    get_node()->get_parameter<int>(policy_name + ".num_obs", rl_params.num_obs);
    get_node()->get_parameter<int>(policy_name + ".num_actions", rl_params.num_actions);
    get_node()->get_parameter<int>(policy_name + ".history_len", rl_params.history_len);
    get_node()->get_parameter<std::vector<std::string>>(
      policy_name + ".observations_name", rl_params.observations_name);
    get_node()->get_parameter<std::vector<std::string>>(
      policy_name + ".commands_name", rl_params.commands_name);
    get_node()->get_parameter<std::vector<scalar_t>>(
      policy_name + ".max_commands", rl_params.max_commands);
    get_node()->get_parameter<std::vector<scalar_t>>(
      policy_name + ".min_commands", rl_params.min_commands);
    get_node()->get_parameter<std::vector<scalar_t>>(
      policy_name + ".commands_comp", rl_params.commands_comp);
    get_node()->get_parameter<scalar_t>(policy_name + ".episode_length", rl_params.episode_length);
    // control
    get_node()->get_parameter<scalar_t>(policy_name + ".time_interval", rl_params.time_interval);
    get_node()->get_parameter<std::vector<scalar_t>>(
      policy_name + ".default_joint_angles", rl_params.default_joint_angles);
    get_node()->get_parameter<std::vector<scalar_t>>(policy_name + ".joint_kp", rl_params.joint_kp);
    get_node()->get_parameter<std::vector<scalar_t>>(policy_name + ".joint_kd", rl_params.joint_kd);
    get_node()->get_parameter<std::vector<scalar_t>>(
      policy_name + ".action_scales", rl_params.action_scales);
    get_node()->get_parameter<scalar_t>(policy_name + ".lin_vel_scale", rl_params.lin_vel_scale);
    get_node()->get_parameter<scalar_t>(policy_name + ".ang_vel_scale", rl_params.ang_vel_scale);
    get_node()->get_parameter<scalar_t>(policy_name + ".dof_pos_scale", rl_params.dof_pos_scale);
    get_node()->get_parameter<scalar_t>(policy_name + ".dof_vel_scale", rl_params.dof_vel_scale);
    // extra
    get_node()->get_parameter<scalar_t>(
      policy_name + ".output_torque_scale", rl_params.output_torque_scale);
    get_node()->get_parameter<std::vector<long int>>(policy_name + ".reindex", rl_params.reindex);
  };

  for (auto policy_name : param->rl_policy_names) {
    auto rl_params = RLParameters();
    get_policy_params(policy_name, rl_params);
    param->rl_params.push_back(rl_params);
  }
  // clang-format on
  RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
}

RlController::~RlController() {}

}  // namespace rl_controller

#include "class_loader/register_macro.hpp"

PLUGINLIB_EXPORT_CLASS(rl_controller::RlController, controller_interface::ControllerInterface)
