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

#ifndef TITA_ROBOT__TITA_ROBOT_HPP_
#define TITA_ROBOT__TITA_ROBOT_HPP_
#include <memory>
#include <string>
#include <vector>

#include "canfd_api.hpp"
class tita_robot
{
public:
  explicit tita_robot(size_t num_motors, size_t num_batteries = 2, std::string can_interface = "can0")
  {
    // Initialize the CAN receiver
    canfd_api_ = std::make_unique<can_device::CanfdApi>(num_motors, num_batteries, can_interface);
    motor_num_ = num_motors;
  }
  /**
     * @brief Get the current battery is connected.
     * @param index: the index of battery.
     * @return bool: if battery is connected, return true.
     */
  bool get_battery_is_connected(int index) const;
  /**
     * @brief Get the current battery voltage.
     * @param index: the index of battery.
     * @return float: current battery voltage in volts.
     */
  float get_battery_voltage(int index) const;
  /**
     * @brief Get the current battery temperature.
     * @param index: the index of battery.
     * @return float: current battery temperature in degrees Celsius.
     */
  float get_battery_temperature(int index) const;
  /**
     * @brief Get the current battery current.
     * @param index: the index of battery.
     * @return float: current battery current in amperes.
     */
  float get_battery_current(int index) const;
  /**
     * @brief Get the current battery percentage.
     * @param index: the index of battery.
     * @return float: current battery percentage in percent.
     */
  float get_battery_percentage(int index) const;
  /**
     * @brief Get the current battery cell voltage.
     * @param index: the index of battery.
     * @return std::vector<float>: current battery cell voltage in volts.
     */
  std::vector<float> get_battery_cell_voltage(int index) const;
  /**
     * @brief Get the current states update timeout.
     * @return bool: if current states not update, return true.
     */
  bool imu_data_timeout() const;
  /**
     * @brief Get the current states update timeout.
     * @return bool: if current states not update, return true.
     */
  bool motors_data_timeout() const;
  /**
     * @brief Get the current joint positions in joint space.
     * @return std::vector<double>: current joint positions in radians.
     */
  std::vector<double> get_joint_q() const;

  /**
     * @brief Get the current joint velocities in joint space.
     * @return std::vector<double>: current joint velocities in radians per second.
     */
  std::vector<double> get_joint_v() const;

  /**
     * @brief Get the current joint torques in joint space.
     * @return std::vector<double>: current joint torques in Newton meters.
     */
  std::vector<double> get_joint_t() const;

  /**
     * @brief Get the current joint status.
     * @note Joints status now is in bool value, true means the joint is online(TODO).
     * @return std::vector<uint16_t>: current joint status.
     */
  std::vector<uint16_t> get_joint_status() const;
  /**
     * @brief Get the current imu quaternion of mcu.
     * @note Quaternion sequence is x y z w.
     * @return std::array<double, 4>: current quaternion.
     */
  std::array<double, 4> get_imu_quaternion() const;  // x y z w

  /**
     * @brief Get the current imu acceleration of mcu.
     * @return std::array<double, 3>: current acceleration.
     */
  std::array<double, 3> get_imu_acceleration() const;

  /**
     * @brief Get the current imu angular velocity of mcu.
     * @return std::array<double, 3>: current angular velocity.
     */
  std::array<double, 3> get_imu_angular_velocity() const;

  /**
     * @brief Set the target joint feed-forward torques.
     * @param t the target joint feed-forward torques.
     * @return return true if the target is set successfully.
     */
  bool set_target_joint_t(const std::vector<double> & t);

  /**
     * @brief MIT control method. Set the target joint positions, velocities, kp, kd and feed-forward torques of the
     motors.
     * @param q the target joint positions in radians.
     * @param v the target joint velocities in radians per second.
     * @param kp the target joint proportional gains.
     * @param kd the target joint derivative gains.
     * @param t the target joint feed-forward torques.
     *
     * @return return true if the target is set successfully
     */
  bool set_target_joint_mit(
    const std::vector<double> & q, const std::vector<double> & v, const std::vector<double> & kp,
    const std::vector<double> & kd, const std::vector<double> & t);
   
//   /**
//    * @brief Set mcu board control mode, maybe remove in the future.
//    * @param mode the target mcu mode, option: READY_WAITING AUTO_LOCOMOTION FORCE_DIRECT.
//    *
//    * @return return true if the target is set successfully
//    */
//   bool set_board_mode(can_device::ready_next_t mode);
//   /**
//      * @brief Set mcu board can direct drive motors, default is auto_locomotion(use mcu control).
//      * @param if_sdk true means use control motors sdk, false means use mcu control.
//      *
//      * @return return true if the target is set successfully
//      */
//   bool set_motors_sdk(bool if_sdk);
//   /**
//      * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
//      * @param forward the target forward.
//      * @param yaw the target yaw.
//      * @param pitch the target pitch.
//      * @param roll the target roll.
//      * @param height the target height.
//      * @param split the target split.
//      * @param tilt the target tilt.
//      * @param forward_accel the target forward acceleration.
//      * @param yaw_accel the target yaw acceleration.
//      * @return return true if the target is set successfully
//      */
//   bool set_rc_input(can_device::api_channel_input_t input);
//   /**
//      * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
//      * @param stand true if stand.
//      * @return return true if the target is set successfully
//      */
//   bool set_robot_stand(bool stand);

//   /**
//      * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
//      * @param jump true to start robot charge, then false change to start jump.
//      * @return return true if the target is set successfully
//      */
//   bool set_robot_jump(bool jump);
//   /**
//      * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
//      * @return return true if the target is set successfully
//      */
//   bool set_robot_stop();
//   /**
//      * @brief Get the current robot loco status. Only used in board mode is AUTO_LOCOMOTION.
//      * @return robot_status_t: current robot status.
//      */
//   std::string get_robot_status() const;

//   /**
//      * @brief Get the current robot jump status. Only used in board mode is AUTO_LOCOMOTION.
//      * @return robot_status_t: current jump status.
//      */
//   std::string get_jump_status() const;
private:
  std::unique_ptr<can_device::CanfdApi> canfd_api_;
  size_t motor_num_;
};

#endif  // TITA_ROBOT__TITA_ROBOT_HPP_
