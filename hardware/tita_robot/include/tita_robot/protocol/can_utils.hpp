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

#ifndef TITA_ROBOT__PROTOCOL__CAN_UTILS_HPP_
#define TITA_ROBOT__PROTOCOL__CAN_UTILS_HPP_

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "linux/can.h"
#include "linux/can/error.h"
#include "linux/can/raw.h"
#include "socket_can_receiver.hpp"
#include "socket_can_sender.hpp"

#define C_END "\033[m"
// #define C_RED "\033[0;32;31m"
#define C_RED "\033[0;31m"
#define C_YELLOW "\033[1;33m"

#ifdef GCC_OPTIMIZE_03
#pragma GCC optimize("O3")
#else
#pragma GCC optimize("O0")
#endif
namespace can_device
{
namespace socket_can
{
using can_std_callback = std::function<void(std::shared_ptr<struct can_frame> recv_frame)>;
using can_fd_callback = std::function<void(std::shared_ptr<struct canfd_frame> recv_frame)>;
class CanRxDev
{
public:
  explicit CanRxDev(
    const std::string & interface, const std::string & name, can_std_callback recv_callback,
    bool is_block, int64_t nano_timeout = -1, uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    init(interface, name, false, recv_callback, nullptr, is_block, nano_timeout);
  }
  explicit CanRxDev(
    const std::string & interface, const std::string & name, can_fd_callback recv_callback,
    bool is_block, int64_t nano_timeout = -1, uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    init(interface, name, true, nullptr, recv_callback, is_block, nano_timeout);
  }

  ~CanRxDev()
  {
    ready_ = false;
    isthreadrunning_ = false;
    if (main_T_ != nullptr) {
      main_T_->join();
    }
    main_T_ = nullptr;
    receiver_ = nullptr;
  }

  bool is_ready() { return ready_; }
  bool is_timeout() { return is_timeout_; }
  void set_filter(const struct can_filter filter[], size_t s)
  {
    if (receiver_ != nullptr) {
      receiver_->set_filter(filter, s);
    }
  }

  std::shared_ptr<canfd_frame> wait_for_can_data_block()
  {
    can_device::socket_can::CanId receive_id{};
    std::string can_type;
    if (canfd_) {
      can_type = "FD";
      rx_fd_frame_ = std::make_shared<struct canfd_frame>();
    } else {
      can_type = "STD";
      rx_std_frame_ = std::make_shared<struct can_frame>();
    }
    if (receiver_ != nullptr) {
      try {
        bool result =
          canfd_ ? receiver_->receive(rx_fd_frame_, std::chrono::nanoseconds(nano_timeout_))
                 : receiver_->receive(rx_std_frame_, std::chrono::nanoseconds(nano_timeout_));
        if (result) {
          is_timeout_ = false;
          return rx_fd_frame_;
        } else {
          return nullptr;
        }
      } catch (const std::exception & ex) {
        if (ex.what()[0] == '$') {
          is_timeout_ = true;
          return nullptr;
        }
      }
    } else {
      isthreadrunning_ = false;
      printf(
        C_RED
        "[CAN_RX %s][ERROR][%s] Error receiving CAN %s message: %s, "
        "no receiver init\r\n" C_END,
        can_type.c_str(), name_.c_str(), can_type.c_str(), interface_.c_str());
    }
    return nullptr;
  }

#ifdef COMMON_PROTOCOL_TEST
  bool testing_setcandata(can_frame frame)
  {
    if (can_std_callback_ != nullptr) {
      can_std_callback_(std::make_shared<struct can_frame>(frame));
      return true;
    }
    return false;
  }
  bool testing_setcandata(canfd_frame frame)
  {
    if (can_fd_callback_ != nullptr) {
      can_fd_callback_(std::make_shared<struct canfd_frame>(frame));
      return true;
    }
    return false;
  }
#endif  // COMMON_PROTOCOL_TEST

private:
  bool ready_;
  bool canfd_;
  bool is_timeout_;
  bool extended_frame_;
  bool isthreadrunning_;
  int64_t nano_timeout_;
  std::string name_;
  std::string interface_;
  can_std_callback can_std_callback_;
  can_fd_callback can_fd_callback_;
  std::unique_ptr<std::thread> main_T_;
  std::shared_ptr<struct can_frame> rx_std_frame_;
  std::shared_ptr<struct canfd_frame> rx_fd_frame_;
  std::unique_ptr<can_device::socket_can::SocketCanReceiver> receiver_;
  uint8_t can_id_offset_ = 0;
  void init(
    const std::string & interface, const std::string & name, bool canfd_on,
    can_std_callback std_callback, can_fd_callback fd_callback, bool is_block, int64_t nano_timeout)
  {
    name_ = name;
    interface_ = interface;
    canfd_ = canfd_on;
    nano_timeout_ = nano_timeout;
    is_timeout_ = false;
    interface_ = interface;
    can_std_callback_ = std_callback;
    can_fd_callback_ = fd_callback;
    try {
      receiver_ = std::make_unique<can_device::socket_can::SocketCanReceiver>(interface_, canfd_);
      if (!is_block) {
        isthreadrunning_ = true;
        ready_ = true;
        main_T_ = std::make_unique<std::thread>(std::bind(&CanRxDev::main_recv_func, this));
      }
    } catch (const std::exception & ex) {
      printf(
        C_RED "[CAN_RX][ERROR][%s] %s receiver creat error! %s\r\n" C_END, name_.c_str(),
        interface_.c_str(), ex.what());
      return;
    }
  }

  bool wait_for_can_data()
  {
    can_device::socket_can::CanId receive_id{};
    std::string can_type;
    if (canfd_) {
      can_type = "FD";
      rx_fd_frame_ = std::make_shared<struct canfd_frame>();
    } else {
      can_type = "STD";
      rx_std_frame_ = std::make_shared<struct can_frame>();
    }

    if (receiver_ != nullptr) {
      try {
        bool result =
          canfd_ ? receiver_->receive(rx_fd_frame_, std::chrono::nanoseconds(nano_timeout_))
                 : receiver_->receive(rx_std_frame_, std::chrono::nanoseconds(nano_timeout_));
        if (result) {
          is_timeout_ = false;
        }
        return result;
      } catch (const std::exception & ex) {
        if (ex.what()[0] == '$') {
          is_timeout_ = true;
          return false;
        }
        printf(
          C_RED "[CAN_RX %s][ERROR][%s] Error receiving CAN %s message: %s - %s\n" C_END,
          can_type.c_str(), name_.c_str(), can_type.c_str(), interface_.c_str(), ex.what());
      }
    } else {
      isthreadrunning_ = false;
      printf(
        C_RED
        "[CAN_RX %s][ERROR][%s] Error receiving CAN %s message: %s, "
        "no receiver init\r\n" C_END,
        can_type.c_str(), name_.c_str(), can_type.c_str(), interface_.c_str());
    }
    return false;
  }

  void main_recv_func()
  {
#ifdef DEBUG_LOG
    printf("[CAN_RX][INFO][%s] Start recv thread: %s\r\n", interface_.c_str(), name_.c_str());
#endif  // DEBUG_LOG
    while (isthreadrunning_ && ready_) {
      if (wait_for_can_data()) {
        if (!canfd_ && can_std_callback_ != nullptr) {
          can_std_callback_(rx_std_frame_);
        } else if (canfd_ && can_fd_callback_ != nullptr) {
          can_fd_callback_(rx_fd_frame_);
        }
      }
    }
#ifdef DEBUG_LOG
    printf("[CAN_RX][INFO][%s] Exit recv thread: %s\r\n", interface_.c_str(), name_.c_str());
#endif  // DEBUG_LOG
  }
};

class CanTxDev
{
public:
  explicit CanTxDev(
    const std::string & interface, const std::string & name, bool extended_frame, bool canfd_on,
    int64_t nano_timeout = -1, uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    can_id_offset_ = can_id_offset;
    ready_ = false;
    name_ = name;
    nano_timeout_ = nano_timeout;
    is_timeout_ = false;
    canfd_on_ = canfd_on;
    interface_ = interface;
    extended_frame_ = extended_frame;
    try {
      sender_ = std::make_unique<can_device::socket_can::SocketCanSender>(interface_, canfd_on_);
    } catch (const std::exception & ex) {
      printf(
        C_RED "[CAN_TX][ERROR][%s] %s sender creat error! %s\r\n" C_END, name_.c_str(),
        interface_.c_str(), ex.what());
      return;
    }
    ready_ = true;
  }

  ~CanTxDev()
  {
    ready_ = false;
    sender_ = nullptr;
  }

  bool send_can_message(struct can_frame & tx_frame)
  {
    return send_can_message(&tx_frame, nullptr);
  }
  bool send_can_message(struct canfd_frame & tx_frame)
  {
    return send_can_message(nullptr, &tx_frame);
  }

  bool is_ready() { return ready_; }
  bool is_timeout() { return is_timeout_; }

private:
  bool ready_;
  bool canfd_on_;
  bool is_timeout_;
  bool extended_frame_;
  int64_t nano_timeout_;
  std::string name_;
  std::string interface_;
  std::unique_ptr<can_device::socket_can::SocketCanSender> sender_;
  uint8_t can_id_offset_;

  bool send_can_message(struct can_frame * std_frame, struct canfd_frame * fd_frame)
  {
    bool self_fd = (fd_frame != nullptr);
    std::string can_type = self_fd ? "FD" : "STD";
    if (canfd_on_ != self_fd) {
      printf(
        C_RED
        "[CAN_TX %s][ERROR][%s] Error sending CAN message: %s - "
        "Not support can/canfd frame mixed\r\n" C_END,
        can_type.c_str(), name_.c_str(), interface_.c_str());
      return false;
    }
    canid_t * canid = self_fd ? &fd_frame->can_id : &std_frame->can_id;

    bool result = true;
    can_device::socket_can::CanId send_id =
      extended_frame_
        ? can_device::socket_can::CanId(
            *canid, can_device::socket_can::FrameType::DATA, can_device::socket_can::ExtendedFrame)
        : can_device::socket_can::CanId(
            *canid, can_device::socket_can::FrameType::DATA, can_device::socket_can::StandardFrame);
    if (sender_ != nullptr) {
      try {
        if (self_fd) {
          sender_->send_fd(
            fd_frame->data, (fd_frame->len == 0) ? 64 : fd_frame->len, send_id,
            std::chrono::nanoseconds(nano_timeout_));
          is_timeout_ = false;
        } else {
          sender_->send(
            std_frame->data, (std_frame->can_dlc == 0) ? 8 : std_frame->can_dlc, send_id,
            std::chrono::nanoseconds(nano_timeout_));
          is_timeout_ = false;
        }
      } catch (const std::exception & ex) {
        if (ex.what()[0] == '$') {
          is_timeout_ = true;
          return false;
        }
        result = false;
        printf(
          C_RED "[CAN_TX %s][ERROR][%s] Error sending CAN message: %s - %s\r\n" C_END,
          can_type.c_str(), name_.c_str(), interface_.c_str(), ex.what());
      }
    } else {
      result = false;
      printf(
        C_RED "[CAN_TX %s][ERROR][%s] Error sending CAN message: %s - No device\r\n" C_END,
        can_type.c_str(), name_.c_str(), interface_.c_str());
    }
    return result;
  }
};

class CanDev
{
public:
  explicit CanDev(
    const std::string & interface, const std::string & name, bool extended_frame,
    can_std_callback recv_callback, bool is_block, int64_t nano_timeout = -1,
    uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    name_ = name;
    send_only_ = false;
    tx_op_ = std::make_unique<CanTxDev>(
      interface, name_, extended_frame, false, nano_timeout, can_id_offset_);
    rx_op_ = std::make_unique<CanRxDev>(
      interface, name_, recv_callback, is_block, nano_timeout, can_id_offset_);
  }
  explicit CanDev(
    const std::string & interface, const std::string & name, bool extended_frame,
    can_fd_callback recv_callback, bool is_block, int64_t nano_timeout = -1,
    uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    name_ = name;
    send_only_ = false;
    tx_op_ = std::make_unique<CanTxDev>(
      interface, name_, extended_frame, true, nano_timeout, can_id_offset_);
    rx_op_ = std::make_unique<CanRxDev>(
      interface, name_, recv_callback, is_block, nano_timeout, can_id_offset_);
  }
  explicit CanDev(
    const std::string & interface, const std::string & name, bool extended_frame, bool canfd_on,
    int64_t nano_timeout = -1, uint8_t can_id_offset = 0)
  : can_id_offset_(can_id_offset)
  {
    name_ = name;
    send_only_ = true;
    tx_op_ = std::make_unique<CanTxDev>(
      interface, name_, extended_frame, canfd_on, nano_timeout, can_id_offset_);
  }

  ~CanDev()
  {
    rx_op_ = nullptr;
    tx_op_ = nullptr;
  }

  std::shared_ptr<canfd_frame> receive_can_message_block()
  {
    return rx_op_->wait_for_can_data_block();
  }

  bool send_can_message(struct can_frame & tx_frame)
  {
#ifdef COMMON_PROTOCOL_TEST
    if (rx_op_ != nullptr) {
      return rx_op_->testing_setcandata(tx_frame);
    }
    return false;
#else
    if (tx_op_ != nullptr) {
      return tx_op_->send_can_message(tx_frame);
    }
    return false;
#endif  // COMMON_PROTOCOL_TEST
  }

  bool send_can_message(struct canfd_frame & tx_frame)
  {
#ifdef COMMON_PROTOCOL_TEST
    if (rx_op_ != nullptr) {
      return rx_op_->testing_setcandata(tx_frame);
    }
    return false;
#else
    if (tx_op_ != nullptr) {
      return tx_op_->send_can_message(tx_frame);
    }
    return false;
#endif  // COMMON_PROTOCOL_TEST
  }

  void set_filter(const struct can_filter filter[], size_t s)
  {
    if (rx_op_ != nullptr) {
      rx_op_->set_filter(filter, s);
    }
  }
  bool is_send_only() { return send_only_; }
  bool is_ready()
  {
    bool result = (tx_op_ == nullptr) ? false : tx_op_->is_ready();
    if (send_only_) {
      return result;
    } else {
      return result && ((rx_op_ == nullptr) ? false : rx_op_->is_ready());
    }
  }
  bool is_rx_timeout() { return rx_op_->is_timeout(); }
  bool is_tx_timeout() { return tx_op_->is_timeout(); }

private:
  bool send_only_;
  std::string name_;
  std::unique_ptr<CanRxDev> rx_op_;
  std::unique_ptr<CanTxDev> tx_op_;
  uint8_t can_id_offset_ = 0;
};

}  // namespace socket_can
}  // namespace can_device

#endif  // TITA_ROBOT__PROTOCOL__CAN_UTILS_HPP_
