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

#ifndef TITA_ROBOT__PROTOCOL__SOCKET_CAN_RECEIVER_HPP_
#define TITA_ROBOT__PROTOCOL__SOCKET_CAN_RECEIVER_HPP_
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>  // for close()

#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include "socket_can_common.hpp"
#include "socket_can_id.hpp"
#include "visibility_control.hpp"

namespace can_device
{
namespace socket_can
{
// using target_can_device = static_cast<std::string>(TARGET_CAN_DEVICE);

class SOCKETCAN_PUBLIC SocketCanReceiver
{
public:
  explicit SocketCanReceiver(const std::string & interface = "can0", bool canfd_on = false)
  {
    if (!bind_can_socket(fd_, interface, canfd_on)) {
      printf("bind_can_socket failed!\r\n");
      return;
    }
  }

  ~SocketCanReceiver() noexcept { (void)close(fd_); }

  bool receive(
    std::shared_ptr<struct can_frame> rx_frame,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero())
  {
    wait(timeout);

    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    const auto nbytes = read(fd_, &frame, sizeof(frame));

    if (nbytes < 0) {
      throw std::runtime_error{strerror(errno)};
    }
    if (static_cast<std::size_t>(nbytes) < CAN_MTU) {
      throw std::logic_error{"read: incomplete CAN frame"};
    }
    if (static_cast<std::size_t>(nbytes) != CAN_MTU) {
      throw std::logic_error{"Message was wrong size"};
    }

    auto receive_id = CanId{frame.can_id, frame.len};
    if (receive_id.frame_type() == can_device::socket_can::FrameType::DATA) {
      rx_frame->can_id = receive_id.standard().get();
      rx_frame->can_id = receive_id.length();
      std::memcpy(rx_frame->data, frame.data, sizeof(rx_frame->data));
      return true;
    }
    return false;
  }

  bool receive(
    std::shared_ptr<struct canfd_frame> rx_frame,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero())
  {
    wait(timeout);

    struct canfd_frame frame;
    const auto nbytes = read(fd_, &frame, sizeof(frame));

    uint8_t except_len = CAN_MTU - 8 + frame.len;
    if (nbytes < 0) {
      throw std::runtime_error{strerror(errno)};
    }
    if (static_cast<std::size_t>(nbytes) < except_len) {
      throw std::runtime_error{"read: incomplete CAN frame"};
    }

    auto receive_id = CanId{frame.can_id, frame.len};
    if (receive_id.frame_type() == can_device::socket_can::FrameType::DATA) {
      rx_frame->can_id = receive_id.standard().get();
      rx_frame->len = receive_id.length();
      std::memcpy(rx_frame->data, frame.data, sizeof(rx_frame->data));
      return true;
    }
    return false;
  }

  void set_filter(const struct can_filter filter[], size_t s)
  {
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filter, s);
  }

private:
  SOCKETCAN_LOCAL void wait(const std::chrono::nanoseconds timeout) const
  {
    if (decltype(timeout)::zero() < timeout) {
      auto c_timeout = to_timeval(timeout);
      auto read_set = single_set(fd_);

      if (0 == select(fd_ + 1, &read_set, NULL, NULL, &c_timeout)) {
        throw SocketCanTimeout{"$CAN Receive Timeout 1"};
      }

      if (!FD_ISSET(fd_, &read_set)) {
        throw SocketCanTimeout{"$CAN Receive Timeout 2"};
      }
    } else {
      auto read_set = single_set(fd_);

      if (0 == select(fd_ + 1, &read_set, NULL, NULL, NULL)) {
        throw SocketCanTimeout{"$CAN Receive Timeout 1"};
      }

      if (!FD_ISSET(fd_, &read_set)) {
        throw SocketCanTimeout{"CAN Receive Timeout 2"};
      }
    }
  }

  int32_t fd_;
};  // class SocketCanReceiver
}  // namespace socket_can
}  // namespace can_device

#endif  // TITA_ROBOT__PROTOCOL__SOCKET_CAN_RECEIVER_HPP_
