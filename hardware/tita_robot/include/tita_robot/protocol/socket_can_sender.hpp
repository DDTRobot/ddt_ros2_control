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

#ifndef TITA_ROBOT__PROTOCOL__SOCKET_CAN_SENDER_HPP_
#define TITA_ROBOT__PROTOCOL__SOCKET_CAN_SENDER_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <stdexcept>
#include <string>

#include "socket_can_common.hpp"
#include "socket_can_id.hpp"
#include "visibility_control.hpp"

namespace can_device
{
namespace socket_can
{
class SOCKETCAN_PUBLIC SocketCanSender
{
public:
  explicit SocketCanSender(
    const std::string & interface = "can0", bool canfd_on = false,
    const CanId & default_id = CanId{})
  : m_default_id{default_id}
  {
    if (!bind_can_socket(fd_, interface, canfd_on)) {
      printf("bind_can_socket failed\r\n");
      return;
    }
  }
  ~SocketCanSender() noexcept
  {
    (void)close(fd_);
    // I'm destructing--there's not much else I can do on an error
    // seem not good
  }

  void send(
    const void * const data, const std::size_t Length,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    send(data, Length, m_default_id, timeout);
  }

  void send(
    const void * const data, const std::size_t length, const CanId id,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    if (length > MAX_DATA_LENGTH) {
      throw std::domain_error{"Size is too large to send via CAN"};
    }
    send_impl(data, length, id, timeout);
  }

  template <typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  void send(
    const T & data, const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    send(data, m_default_id, timeout);
  }

  template <typename T, typename = std::enable_if<!std::is_pointer<T>::value>>
  void send(
    const T & data, const CanId id,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    static_assert(sizeof(data) <= MAX_DATA_LENGTH, "Data type too large for CAN");
    send_impl(reinterpret_cast<const char *>(&data), sizeof(data), id, timeout);
  }

  void send_fd(
    const void * const data, const std::size_t length, const CanId id,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    if (length > MAX_DATA_LENGTH) {
      throw std::domain_error{"Size is too large to send via CAN"};
    }
    send_fd_impl(data, length, id, timeout);
  }

  CanId default_id() const noexcept { return m_default_id; }

private:
  void send_impl(
    const void * const data, const std::size_t length, const CanId id,
    const std::chrono::nanoseconds timeout) const
  {
    // (void)data;
    // (void)length;
    // (void)id;
    (void)timeout;

    struct can_frame data_frame;
    data_frame.can_id = id.get();

    data_frame.can_dlc = static_cast<decltype(data_frame.can_dlc)>(length);

    (void)std::memcpy(static_cast<void *>(&data_frame.data[0U]), data, length);

    if (write(fd_, &data_frame, static_cast<int>(CAN_MTU)) != static_cast<int>(CAN_MTU)) {
      throw std::runtime_error{strerror(errno)};
    }
  }

  void send_fd_impl(
    const void * const data, const std::size_t length, const CanId id,
    const std::chrono::nanoseconds timeout) const
  {
    // (void)data;
    // (void)length;
    // (void)id;
    (void)timeout;

    struct canfd_frame data_frame;
    memset(&data_frame, 0, sizeof(data_frame));
    data_frame.can_id = id.get();
    data_frame.flags = CANFD_FDF | CANFD_BRS;
    data_frame.len = static_cast<decltype(data_frame.len)>(length);
    std::memcpy(static_cast<void *>(&data_frame.data[0U]), data, length);
    if (write(fd_, &data_frame, CANFD_MTU) != CANFD_MTU) {
      throw std::runtime_error{strerror(errno)};
    }
  }

  SOCKETCAN_LOCAL void wait(const std::chrono::nanoseconds timeout) const
  {
    if (decltype(timeout)::zero() < timeout) {
      auto c_timeout = to_timeval(timeout);
      auto write_set = single_set(fd_);

      // wait
      if (0 == select(fd_ + 1, NULL, &write_set, NULL, &c_timeout)) {
        throw SocketCanTimeout{"$CAN Send Timeout [1]"};
      }

      if (!FD_ISSET(fd_, &write_set)) {
        throw SocketCanTimeout{"$CAN Send Timeout [2]"};
      }
    } else {
      // do nothing
      // auto write_set = single_set(fd_);
    }
  }

  int32_t fd_;
  CanId m_default_id;
};
}  // namespace socket_can
}  // namespace can_device

#endif  // TITA_ROBOT__PROTOCOL__SOCKET_CAN_SENDER_HPP_
