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

#ifndef TITA_ROBOT__PROTOCOL__SOCKET_CAN_ID_HPP_
#define TITA_ROBOT__PROTOCOL__SOCKET_CAN_ID_HPP_

#include <linux/can.h>

#include <stdexcept>
#include <utility>

#include "visibility_control.hpp"

namespace can_device
{
namespace socket_can
{
using IdT = uint32_t;
using LengthT = uint32_t;

constexpr std::size_t MAX_DATA_LENGTH = 64U;

static_assert(
  MAX_DATA_LENGTH == sizeof(std::declval<struct canfd_frame>().data),
  "Unexpected CAN frame data size");
static_assert(std::is_same<IdT, canid_t>::value, "Underlying type of CanId is incorrect");
constexpr IdT EXTENDED_MASK = CAN_EFF_FLAG;
constexpr IdT REMOTE_MASK = CAN_RTR_FLAG;
constexpr IdT ERROR_MASK = CAN_ERR_FLAG;
constexpr IdT EXTENDED_ID_MASK = CAN_EFF_MASK;
constexpr IdT STANDARD_ID_MASK = CAN_SFF_MASK;

class SOCKETCAN_PUBLIC SocketCanTimeout : public std::runtime_error
{
public:
  explicit SocketCanTimeout(const char * const what) : runtime_error{what} {};
};  // class SocketCanTimeout

enum class FrameType : uint32_t { DATA, ERROR, REMOTE };

struct StandardFrame_
{
};

constexpr StandardFrame_ StandardFrame;

struct ExtendedFrame_
{
};

constexpr ExtendedFrame_ ExtendedFrame;

class SOCKETCAN_PUBLIC CanId
{
public:
  CanId() = default;
  explicit CanId(const IdT raw_id, const LengthT data_length = 0U)
  : m_id{raw_id}, m_data_length{data_length}
  {
    (void)frame_type();
  }

  CanId(const IdT id, FrameType type, StandardFrame_) : CanId{id, type, false} {}
  CanId(const IdT id, FrameType type, ExtendedFrame_) : CanId{id, type, true} {}

  CanId & standard() noexcept
  {
    m_id = m_id & (~EXTENDED_MASK);
    return *this;
  }

  CanId & extended() noexcept
  {
    m_id = m_id | EXTENDED_MASK;
    return *this;
  }

  CanId & error_frame() noexcept
  {
    m_id = m_id & (~REMOTE_MASK);
    m_id = m_id | ERROR_MASK;
    return *this;
  }

  CanId & remote_frame() noexcept
  {
    m_id = m_id & (~ERROR_MASK);
    m_id = m_id | REMOTE_MASK;
    return *this;
  }

  CanId & data_frame() noexcept
  {
    m_id = m_id & (~ERROR_MASK);
    m_id = m_id & (~REMOTE_MASK);
    return *this;
  }

  CanId & frame_type(const FrameType type)
  {
    switch (type) {
      case FrameType::DATA:
        (void)data_frame();
        break;
      case FrameType::ERROR:
        (void)error_frame();
        break;
      case FrameType::REMOTE:
        (void)remote_frame();
        break;
      default:
        throw std::logic_error{"CanId: No such type"};
    }
    return *this;
  }

  CanId & identifier(const IdT id)
  {
    constexpr auto MAX_EXTENDED = 0x1FBF'FFFFU;
    constexpr auto MAX_STANDARD = 0x7FFU;

    static_assert(MAX_EXTENDED <= EXTENDED_ID_MASK, "Max extended id value is wrong");
    static_assert(MAX_STANDARD <= STANDARD_ID_MASK, "Max standed id value is wrong");
    const auto max_id = is_extended() ? MAX_EXTENDED : MAX_STANDARD;
    if (max_id < id) {
      throw std::domain_error{"CanId would be truncated!"};
    }
    m_id = m_id & (~EXTENDED_ID_MASK);
    m_id = m_id | id;
    return *this;
  }

  IdT identifier() const noexcept
  {
    const auto mask = is_extended() ? EXTENDED_ID_MASK : STANDARD_ID_MASK;
    return m_id & mask;
  }

  IdT get() const noexcept { return m_id; }

  bool is_extended() const noexcept { return (m_id & EXTENDED_MASK) == EXTENDED_MASK; }

  FrameType frame_type() const
  {
    const auto is_error = (m_id & ERROR_MASK) == ERROR_MASK;
    const auto is_remote = (m_id & REMOTE_MASK) == REMOTE_MASK;
    if (is_error && is_remote) {
      throw std::domain_error{"CanId has both bit 29 and 30 set! Inconsistent"};
    }
    if (is_error) {
      return FrameType::ERROR;
    }
    if (is_remote) {
      return FrameType::REMOTE;
    }
    return FrameType::DATA;
  }

  LengthT length() const noexcept { return m_data_length; }
  IdT m_id{};

private:
  SOCKETCAN_LOCAL CanId(const IdT id, FrameType type, bool is_extended)
  {
    if (is_extended) {
      (void)extended();
    }
    (void)frame_type(type);
    (void)identifier(id);
  }

  LengthT m_data_length{};
};  // class CanId
}  // namespace socket_can
}  // namespace can_device

#endif  // TITA_ROBOT__PROTOCOL__SOCKET_CAN_ID_HPP_
