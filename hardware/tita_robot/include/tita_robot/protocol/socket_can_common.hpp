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

#ifndef TITA_ROBOT__PROTOCOL__SOCKET_CAN_COMMON_HPP_
#define TITA_ROBOT__PROTOCOL__SOCKET_CAN_COMMON_HPP_

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>

#define GCC_OPTIMIZE_03

#ifdef GCC_OPTIMIZE_03
#pragma GCC optimize("O3")
#else
#pragma GCC optimize("O0")
#endif

namespace can_device
{
namespace socket_can
{
inline bool bind_can_socket(int & fd, const std::string & interface, const bool canfd_on = false)
{
  if (interface.length() >= static_cast<std::string::size_type>(IFNAMSIZ)) {
    throw std::domain_error{"CAN interface name too long"};
    return false;
  }

  fd = socket(PF_CAN, static_cast<int32_t>(SOCK_RAW), CAN_RAW);
  if (0 > fd) {
    throw std::runtime_error{"Failed to open CAN socket"};
    return false;
  }

  // Make it non-blocking so we can use timeouts

  struct ifreq ifr;
  (void)strncpy(&ifr.ifr_name[0U], interface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex) {
    perror("if_nametoindex");
    return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (canfd_on) {
    if (ioctl(fd, SIOCGIFMTU, &ifr) < 0) {
      throw std::runtime_error{"Failed to set CAN FD socket name via ioctl()"};
      return false;
    }
    if (ifr.ifr_mtu != CANFD_MTU) {
      throw std::runtime_error{"Interface does not support CAN FD"};
      return false;
    }
    int _canfd_on = canfd_on ? 1 : 0;
    auto ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &_canfd_on, sizeof(_canfd_on));
    if (ret) {
      throw std::runtime_error{"error whem enable canfd"};
      return false;
    }
  }

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (0 > bind(fd, (struct sockaddr *)&addr, sizeof(addr))) {
    throw std::runtime_error{"Failed to bind CAN socket"};
    return false;
  }

  return true;
}

inline struct timeval to_timeval(const std::chrono::nanoseconds timeout) noexcept
{
  const auto count = timeout.count();
  constexpr auto BILLION = 1'000'000'000LL;
  struct timeval c_timeout;
  c_timeout.tv_sec = static_cast<decltype(c_timeout.tv_sec)>(count / BILLION);
  c_timeout.tv_usec = static_cast<decltype(c_timeout.tv_usec)>((count % BILLION) * (0.001));

  return c_timeout;
}

inline fd_set single_set(int32_t file_descriptor) noexcept
{
  fd_set descriptor_set;
  FD_ZERO(&descriptor_set);
  FD_SET(file_descriptor, &descriptor_set);

  return descriptor_set;
}
}  // namespace socket_can
}  // namespace can_device

#endif  // TITA_ROBOT__PROTOCOL__SOCKET_CAN_COMMON_HPP_
