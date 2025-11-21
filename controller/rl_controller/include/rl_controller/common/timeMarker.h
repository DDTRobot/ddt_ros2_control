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

#ifndef RL_CONTROLLER__COMMON__TIMEMARKER_H_
#define RL_CONTROLLER__COMMON__TIMEMARKER_H_

#include <sys/time.h>
#include <unistd.h>

#include <iostream>

// 时间戳  微秒级， 需要#include <sys/time.h>
inline long long getSystemTime()
{
  struct timeval t;
  gettimeofday(&t, NULL);
  return 1000000 * t.tv_sec + t.tv_usec;
}
// 时间戳  秒级， 需要getSystemTime()
inline double getTimeSecond()
{
  double time = getSystemTime() * 0.000001;
  return time;
}
// 等待函数，微秒级，从startTime开始等待waitTime微秒
inline void absoluteWait(long long startTime, long long waitTime)
{
  if (getSystemTime() - startTime > waitTime) {
    std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!"
              << std::endl
              << "The program has already cost " << getSystemTime() - startTime << "us."
              << std::endl;
  }
  while (getSystemTime() - startTime < waitTime) {
    usleep(50);
  }
}

#endif  // RL_CONTROLLER__COMMON__TIMEMARKER_H_
