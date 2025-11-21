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

#ifndef RL_CONTROLLER__COMMON__MATH__MATHUTILITIES_H_
#define RL_CONTROLLER__COMMON__MATH__MATHUTILITIES_H_

#include <eigen3/Eigen/Dense>
#include <vector>
/*!
 * Square a number
 */
template <typename T>
T square(T a)
{
  return a * a;
}

/*!
 * Are two eigen matrices almost equal?
 */
template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T> & a, const Eigen::MatrixBase<T> & b, T2 tol)
{
  long x = T::RowsAtCompileTime;
  long y = T::ColsAtCompileTime;

  if (T::RowsAtCompileTime == Eigen::Dynamic || T::ColsAtCompileTime == Eigen::Dynamic) {
    assert(a.rows() == b.rows());
    assert(a.cols() == b.cols());
    x = a.rows();
    y = a.cols();
  }

  for (long i = 0; i < x; i++) {
    for (long j = 0; j < y; j++) {
      T2 error = std::abs(a(i, j) - b(i, j));
      if (error >= tol) return false;
    }
  }
  return true;
}

template <typename T>
static inline Eigen::Matrix<T, Eigen::Dynamic, 1> vectorToEigen(const std::vector<T> & vec)
{
  return Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>(vec.data(), vec.size());
}
template <typename T>
static std::vector<T> eigenToVector(const Eigen::Matrix<T, Eigen::Dynamic, 1> & eigen_vec)
{
  return std::vector<T>(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());
}
static inline Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> d2f(
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & eigen_vec)
{
  return eigen_vec.template cast<float>();
}

static inline Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> f2d(
  const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & eigen_vec)
{
  return eigen_vec.template cast<double>();
}

#endif  // RL_CONTROLLER__COMMON__MATH__MATHUTILITIES_H_
