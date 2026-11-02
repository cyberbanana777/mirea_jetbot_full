// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// Ковариации одометрии: диагональ 6x6 из параметра узла.
// Pure C++ without ROS, so it can be unit-tested separately.

#ifndef SERIAL_BRIDGE_CPP__COVARIANCE_HPP_
#define SERIAL_BRIDGE_CPP__COVARIANCE_HPP_

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace serial_bridge_cpp
{

// "Не измеряется": так помечаются оси, которых дифференциальный привод не
// наблюдает (z, крен, тангаж; поперечная и вертикальная скорость и т.д.).
constexpr double UNMEASURED_VARIANCE = 1e6;

// ЗАГЛУШКИ. Реальные дисперсии одометрии jetbot ещё не измерены
// (см. IMPROVEMENTS.md). Для наблюдаемых осей стоят небольшие условные
// значения, для ненаблюдаемых - UNMEASURED_VARIANCE.
// Порядок осей: x, y, z, крен, тангаж, рыскание (yaw).
inline const std::vector<double> & default_pose_covariance_diagonal()
{
  static const std::vector<double> v{
    1e-3, 1e-3, UNMEASURED_VARIANCE, UNMEASURED_VARIANCE, UNMEASURED_VARIANCE, 1e-2};
  return v;
}

// Порядок осей: vx, vy, vz, wx, wy, wz.
inline const std::vector<double> & default_twist_covariance_diagonal()
{
  static const std::vector<double> v{
    1e-3, UNMEASURED_VARIANCE, UNMEASURED_VARIANCE,
    UNMEASURED_VARIANCE, UNMEASURED_VARIANCE, 1e-2};
  return v;
}

// Диагональ (6 неотрицательных конечных чисел) -> ковариация 6x6 (row-major,
// как в nav_msgs/Odometry). Бросает std::invalid_argument при ошибке.
inline std::array<double, 36> diagonal_to_covariance(
  const std::vector<double> & diagonal, const std::string & name)
{
  if (diagonal.size() != 6) {
    throw std::invalid_argument(
            name + " must contain 6 values, got " + std::to_string(diagonal.size()));
  }
  std::array<double, 36> cov{};
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(diagonal[i]) || diagonal[i] < 0.0) {
      throw std::invalid_argument(name + " values must be finite and >= 0");
    }
    cov[i * 6 + i] = diagonal[i];
  }
  return cov;
}

}  // namespace serial_bridge_cpp

#endif  // SERIAL_BRIDGE_CPP__COVARIANCE_HPP_
