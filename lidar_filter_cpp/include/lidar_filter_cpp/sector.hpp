// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// Which beams of a LaserScan lie inside an angular sector.
// Pure C++ without ROS, so it can be unit-tested separately.

#ifndef LIDAR_FILTER_CPP__SECTOR_HPP_
#define LIDAR_FILTER_CPP__SECTOR_HPP_

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace lidar_filter_cpp
{

// Normalize angle to [-pi, pi]
inline double normalize_angle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

// Is the (normalized) angle inside [start, end]? Handles sectors
// that cross the -pi/pi boundary (start > end).
inline bool angle_in_sector(double angle, double start, double end)
{
  if (start <= end) {
    return start <= angle && angle <= end;
  }
  return angle >= start || angle <= end;
}

struct Sector
{
  std::size_t first;               // index of the first kept beam
  std::size_t last;                // index of the last kept beam
  std::vector<std::size_t> gaps;   // removed beams inside [first, last], relative to first
};

// Compute what to keep from a scan with the given geometry.
// std::nullopt if no beam is kept.
inline std::optional<Sector> compute_sector(
  std::size_t num_readings, double angle_min, double angle_increment,
  double start, double end, bool invert)
{
  start = normalize_angle(start);
  end = normalize_angle(end);

  std::vector<bool> keep(num_readings);
  bool any = false;
  for (std::size_t i = 0; i < num_readings; ++i) {
    const double angle = normalize_angle(angle_min + static_cast<double>(i) * angle_increment);
    keep[i] = angle_in_sector(angle, start, end) != invert;
    any = any || keep[i];
  }
  if (!any) {
    return std::nullopt;
  }

  Sector s{};
  s.first = 0;
  while (!keep[s.first]) {
    ++s.first;
  }
  s.last = num_readings - 1;
  while (!keep[s.last]) {
    --s.last;
  }
  for (std::size_t i = s.first; i <= s.last; ++i) {
    if (!keep[i]) {
      s.gaps.push_back(i - s.first);
    }
  }
  return s;
}

}  // namespace lidar_filter_cpp

#endif  // LIDAR_FILTER_CPP__SECTOR_HPP_
