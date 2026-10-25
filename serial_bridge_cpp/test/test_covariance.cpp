// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "serial_bridge_cpp/covariance.hpp"

using serial_bridge_cpp::diagonal_to_covariance;

TEST(Covariance, DiagonalIsPlacedRowMajor)
{
  auto c = diagonal_to_covariance({1, 2, 3, 4, 5, 6}, "p");
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_DOUBLE_EQ(c[i * 6 + j], i == j ? i + 1.0 : 0.0);
    }
  }
}

TEST(Covariance, DefaultsAreValid)
{
  auto p = diagonal_to_covariance(serial_bridge_cpp::default_pose_covariance_diagonal(), "p");
  auto t = diagonal_to_covariance(serial_bridge_cpp::default_twist_covariance_diagonal(), "t");
  EXPECT_GT(p[0], 0.0);
  EXPECT_DOUBLE_EQ(p[14], serial_bridge_cpp::UNMEASURED_VARIANCE);  // z
  EXPECT_GT(t[35], 0.0);                                            // wz
}

TEST(Covariance, RejectsBadInput)
{
  EXPECT_THROW(diagonal_to_covariance({1, 2, 3}, "p"), std::invalid_argument);
  EXPECT_THROW(diagonal_to_covariance({1, 2, 3, 4, 5, -1}, "p"), std::invalid_argument);
  EXPECT_THROW(diagonal_to_covariance({1, 2, 3, 4, 5, NAN}, "p"), std::invalid_argument);
}
