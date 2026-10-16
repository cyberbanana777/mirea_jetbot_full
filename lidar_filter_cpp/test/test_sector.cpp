// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>

#include "lidar_filter_cpp/sector.hpp"

using lidar_filter_cpp::compute_sector;

constexpr double DEG = M_PI / 180.0;

// Scan like sllidar with angle_compensate: 360 beams from -pi with 1 deg step
constexpr std::size_t N = 360;
const double ANGLE_MIN = -M_PI;
const double INC = 2.0 * M_PI / (N - 1);

TEST(Sector, DefaultContiguousSectorIsCropped)
{
  auto s = compute_sector(N, ANGLE_MIN, INC, -135 * DEG, 135 * DEG, false);
  ASSERT_TRUE(s);
  EXPECT_TRUE(s->gaps.empty());
  EXPECT_GE(ANGLE_MIN + s->first * INC, -135 * DEG);
  EXPECT_LT(ANGLE_MIN + (s->first - 1) * INC, -135 * DEG);
  EXPECT_LE(ANGLE_MIN + s->last * INC, 135 * DEG);
  EXPECT_GT(ANGLE_MIN + (s->last + 1) * INC, 135 * DEG);
}

TEST(Sector, InvertedSectorKeepsGeometry)
{
  // keep everything except the front: the kept beams are at both ends,
  // the middle must become a gap, not be glued together
  auto s = compute_sector(N, ANGLE_MIN, INC, -45 * DEG, 45 * DEG, true);
  ASSERT_TRUE(s);
  EXPECT_EQ(s->first, 0u);
  EXPECT_EQ(s->last, N - 1);
  EXPECT_FALSE(s->gaps.empty());
  for (auto g : s->gaps) {
    const double a = ANGLE_MIN + (s->first + g) * INC;
    EXPECT_GE(a, -45 * DEG);
    EXPECT_LE(a, 45 * DEG);
  }
}

TEST(Sector, SectorAcrossPi)
{
  // rear sector 135..-135 deg goes through +-pi
  auto s = compute_sector(N, ANGLE_MIN, INC, 135 * DEG, -135 * DEG, false);
  ASSERT_TRUE(s);
  EXPECT_EQ(s->first, 0u);
  EXPECT_EQ(s->last, N - 1);
  EXPECT_FALSE(s->gaps.empty());
}

TEST(Sector, AnglesOutsidePiAreNormalized)
{
  // scan from 0 to 2*pi (another driver convention)
  auto s = compute_sector(N, 0.0, INC, -10 * DEG, 10 * DEG, false);
  ASSERT_TRUE(s);
  EXPECT_EQ(s->first, 0u);
  EXPECT_EQ(s->last, N - 1);
}

TEST(Sector, EmptySector)
{
  // sector narrower than the beam step between two beams
  auto s = compute_sector(N, ANGLE_MIN, INC, 0.1 * DEG, 0.2 * DEG, false);
  EXPECT_FALSE(s);
}

TEST(Sector, NegativeIncrement)
{
  auto s = compute_sector(N, M_PI, -INC, -90 * DEG, 90 * DEG, false);
  ASSERT_TRUE(s);
  EXPECT_TRUE(s->gaps.empty());
  EXPECT_NEAR(M_PI - s->first * INC, 90 * DEG, INC);
  EXPECT_NEAR(M_PI - s->last * INC, -90 * DEG, INC);
}
