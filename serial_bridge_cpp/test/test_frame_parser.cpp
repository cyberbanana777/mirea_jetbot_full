// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "serial_bridge_cpp/frame_parser.hpp"

using serial_bridge_cpp::FrameParser;
using serial_bridge_cpp::parse_fields;

static std::vector<std::string> feed_all(FrameParser & p, const std::string & data)
{
  std::vector<std::string> out;
  p.feed(data.data(), data.size(), [&](const std::string & f) {out.push_back(f);});
  return out;
}

TEST(FrameParser, SingleFrameWithCrlf)
{
  FrameParser p;
  auto frames = feed_all(p, "$1;0.1;0.2;#\r\n");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$1;0.1;0.2;#");
}

TEST(FrameParser, FrameSplitBetweenReads)
{
  FrameParser p;
  EXPECT_TRUE(feed_all(p, "$1;0.").empty());
  EXPECT_TRUE(feed_all(p, "123;4").empty());
  auto frames = feed_all(p, "5;#\r\n$2;");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$1;0.123;45;#");
  frames = feed_all(p, "1;#");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$2;1;#");
}

TEST(FrameParser, SeveralFramesInOneRead)
{
  FrameParser p;
  auto frames = feed_all(p, "$1;1;#\r\n$1;2;#\r\n$2;3;#");
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[2], "$2;3;#");
}

TEST(FrameParser, GarbageAndBrokenFrame)
{
  FrameParser p;
  // text from the ESP (e.g. "INA219 reinitialized") is ignored,
  // a broken frame without '#' is dropped when a new '$' arrives
  auto frames = feed_all(p, "INA219 reinitialized\r\n$1;0.5$1;0.7;#");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$1;0.7;#");
}

TEST(FrameParser, TooLongFrameDropped)
{
  FrameParser p(16);
  auto frames = feed_all(p, "$1;" + std::string(100, '1') + ";#$1;2;#");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$1;2;#");
}

TEST(ParseFields, OdometryFrame)
{
  std::vector<double> f;
  ASSERT_TRUE(parse_fields(
      "$1;0.12345;-1.50000;3.14159;0.20000;-0.10000;0.15000;0.25000;#", f));
  ASSERT_EQ(f.size(), 8u);
  EXPECT_DOUBLE_EQ(f[0], 1.0);
  EXPECT_DOUBLE_EQ(f[2], -1.5);
  EXPECT_DOUBLE_EQ(f[7], 0.25);
}

TEST(ParseFields, PidFrameTrailingSeparator)
{
  std::vector<double> f;
  ASSERT_TRUE(parse_fields("$2;0.85000;1.50000;0.05000;0.80000;1.50000;0.04000;#", f));
  EXPECT_EQ(f.size(), 7u);
}

TEST(ParseFields, Malformed)
{
  std::vector<double> f;
  EXPECT_FALSE(parse_fields("1;2;#", f));
  EXPECT_FALSE(parse_fields("$1;2;", f));
  EXPECT_FALSE(parse_fields("$1;abc;#", f));
  EXPECT_FALSE(parse_fields("$1;0.5x;#", f));
  EXPECT_FALSE(parse_fields("$#", f));
  EXPECT_FALSE(parse_fields("$;;#", f));
}
