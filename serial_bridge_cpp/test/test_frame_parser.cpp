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

static void feed_both(
  FrameParser & p, const std::string & data, std::vector<std::string> & frames,
  std::vector<std::string> & lines)
{
  p.feed(
    data.data(), data.size(),
    [&](const std::string & f) {frames.push_back(f);},
    [&](const std::string & l) {lines.push_back(l);});
}

TEST(FrameParserText, TextLinesBetweenFrames)
{
  FrameParser p;
  std::vector<std::string> frames, lines;
  feed_both(p, "$1;1;#\r\nINA219 reinit failed\r\n$1;2;#\r\nWiFi retry, SSID = lab\r\n",
    frames, lines);
  ASSERT_EQ(frames.size(), 2u);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "INA219 reinit failed");
  EXPECT_EQ(lines[1], "WiFi retry, SSID = lab");
}

TEST(FrameParserText, LineSplitBetweenReadsAndAroundFrame)
{
  FrameParser p;
  std::vector<std::string> frames, lines;
  feed_both(p, "OTA er", frames, lines);
  EXPECT_TRUE(lines.empty());
  feed_both(p, "ror[2]: $1;5;#\r\nConnect Failed\r", frames, lines);
  EXPECT_TRUE(lines.empty());
  feed_both(p, "\n", frames, lines);
  ASSERT_EQ(frames.size(), 1u);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "OTA error[2]: Connect Failed");
}

TEST(FrameParserText, EmptyLinesSkippedAndGarbageSanitized)
{
  FrameParser p;
  std::vector<std::string> frames, lines;
  feed_both(p, std::string("\r\n\r\n  \r\n\x01\xff boot\r\n", 17), frames, lines);
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "?? boot");
}

TEST(FrameParserText, LongLineEmittedInParts)
{
  FrameParser p(16);
  std::vector<std::string> frames, lines;
  feed_both(p, std::string(40, 'a') + "\n", frames, lines);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[0].size(), 16u);
  EXPECT_EQ(lines[2].size(), 8u);
}

TEST(FrameParserText, FramesOnlyOverloadIgnoresText)
{
  FrameParser p;
  auto frames = feed_all(p, "hello\r\n$1;1;#\r\n");
  ASSERT_EQ(frames.size(), 1u);
}
