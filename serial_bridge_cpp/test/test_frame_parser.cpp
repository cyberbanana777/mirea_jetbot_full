// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
//
// Тесты разбора кадров моста на протоколе v2.
// Форматы кадров — как в esp_firmware/esp32_firmware/protocol_wrapper.cpp:
//   $301;x;y;theta;v;w;v_left;v_right;#  — одометрия (8 полей)
//   $202;KpL;KiL;KdL;KpR;KiR;KdR;#        — коэффициенты PID (7 полей)

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
  auto frames = feed_all(p, "$301;0.1;0.2;#\r\n");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$301;0.1;0.2;#");
}

TEST(FrameParser, FrameSplitBetweenReads)
{
  FrameParser p;
  EXPECT_TRUE(feed_all(p, "$301;0.").empty());
  EXPECT_TRUE(feed_all(p, "123;4").empty());
  auto frames = feed_all(p, "5;#\r\n$202;");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$301;0.123;45;#");
  frames = feed_all(p, "1;#");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$202;1;#");
}

TEST(FrameParser, SeveralFramesInOneRead)
{
  FrameParser p;
  auto frames = feed_all(p, "$301;1;#\r\n$301;2;#\r\n$202;3;#");
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[2], "$202;3;#");
}

TEST(FrameParser, GarbageAndBrokenFrame)
{
  FrameParser p;
  // text from the ESP (e.g. "INA219 reinitialized") is ignored,
  // a broken frame without '#' is dropped when a new '$' arrives
  auto frames = feed_all(p, "INA219 reinitialized\r\n$301;0.5$301;0.7;#");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$301;0.7;#");
}

TEST(FrameParser, TooLongFrameDropped)
{
  FrameParser p(16);
  auto frames = feed_all(p, "$301;" + std::string(100, '1') + ";#$301;2;#");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], "$301;2;#");
}

TEST(ParseFields, OdometryFrame)
{
  std::vector<double> f;
  ASSERT_TRUE(parse_fields(
      "$301;0.12345;-1.50000;3.14159;0.20000;-0.10000;0.15000;0.25000;#", f));
  ASSERT_EQ(f.size(), 8u);
  EXPECT_DOUBLE_EQ(f[0], 301.0);
  EXPECT_DOUBLE_EQ(f[2], -1.5);
  EXPECT_DOUBLE_EQ(f[7], 0.25);
}

TEST(ParseFields, PidFrame)
{
  std::vector<double> f;
  ASSERT_TRUE(parse_fields("$202;0.85000;1.50000;0.05000;0.80000;1.50000;0.04000;#", f));
  EXPECT_EQ(f.size(), 7u);
  EXPECT_DOUBLE_EQ(f[0], 202.0);
}

TEST(ParseFields, NonFiniteRejected)
{
  // Значения nan/inf не должны попасть в одометрию и ПИД.
  std::vector<double> f;
  EXPECT_FALSE(parse_fields("$301;nan;#", f));
  EXPECT_FALSE(parse_fields("$301;inf;#", f));
  EXPECT_FALSE(parse_fields("$301;-inf;#", f));
  EXPECT_FALSE(parse_fields("$202;1;2;3;nan;5;6;#", f));
}

TEST(ParseFields, Malformed)
{
  std::vector<double> f;
  EXPECT_FALSE(parse_fields("1;2;#", f));
  EXPECT_FALSE(parse_fields("$301;2;", f));
  EXPECT_FALSE(parse_fields("$301;abc;#", f));
  EXPECT_FALSE(parse_fields("$301;0.5x;#", f));
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
  feed_both(p, "$301;1;#\r\nINA219 reinit failed\r\n$301;2;#\r\nWiFi retry, SSID = lab\r\n",
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
  feed_both(p, "ror[2]: $301;5;#\r\nConnect Failed\r", frames, lines);
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
  auto frames = feed_all(p, "hello\r\n$301;1;#\r\n");
  ASSERT_EQ(frames.size(), 1u);
}
