// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
//
// Тесты констант и форматтеров протокола v2 (serial_bridge_cpp/protocol.hpp).
// Форматтеры используются узлом для отправки команд прошивке, поэтому
// расхождение с прошивкой (esp_firmware/esp32_firmware/command_parser.cpp)
// ломает управление роботом.

#include <gtest/gtest.h>

#include <string>

#include "serial_bridge_cpp/protocol.hpp"

using serial_bridge_cpp::format_stop;
using serial_bridge_cpp::format_twist;
using serial_bridge_cpp::is_known_unmapped;
using serial_bridge_cpp::is_motion_id;
using serial_bridge_cpp::raw_frame_id;

TEST(Protocol, FormatTwist)
{
  EXPECT_EQ(format_twist(0.0, 0.0), "$101;0.000;0.000;#");
  EXPECT_EQ(format_twist(0.25, -0.5), "$101;0.250;-0.500;#");
  EXPECT_EQ(format_twist(1.0, 2.0), "$101;1.000;2.000;#");
}

TEST(Protocol, FormatStop)
{
  EXPECT_EQ(format_stop(), "$103;#");
}

TEST(Protocol, MotionIds)
{
  // Только эти три ID продлевают watchdog движения в прошивке.
  EXPECT_TRUE(is_motion_id(serial_bridge_cpp::CMD_SET_TWIST));
  EXPECT_TRUE(is_motion_id(serial_bridge_cpp::CMD_SET_WHEEL_SPEEDS));
  EXPECT_TRUE(is_motion_id(serial_bridge_cpp::CMD_SET_DUTY));

  // STOP останавливает, но watchdog не продлевает.
  EXPECT_FALSE(is_motion_id(serial_bridge_cpp::CMD_STOP));
  EXPECT_FALSE(is_motion_id(serial_bridge_cpp::MSG_ODOMETRY));
  EXPECT_FALSE(is_motion_id(serial_bridge_cpp::MSG_PID));
  EXPECT_FALSE(is_motion_id(1));   // ID v1
  EXPECT_FALSE(is_motion_id(2));   // ID v1
}

TEST(Protocol, RawFrameId)
{
  EXPECT_EQ(raw_frame_id("$101;0.1;0.2;#"), 101);
  EXPECT_EQ(raw_frame_id("$102;0.1;0.2;#"), 102);
  EXPECT_EQ(raw_frame_id("$103;#"), 103);
  EXPECT_EQ(raw_frame_id("$104;0.1;0.2;#"), 104);
  EXPECT_EQ(raw_frame_id("$301;1;2;3;4;5;6;7;#"), 301);
  EXPECT_EQ(raw_frame_id("$202;#"), 202);

  EXPECT_EQ(raw_frame_id(""), -1);
  EXPECT_EQ(raw_frame_id("текст"), -1);
  EXPECT_EQ(raw_frame_id("$abc;1;#"), -1);
  EXPECT_EQ(raw_frame_id("$;1;#"), -1);
  EXPECT_EQ(raw_frame_id("101;1;#"), -1);   // нет ведущего '$'
}

TEST(Protocol, KnownUnmappedFrames)
{
  // Кадры, которые прошивка шлёт, но мост пока не публикует в ROS:
  // они не должны попадать в лог как "unexpected".
  EXPECT_TRUE(is_known_unmapped(203));
  EXPECT_TRUE(is_known_unmapped(205));
  EXPECT_TRUE(is_known_unmapped(207));
  EXPECT_TRUE(is_known_unmapped(211));
  EXPECT_TRUE(is_known_unmapped(401));
  EXPECT_TRUE(is_known_unmapped(402));
  EXPECT_TRUE(is_known_unmapped(404));
  EXPECT_TRUE(is_known_unmapped(405));
  EXPECT_TRUE(is_known_unmapped(501));
  EXPECT_TRUE(is_known_unmapped(502));
  EXPECT_TRUE(is_known_unmapped(503));

  // Одометрия и PID публикуются — они не «непривязанные».
  EXPECT_FALSE(is_known_unmapped(serial_bridge_cpp::MSG_ODOMETRY));
  EXPECT_FALSE(is_known_unmapped(serial_bridge_cpp::MSG_PID));
  EXPECT_FALSE(is_known_unmapped(999));
}
