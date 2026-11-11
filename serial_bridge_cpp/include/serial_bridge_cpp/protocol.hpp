// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// Protocol v2 of the ESP32 firmware: message IDs, field layouts and the
// frame formats the bridge produces.
// Pure C++ without ROS, so the contract with the firmware can be unit-tested
// on the host (see esp_firmware/tests/test_end_to_end_v2.cpp).
//
// The authority for the protocol is esp_firmware/PROTOCOL.md.

#ifndef SERIAL_BRIDGE_CPP__PROTOCOL_HPP_
#define SERIAL_BRIDGE_CPP__PROTOCOL_HPP_

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace serial_bridge_cpp
{

// --- Commands (Jetson -> ESP32) -------------------------------------------
// The firmware rejects every v1 ID ($1, $2, ...) as unknown, so the bridge
// must use the v2 ones.
constexpr int CMD_SET_TWIST = 101;          // $101;linear;angular;#
constexpr int CMD_SET_WHEEL_SPEEDS = 102;   // $102;left;right;#
constexpr int CMD_STOP = 103;               // $103;#
constexpr int CMD_SET_DUTY = 104;           // $104;duty_left;duty_right;#

// --- Responses (ESP32 -> Jetson) ------------------------------------------
constexpr int MSG_ODOMETRY = 301;   // $301;x;y;theta;v;w;v_left;v_right;#
constexpr int MSG_PID = 202;        // $202;KpL;KiL;KdL;KpR;KiR;KdR;#

// Number of fields including the message ID.
constexpr std::size_t ODOMETRY_FIELDS = 8;  // type + 7 values
constexpr std::size_t PID_FIELDS = 7;       // type + 6 values

// Only these commands refresh the motion watchdog on the ESP32 side.
// The stop command ($103) does not: it stops the robot without keeping the
// watchdog alive, so a missing /cmd_vel still stops the motors.
inline bool is_motion_id(int id)
{
  return id == CMD_SET_TWIST || id == CMD_SET_WHEEL_SPEEDS || id == CMD_SET_DUTY;
}

// Frames the ESP32 sends, but the bridge does not map to a ROS topic yet.
// They are not errors, so they are not reported as unexpected frames:
// $401 is sent at 50-100 Hz and would flood the log.
inline bool is_known_unmapped(int type)
{
  switch (type) {
    case 203:   // ADRC state
    case 205:   // feedforward state
    case 207:   // reference filter state
    case 211:   // transactional PID read response
    case 401:   // diagnostic telemetry stream
    case 402:   // diagnostic snapshot
    case 404:   // speed estimator window
    case 405:   // read-only I2C snapshot
    case 501:   // protocol version
    case 502:   // ACK
    case 503:   // NACK
      return true;
    default:
      return false;
  }
}

// ID of a raw frame ("$101;...;#"), or -1 if the frame has no numeric ID.
// Used to tell whether a raw command from serial/cmd drives the motors.
inline int raw_frame_id(const std::string & frame)
{
  if (frame.empty() || frame[0] != '$') {
    return -1;
  }
  std::size_t i = 1;
  while (i < frame.size() && frame[i] >= '0' && frame[i] <= '9') {
    ++i;
  }
  if (i == 1 || i >= frame.size() || frame[i] != ';') {
    return -1;
  }
  return std::atoi(frame.c_str() + 1);
}

// Frame the bridge sends for /cmd_vel.
inline std::string format_twist(double linear, double angular)
{
  char buf[64];
  const int len = std::snprintf(
    buf, sizeof(buf), "$101;%.3f;%.3f;#", linear, angular);
  if (len <= 0 || len >= static_cast<int>(sizeof(buf))) {
    return std::string();
  }
  return std::string(buf, static_cast<std::size_t>(len));
}

// Frame the bridge sends when /cmd_vel stayed silent for too long.
inline std::string format_stop()
{
  return "$103;#";
}

}  // namespace serial_bridge_cpp

#endif  // SERIAL_BRIDGE_CPP__PROTOCOL_HPP_
