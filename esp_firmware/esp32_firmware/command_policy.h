#ifndef COMMAND_POLICY_H
#define COMMAND_POLICY_H
#include "command_parser.h"
#include <math.h>

// Input validation and watchdog policy: pure logic, no hardware dependency.
struct CommandPolicy {
  static bool validPid(float kpL, float kiL, float kdL, float kpR, float kiR, float kdR) {
    const float x[6] = {kpL, kiL, kdL, kpR, kiR, kdR};
    for (float v : x) if (!isfinite(v) || v < 0.0f || v > 1000.0f) return false;
    return true;
  }
  static bool validRequestId(float x) {
    return isfinite(x) && x >= 1.0f && x <= 65535.0f && floorf(x) == x;
  }
  static bool validTransactionalPid(const ParsedCommand& c) {
    return c.count == 7 && validRequestId(c.args[0]) &&
           validPid(c.args[1], c.args[2], c.args[3], c.args[4], c.args[5], c.args[6]);
  }
  static bool refreshesMotionWatchdog(Command t) {
    return t == Command::SET_TWIST || t == Command::SET_WHEEL_SPEEDS ||
           t == Command::SET_DUTY;
  }
  static bool validMotionValues(const ParsedCommand& c) {
    if (!refreshesMotionWatchdog(c.type) || c.count != 2) return false;
    return isfinite(c.args[0]) && isfinite(c.args[1]);
  }
  static bool validShape(const ParsedCommand& c) {
    switch (c.type) {
      case Command::REQUEST_PID_TX: return c.count == 1 && validRequestId(c.args[0]);
      case Command::SET_PID_TX: return c.count == 7 && validRequestId(c.args[0]);
      case Command::SET_TWIST: case Command::SET_WHEEL_SPEEDS:
      case Command::SET_DUTY: return c.count == 2;
      case Command::SET_PID_TEMP: case Command::SET_PID_FLASH: return c.count == 6;
      case Command::SET_FF: return c.count == 4 || c.count == 5;
      case Command::SET_ADRC: return c.count >= 4 && c.count <= 6;
      case Command::SET_REF_FILTER: case Command::SET_CTRL_MODE:
      case Command::SET_SPEED_WINDOW: return c.count == 1 || c.count == 2 ||
          (c.type == Command::SET_SPEED_WINDOW && c.count == 0);
      case Command::SET_TELEMETRY: return c.count == 1;
      case Command::STOP: case Command::REQUEST_PID: case Command::REQUEST_FF:
      case Command::REQUEST_ADRC: case Command::REQUEST_DIAGNOSTICS:
      case Command::REQUEST_INFO: case Command::REQUEST_I2C_DIAGNOSTICS: return c.count == 0;
      default: return false;
    }
  }
};
#endif
