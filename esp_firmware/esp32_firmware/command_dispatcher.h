#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include "command_parser.h"
#include "motor_controller.h"
#include "flash_storage.h"
#include "speed_estimator.h"

// Each service receives only its own non-owning dependencies.
struct MotionContext {
  MotorController& controller;
  bool& telemetry_on;
  uint8_t& telemetry_div;
};

struct ConfigContext {
  MotorController& controller;
  FlashStorageManager& flashStorage;
  MTSpeedEstimator& speedL;
  MTSpeedEstimator& speedR;
  float& KP_L;
  float& KI_L;
  float& KD_L;
  float& KP_R;
  float& KI_R;
  float& KD_R;
  float& FF_GAIN_L;
  float& FF_DEADZONE_L;
  float& FF_GAIN_R;
  float& FF_DEADZONE_R;
  float& REF_TAU_S;
  float& ADRC_WC;
  float& ADRC_WO;
  float& ADRC_B0_L;
  float& ADRC_B0_R;
  float& ADRC_FF;
  int& CTRL_MODE;
  uint32_t& accepted_commands;
  uint32_t& rejected_commands;
};

struct DiagnosticsContext {
  MotorController& controller;
  uint32_t& accepted_commands;
  uint32_t& rejected_commands;
  uint32_t& watchdog_stop_count;
  bool& inaOk;
  bool& pwmOk;
};

struct CommandContext {
  MotionContext motion;
  ConfigContext config;
  DiagnosticsContext diagnostics;
};

void dispatchCommand(const ParsedCommand& cmd, CommandContext& ctx);
#endif
