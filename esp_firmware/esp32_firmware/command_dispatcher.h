#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H
#include "command_parser.h"
#include "motor_controller.h"
#include "flash_storage.h"
#include "speed_estimator.h"

// Non-owning dependencies. Control math and persistent storage remain unchanged.
struct CommandContext {
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
  bool& telemetry_on;
  uint8_t& telemetry_div;
};

void dispatchCommand(const ParsedCommand& cmd, CommandContext& ctx);
#endif
