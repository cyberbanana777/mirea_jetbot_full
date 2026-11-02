#pragma once
#include <Arduino.h>
#include <INA219.h>
#include "encoders.h"
#include "errors.h"
#include "motor_controller.h"
#include "flash_storage.h"
#include "display_manager.h"
#include "speed_estimator.h"

// Non-owning references to objects and timers declared in the sketch.
// setup() order is preserved; this class performs startup only.
struct StartupContext {
  ErrorManager &error_manager;
  Encoders &encoders;
  MotorController &controller;
  INA219 &INA;
  DisplayManager &display;
  FlashStorageManager &flashStorage;
  unsigned long &emergency_timer;
  MTSpeedEstimator &speedL;
  MTSpeedEstimator &speedR;
  unsigned long &odom_tmr;
  unsigned long &control_next_us;
  bool &inaOk;
  bool &pwmOk;
  bool &oledOk;
  const char *version;
  bool (*probe)(uint8_t);
};

class StartupService {
public:
  explicit StartupService(StartupContext &ctx) : ctx_(ctx) {}
  void begin();
private:
  StartupContext &ctx_;
};
