#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>
#include "motor_controller.h"

// Non-owning references: original application state and watchdog are unchanged.
struct SafetyManagerContext {
  MotorController &controller;
  unsigned long &lastMotionMs;
  const unsigned long commandTimeoutMs;
  uint32_t &watchdogStops;
  bool &watchdogLatched;
  bool &telemetryEnabled;
  bool &inaOk;
  bool &pwmOk;
  float &busVoltage;
  const float minVoltage;
};

class SafetyManager {
public:
  explicit SafetyManager(SafetyManagerContext &context) : ctx_(context) {}
  void check();
private:
  SafetyManagerContext &ctx_;
};

#endif
