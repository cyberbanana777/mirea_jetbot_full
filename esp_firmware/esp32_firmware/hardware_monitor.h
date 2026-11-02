#pragma once

#include <Arduino.h>
#include <INA219.h>
#include "motor_controller.h"
#include "display_manager.h"

// Non-owning references: all objects and flags live in esp32_firmware.ino.
struct HardwareMonitorContext {
  INA219 &ina;
  MotorController &controller;
  DisplayManager &display;
  bool &inaOk;
  bool &pwmOk;
  bool &oledOk;
  uint8_t &inaErrors;
  uint8_t &pwmErrors;
  uint8_t &oledErrors;
  float &busVoltage;
  float &shuntVoltage;
  float &current;
  int &batteryPercent;
  const float minVoltage;
  const float maxVoltage;
  const uint8_t maxI2cErrors;
  const uint32_t i2cClockHz;
  const char *version;
  bool (*probe)(uint8_t);
};

class HardwareMonitor {
public:
  explicit HardwareMonitor(HardwareMonitorContext &ctx) : ctx_(ctx) {}
  void samplePower();
  void checkI2CDevices();
private:
  HardwareMonitorContext &ctx_;
};
