#pragma once

#include <Arduino.h>
#include <INA219.h>
#include "motor_controller.h"
#include "display_manager.h"

// Diagnostics only. These counters do not influence motor safety decisions.
enum class I2CDeviceState : uint8_t {
  kNotDetected = 0,
  kHealthy = 1,
  kFault = 2,
  kRecovered = 3
};

struct I2CDeviceDiagnostics {
  I2CDeviceState state = I2CDeviceState::kNotDetected;
  uint32_t probeFailures = 0;
  uint32_t recoveryAttempts = 0;
  uint32_t successfulRecoveries = 0;
};

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
  I2CDeviceDiagnostics &inaDiagnostics;
  I2CDeviceDiagnostics &pwmDiagnostics;
  I2CDeviceDiagnostics &oledDiagnostics;
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
  void observeStartup();
  void samplePower();
  void checkI2CDevices();
private:
  static void recordProbe(I2CDeviceDiagnostics &diag, bool responds);
  static void recordRecoveryAttempt(I2CDeviceDiagnostics &diag);
  static void recordRecoveryResult(I2CDeviceDiagnostics &diag, bool successful);
  HardwareMonitorContext &ctx_;
};
