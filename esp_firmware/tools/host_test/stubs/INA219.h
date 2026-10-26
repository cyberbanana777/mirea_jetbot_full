#pragma once
#include "Arduino.h"
class INA219 {
public:
  INA219(uint8_t) {}
  bool begin() { return true; }
  float getBusVoltage() { g_micros += 500; return 12.0f; }
  float getShuntVoltage_mV() { g_micros += 500; return 0.0f; }
};
