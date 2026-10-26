#pragma once
#include "Arduino.h"
class TwoWire {
public:
  bool begin() { return true; }
  void setTimeOut(uint16_t) {}
  void setClock(uint32_t hz) { clock = hz; }
  void beginTransmission(uint8_t) {}
  uint8_t endTransmission() { g_micros += 300; return 0; }   // ~0.3 мс на транзакцию (400 кГц)
  uint32_t clock = 100000;
};
extern TwoWire Wire;
