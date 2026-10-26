// Заглушка PCA9685: запоминает последние значения каналов и считает записи
#pragma once
#include "Arduino.h"
class Adafruit_PWMServoDriver {
public:
  Adafruit_PWMServoDriver(uint8_t = 0x40) {}
  bool begin() { return true; }
  void setPWMFreq(float) {}
  void reset() {}
  void setPin(uint8_t num, uint16_t val, bool) { pins[num] = val; ++writes; g_micros += 150; }   // запись регистра ~0.15 мс на 400 кГц
  uint16_t pins[16] = {0};
  long writes = 0;
};
