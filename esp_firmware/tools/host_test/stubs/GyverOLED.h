#pragma once
#include "Arduino.h"
#define SSD1306_128x32 0
#define OLED_BUFFER 1
extern void (*g_onOledChunk)();   // тест может подписаться на каждую отправку куска кадра
template <int T, int B>
class GyverOLED {
public:
  void init() {}
  void clear() {}
  void home() {}
  void setScale(int) {}
  template <typename X> void print(X) {}
  template <typename X> void println(X) {}
  void update() {}
  void update(int, int, int, int) { ++chunks; if (g_onOledChunk) g_onOledChunk(); }
  int chunks = 0;
};
