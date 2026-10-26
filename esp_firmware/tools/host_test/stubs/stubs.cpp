#include "Arduino.h"
#include "Wire.h"
unsigned long g_micros = 0;
int g_pins[64] = {0};
HardwareSerial Serial;
HardwareSerial Serial1;
EspClass ESP;
TwoWire Wire;
void (*g_onOledChunk)() = nullptr;
