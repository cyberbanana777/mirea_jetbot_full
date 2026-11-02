// Минимальные заглушки Arduino/ESP32 API - только чтобы собрать код прошивки на ПК
// (проверка синтаксиса и тест логики). Не для прошивки платы.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#define IRAM_ATTR
#define INPUT 0
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define CHANGE 3
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
// Макросы из настоящего Arduino.h (ESP32 core 3.x): без них хост-тест не заметит конфликт имён
// (так ControlMode::PI собирался на ПК и ломался в Arduino IDE)
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352
#define SERIAL 0x0
#define DISPLAY 0x1
#define LSBFIRST 0
#define MSBFIRST 1

// Время задаёт тест
extern unsigned long g_micros;
inline unsigned long micros() { return g_micros; }
inline unsigned long millis() { return g_micros / 1000; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
extern int g_pins[64];
inline int digitalRead(int p) { return g_pins[p]; }
inline void attachInterrupt(int, void (*)(void), int) {}

typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
#define portENTER_CRITICAL_ISR(m) ((void)(m))
#define portEXIT_CRITICAL_ISR(m) ((void)(m))

class String {
public:
  std::string s;
  String() {}
  String(const char *c) : s(c ? c : "") {}
  const char *c_str() const { return s.c_str(); }
  size_t length() const { return s.size(); }
  String substring(size_t a, size_t b) const { return String(s.substr(a, b > a ? b - a : 0).c_str()); }
  String substring(size_t a) const { return String(s.substr(a).c_str()); }
  String &operator+=(const char *c) { s += c; return *this; }
  String &operator=(const char *c) { s = c ? c : ""; return *this; }
};

class Stream {
public:
  virtual ~Stream() {}
  virtual int available() { return 0; }
  virtual int read() { return -1; }
};

class HardwareSerial : public Stream {
public:
  std::string rx, tx;   // тест кладёт входящие байты в rx и читает отправленное из tx
  size_t rxPos = 0;
  void begin(unsigned long) {}
  int available() override { return (int)(rx.size() - rxPos); }
  int read() override { return rxPos < rx.size() ? (unsigned char)rx[rxPos++] : -1; }
  int availableForWrite() { return 128; }
  size_t write(const char *p, size_t n) { tx.append(p, n); return n; }
  size_t write(const uint8_t *p, size_t n) { tx.append((const char *)p, n); return n; }
  void print(const char *) {}
  void print(const String &) {}
  void print(float) {}
  void println(const char *) {}
  void println(const String &) {}
};
extern HardwareSerial Serial;
extern HardwareSerial Serial1;

struct EspClass { void restart() {} };
extern EspClass ESP;
