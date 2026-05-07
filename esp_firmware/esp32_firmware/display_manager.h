#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <GyverOLED.h>

class DisplayManager {
public:
  // interval    – период обычного обновления экрана, мс (по умолчанию 200)
  // blinkPeriod – период переключения страниц при низком заряде, мс (по умолчанию 500)
  DisplayManager(unsigned long interval = 200, unsigned long blinkPeriod = 1500);

  // Инициализация дисплея, вывод стартового сообщения
  bool begin(const char* version);

  // Попытаться переинициализировать дисплей после сбоя I²C
  bool reinitialize();

  // Обновляет содержимое экрана в соответствии с текущим состоянием батареи.
  // Вызывайте в каждом цикле loop().
  void update(const String& ssid, const String& ip, const String& password,
              float busVoltage, float current, int batteryPercent);

private:
  GyverOLED<SSD1306_128x32, OLED_BUFFER> _oled;
  unsigned long _interval;
  unsigned long _blinkPeriod;
  unsigned long _lastUpdate;
  unsigned long _lastBlink;
  bool _lowBatteryMode;
  bool _showWarning;  // true = показывать предупреждение

  // Отрисовка одной из двух страниц в зависимости от _showWarning
  void _render(const String& ssid, const String& ip, const String& password,
               float busVoltage, float current, int batteryPercent);
};

#endif