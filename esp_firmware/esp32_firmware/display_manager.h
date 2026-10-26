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
  bool reinitialize(const char* version);

  // Обновляет содержимое экрана в соответствии с текущим состоянием батареи.
  // Вызывайте в каждом цикле loop().
  // Кадр рисуется в буфер (RAM, шину не занимает), а в дисплей уходит по кускам:
  // не больше одного куска (CHUNK_COLS байт) за вызов, и только если allowBus = true.
  // Так обновление экрана не растягивается на десятки миллисекунд и не срывает
  // такты контура скорости: вызывающий разрешает шину, когда до такта далеко.
  void update(const String& ssid, const String& ip, const String& password,
              float busVoltage, float current, int batteryPercent,
              bool allowBus = true);

private:
  GyverOLED<SSD1306_128x32, OLED_BUFFER> _oled;
  unsigned long _interval;
  unsigned long _blinkPeriod;
  unsigned long _lastUpdate;
  unsigned long _lastBlink;
  bool _lowBatteryMode;
  bool _showWarning;  // true = показывать предупреждение

  // Порционная отправка кадра: 128x32 = 4 страницы по 128 столбцов
  static const uint8_t CHUNK_COLS = 64;  // столбцов (байт) за один вызов; 64 байта ~ 1.7 мс на 400 кГц
  static const uint8_t PAGES = 4;
  bool _flushing = false;
  uint8_t _flushPage = 0;
  uint8_t _flushCol = 0;
  void _flushStep();

  // Отрисовка одной из двух страниц в зависимости от _showWarning
  void _render(const String& ssid, const String& ip, const String& password,
               float busVoltage, float current, int batteryPercent);
};

#endif