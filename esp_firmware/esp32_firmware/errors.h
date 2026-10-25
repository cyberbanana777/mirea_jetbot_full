#include <sys/_stdint.h>
#ifndef ERRORS_H
#define ERRORS_H

#include <Arduino.h>

/**
 * Индикация ошибок светодиодом D2 (ERROR_LED_PIN):
 *  - не горит                      - всё в порядке;
 *  - мигает часто (4 раза в секунду) - ошибка во время работы (setFault),
 *                                    программа продолжает работать;
 *  - мигает раз в секунду ~10 с,   - ошибка инициализации (raise_error),
 *    затем перезагрузка              текст ошибки выводится в Serial.
 */
class ErrorManager{
public:
  // Настроить вывод светодиода и погасить его
  void begin(uint8_t LedPin);
  // Функция для аварийной остановки с выводом текста ошибки
  void raise_error(String error_text);

  // Функция проверки условия: если trigger == false, вызвать raise_error
  void check_exec(bool trigger, String error_text);

  void soft_check_exec(bool trigger, String error_text);

  // Ошибка во время работы: true - мигать, false - погасить светодиод
  void setFault(bool active);
  // Вызывать в каждом loop(): мигание без delay()
  void update();

private:
  uint8_t _LedPin;
  bool _fault = false;
  bool _ledOn = false;
  unsigned long _lastToggle = 0;
  static const unsigned long FAULT_BLINK_MS = 125; // 125 мс горит / 125 мс нет

  void _setLed(bool on);
};


#endif
