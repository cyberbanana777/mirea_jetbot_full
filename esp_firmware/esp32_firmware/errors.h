#include <sys/_stdint.h>
#ifndef ERRORS_H
#define ERRORS_H

#include <Arduino.h>

class ErrorManager{
public:
  void begin(uint8_t LedPin);
  // Функция для аварийной остановки с выводом текста ошибки
  void raise_error(String error_text);

  // Функция проверки условия: если trigger == false, вызвать raise_error
  void check_exec(bool trigger, String error_text);

  void soft_check_exec(bool trigger, String error_text);

private:
  uint8_t _LedPin;
};


#endif