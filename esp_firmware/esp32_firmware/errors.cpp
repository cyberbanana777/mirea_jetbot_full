#include <sys/_stdint.h>
#include "errors.h"
#include <Arduino.h>

void ErrorManager::begin(uint8_t LedPin){
  _LedPin = LedPin;
  pinMode(_LedPin, OUTPUT);
  // По умолчанию светодиод не горит: он нужен только для индикации ошибок
  _setLed(false);
}

void ErrorManager::_setLed(bool on){
  _ledOn = on;
  digitalWrite(_LedPin, on ? HIGH : LOW);
}

void ErrorManager::raise_error(String error_text){
  uint64_t start_timer_error = millis();
  while(true){
    _setLed(true);
    delay(500);
    _setLed(false);
    delay(500);
    Serial.println(error_text);
    if (millis() - start_timer_error > 10000){
      ESP.restart();
    }
  }
}

void ErrorManager::check_exec(bool trigger, String error_text){
  if(!trigger){
    raise_error(error_text);
  }
}

void ErrorManager::soft_check_exec(bool trigger, String error_text){
  if(!trigger){
    Serial.println(error_text);
    delay(500);
  }
}

void ErrorManager::setFault(bool active){
  if (active == _fault) return;
  _fault = active;
  if (_fault){
    _setLed(true);            // первая вспышка сразу
    _lastToggle = millis();
  } else {
    _setLed(false);           // ошибка ушла - гасим
  }
}

void ErrorManager::update(){
  if (!_fault) return;
  unsigned long now = millis();
  if (now - _lastToggle >= FAULT_BLINK_MS){
    _lastToggle = now;
    _setLed(!_ledOn);
  }
}
