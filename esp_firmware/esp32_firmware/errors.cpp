#include <sys/_stdint.h>
#include "errors.h"
#include <Arduino.h>

void ErrorManager::begin(uint8_t LedPin){
  _LedPin = LedPin;
  pinMode(_LedPin, OUTPUT);
}

void ErrorManager::raise_error(String error_text){
  while(true){
    digitalWrite(_LedPin, HIGH);
    delay(500);
    digitalWrite(_LedPin, LOW);
    delay(500);
    Serial.println(error_text);
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