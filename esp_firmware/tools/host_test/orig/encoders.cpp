#include <sys/_stdint.h>
#include "encoders.h"

volatile int32_t Encoders::_leftPos = 0;
volatile int32_t Encoders::_rightPos = 0;
// Для прерываний (машины состояний)
volatile bool Encoders::_ReadPIN_R_A_A;
volatile bool Encoders::_ReadPIN_R_B_A;
volatile bool Encoders::_ReadPIN_R_A_B;
volatile bool Encoders::_ReadPIN_R_B_B;
volatile bool Encoders::_ReadPIN_L_A_A;
volatile bool Encoders::_ReadPIN_L_B_A;
volatile bool Encoders::_ReadPIN_L_A_B;
volatile bool Encoders::_ReadPIN_L_B_B;
volatile int Encoders::_PIN_R_A;
volatile int Encoders::_PIN_R_B;
volatile int Encoders::_PIN_L_A;
volatile int Encoders::_PIN_L_B;
// Защищает счётчики: чтение+обнуление в loop() не должно разрываться прерыванием
portMUX_TYPE Encoders::_mux = portMUX_INITIALIZER_UNLOCKED;


bool Encoders::begin(int PIN_R_A, int PIN_R_B, int PIN_L_A, int PIN_L_B) {
  _PIN_R_A = PIN_R_A;
  _PIN_R_B = PIN_R_B;
  _PIN_L_A = PIN_L_A;
  _PIN_L_B = PIN_L_B;
  pinMode(_PIN_R_A, INPUT);
  pinMode(_PIN_R_B, INPUT);
  pinMode(_PIN_L_A, INPUT);
  pinMode(_PIN_L_B, INPUT);
  attachInterrupt(_PIN_R_A, _readR_A, CHANGE);
  attachInterrupt(_PIN_R_B, _readR_B, CHANGE);
  attachInterrupt(_PIN_L_A, _readL_A, CHANGE);
  attachInterrupt(_PIN_L_B, _readL_B, CHANGE);
  return true;
}

// Чтение и обнуление выполняются атомарно: иначе тик, пришедший из
// прерывания между чтением и обнулением, терялся бы.
int32_t Encoders::getLeftAndReset() {
  portENTER_CRITICAL(&_mux);
  int32_t val = _leftPos;
  _leftPos = 0;
  portEXIT_CRITICAL(&_mux);
  return val;
}

int32_t Encoders::getRightAndReset() {
  portENTER_CRITICAL(&_mux);
  int32_t val = _rightPos;
  _rightPos = 0;
  portEXIT_CRITICAL(&_mux);
  return val;
}

void Encoders::reset() {
  portENTER_CRITICAL(&_mux);
  _leftPos = 0;
  _rightPos = 0;
  portEXIT_CRITICAL(&_mux);
}

// обработчики прерываний (код из вашего файла, адаптированный)
void IRAM_ATTR Encoders::_readR_A() {
  _ReadPIN_R_A_A = digitalRead(_PIN_R_A);
  _ReadPIN_R_B_A = digitalRead(_PIN_R_B);
  portENTER_CRITICAL_ISR(&_mux);

  switch (_ReadPIN_R_A_A) {
    case 0:
      if (_ReadPIN_R_B_A == 1) {_rightPos += 1; break;}
      if (_ReadPIN_R_B_A == 0) {_rightPos -= 1; break;}
      break;
    
    case 1:
      if (_ReadPIN_R_B_A == 1) {_rightPos -= 1; break;}
      if (_ReadPIN_R_B_A == 0) {_rightPos += 1; break;}
      break;
  }
  portEXIT_CRITICAL_ISR(&_mux);
}

void IRAM_ATTR Encoders::_readR_B() {
  _ReadPIN_R_A_B = digitalRead(_PIN_R_A);
  _ReadPIN_R_B_B = digitalRead(_PIN_R_B);
  portENTER_CRITICAL_ISR(&_mux);

  switch (_ReadPIN_R_B_B) {
    case 0:
      if (_ReadPIN_R_A_B == 1) {_rightPos -= 1; break;}
      if (_ReadPIN_R_A_B == 0) {_rightPos += 1; break;}
      break;
    
    case 1:
      if (_ReadPIN_R_A_B == 1) {_rightPos += 1; break;}
      if (_ReadPIN_R_A_B == 0) {_rightPos -= 1; break;}
      break;
  }
  portEXIT_CRITICAL_ISR(&_mux);
}


void IRAM_ATTR Encoders::_readL_A() {
  _ReadPIN_L_A_A = digitalRead(_PIN_L_A);
  _ReadPIN_L_B_A = digitalRead(_PIN_L_B);
  portENTER_CRITICAL_ISR(&_mux);

  switch (_ReadPIN_L_A_A) {
    case 0:
      if (_ReadPIN_L_B_A == 1) {_leftPos += 1; break;}
      if (_ReadPIN_L_B_A == 0) {_leftPos -= 1; break;}
      break;
    
    case 1:
      if (_ReadPIN_L_B_A == 1) {_leftPos -= 1; break;}
      if (_ReadPIN_L_B_A == 0) {_leftPos += 1; break;}
      break;
  }
  portEXIT_CRITICAL_ISR(&_mux);
}

void IRAM_ATTR Encoders::_readL_B() {
  _ReadPIN_L_A_B = digitalRead(_PIN_L_A);
  _ReadPIN_L_B_B = digitalRead(_PIN_L_B);
  portENTER_CRITICAL_ISR(&_mux);

  switch (_ReadPIN_L_B_B) {
    case 0:
      if (_ReadPIN_L_A_B == 1) {_leftPos -= 1; break;}
      if (_ReadPIN_L_A_B == 0) {_leftPos += 1; break;}
      break;
    
    case 1:
      if (_ReadPIN_L_A_B == 1) {_leftPos += 1; break;}
      if (_ReadPIN_L_A_B == 0) {_leftPos -= 1; break;}
      break;
  }
  portEXIT_CRITICAL_ISR(&_mux);
}

