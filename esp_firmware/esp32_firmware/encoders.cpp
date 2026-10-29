#include <sys/_stdint.h>
#include "encoders.h"

volatile int32_t Encoders::_leftPos = 0;
volatile int32_t Encoders::_rightPos = 0;
// Накопленная позиция (не обнуляется) и время последнего фронта - для оценки скорости
volatile int32_t Encoders::_leftTotal = 0;
volatile int32_t Encoders::_rightTotal = 0;
volatile uint32_t Encoders::_leftLastUs = 0;
volatile uint32_t Encoders::_rightLastUs = 0;
volatile uint32_t Encoders::_leftPhaseUs[4] = {0, 0, 0, 0};
volatile uint32_t Encoders::_rightPhaseUs[4] = {0, 0, 0, 0};
volatile int8_t Encoders::_leftDir = 1;
volatile int8_t Encoders::_rightDir = 1;
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

// Позиция и время фронта читаются под одним замком: пара (total, lastUs)
// всегда относится к одному и тому же фронту.
void Encoders::snapshotLeft(int32_t &total, uint32_t &lastEdgeUs) {
  portENTER_CRITICAL(&_mux);
  total = _leftTotal;
  lastEdgeUs = _leftLastUs;
  portEXIT_CRITICAL(&_mux);
}

void Encoders::snapshotRight(int32_t &total, uint32_t &lastEdgeUs) {
  portENTER_CRITICAL(&_mux);
  total = _rightTotal;
  lastEdgeUs = _rightLastUs;
  portEXIT_CRITICAL(&_mux);
}

void Encoders::snapshotLeftPhases(int32_t &total, uint32_t phaseUs[4], int8_t &lastDir) {
  portENTER_CRITICAL(&_mux);
  total = _leftTotal;
  for (int i = 0; i < 4; ++i) phaseUs[i] = _leftPhaseUs[i];
  lastDir = _leftDir;
  portEXIT_CRITICAL(&_mux);
}

void Encoders::snapshotRightPhases(int32_t &total, uint32_t phaseUs[4], int8_t &lastDir) {
  portENTER_CRITICAL(&_mux);
  total = _rightTotal;
  for (int i = 0; i < 4; ++i) phaseUs[i] = _rightPhaseUs[i];
  lastDir = _rightDir;
  portEXIT_CRITICAL(&_mux);
}

// Учёт одного фронта. Вызывается из прерывания внутри критической секции.
// Номер фронта (тип = id & 3): id = большая из позиций до и после фронта.
void IRAM_ATTR Encoders::_countR(int8_t d) {
  const uint32_t now = micros();
  const int32_t before = _rightTotal;
  _rightPos += d;
  _rightTotal += d;
  _rightLastUs = now;
  if (d != 0) {
    const uint32_t id = (uint32_t)(d > 0 ? _rightTotal : before);
    _rightPhaseUs[id & 3u] = now;
    _rightDir = d;
  }
}

void IRAM_ATTR Encoders::_countL(int8_t d) {
  const uint32_t now = micros();
  const int32_t before = _leftTotal;
  _leftPos += d;
  _leftTotal += d;
  _leftLastUs = now;
  if (d != 0) {
    const uint32_t id = (uint32_t)(d > 0 ? _leftTotal : before);
    _leftPhaseUs[id & 3u] = now;
    _leftDir = d;
  }
}

// обработчики прерываний (код из вашего файла, адаптированный):
// конечный автомат прежний, но вместо прямого изменения счётчика он
// выдаёт шаг d = +1/-1, а учёт (позиция + время фронта) делает _countR/_countL.
void IRAM_ATTR Encoders::_readR_A() {
  _ReadPIN_R_A_A = digitalRead(_PIN_R_A);
  _ReadPIN_R_B_A = digitalRead(_PIN_R_B);
  int8_t d = 0;

  switch (_ReadPIN_R_A_A) {
    case 0:
      if (_ReadPIN_R_B_A == 1) {d = +1; break;}
      if (_ReadPIN_R_B_A == 0) {d = -1; break;}
      break;

    case 1:
      if (_ReadPIN_R_B_A == 1) {d = -1; break;}
      if (_ReadPIN_R_B_A == 0) {d = +1; break;}
      break;
  }
  portENTER_CRITICAL_ISR(&_mux);
  _countR(d);
  portEXIT_CRITICAL_ISR(&_mux);
}

void IRAM_ATTR Encoders::_readR_B() {
  _ReadPIN_R_A_B = digitalRead(_PIN_R_A);
  _ReadPIN_R_B_B = digitalRead(_PIN_R_B);
  int8_t d = 0;

  switch (_ReadPIN_R_B_B) {
    case 0:
      if (_ReadPIN_R_A_B == 1) {d = -1; break;}
      if (_ReadPIN_R_A_B == 0) {d = +1; break;}
      break;

    case 1:
      if (_ReadPIN_R_A_B == 1) {d = +1; break;}
      if (_ReadPIN_R_A_B == 0) {d = -1; break;}
      break;
  }
  portENTER_CRITICAL_ISR(&_mux);
  _countR(d);
  portEXIT_CRITICAL_ISR(&_mux);
}


void IRAM_ATTR Encoders::_readL_A() {
  _ReadPIN_L_A_A = digitalRead(_PIN_L_A);
  _ReadPIN_L_B_A = digitalRead(_PIN_L_B);
  int8_t d = 0;

  switch (_ReadPIN_L_A_A) {
    case 0:
      if (_ReadPIN_L_B_A == 1) {d = +1; break;}
      if (_ReadPIN_L_B_A == 0) {d = -1; break;}
      break;

    case 1:
      if (_ReadPIN_L_B_A == 1) {d = -1; break;}
      if (_ReadPIN_L_B_A == 0) {d = +1; break;}
      break;
  }
  portENTER_CRITICAL_ISR(&_mux);
  _countL(d);
  portEXIT_CRITICAL_ISR(&_mux);
}

void IRAM_ATTR Encoders::_readL_B() {
  _ReadPIN_L_A_B = digitalRead(_PIN_L_A);
  _ReadPIN_L_B_B = digitalRead(_PIN_L_B);
  int8_t d = 0;

  switch (_ReadPIN_L_B_B) {
    case 0:
      if (_ReadPIN_L_A_B == 1) {d = -1; break;}
      if (_ReadPIN_L_A_B == 0) {d = +1; break;}
      break;

    case 1:
      if (_ReadPIN_L_A_B == 1) {d = +1; break;}
      if (_ReadPIN_L_A_B == 0) {d = -1; break;}
      break;
  }
  portENTER_CRITICAL_ISR(&_mux);
  _countL(d);
  portEXIT_CRITICAL_ISR(&_mux);
}

