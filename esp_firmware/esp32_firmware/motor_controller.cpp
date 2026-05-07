#include <cmath>
#include "motor_controller.h"

MotorController::MotorController(float wheelBase, float maxWheelSpeed, float dt,
                                 float kpL, float kiL, float kdL,
                                 float kpR, float kiR, float kdR,
                                 uint8_t leftChannel, uint8_t rightChannel,
                                 bool invertLeft, bool invertRight)
  : _pwm(0x60)  // адрес PCA9685 по умолчанию
    ,
    _pidL(kpL, kiL, kdL, dt),
    _pidR(kpR, kiR, kdR, dt),
    _wheelBase(wheelBase),
    _maxWheelSpeed(maxWheelSpeed),
    _dt(dt),
    _targetLeft(0.0f), _targetRight(0.0f),
    _outputLeft(0.0f), _outputRight(0.0f),
    _leftChannel(leftChannel), _rightChannel(rightChannel),
    _invertLeft(invertLeft), _invertRight(invertRight), _deathLine(0.05f)

{}

bool MotorController::begin() {
  delay(10);
  if (!_pwm.begin()) return false;
  _pwm.setPWMFreq(1000);
  delay(10);

  _pidL.setLimits(-1.0, 1.0);
  _pidR.setLimits(-1.0, 1.0);
  return true;
}

void MotorController::setTargetVelocity(float linear, float angular) {
  // Дифференциальный привод
  float rawLeft = linear - angular * _wheelBase / 2.0f;   // м/с
  float rawRight = linear + angular * _wheelBase / 2.0f;  // м/с

  // Ограничение и масштабирование в диапазон [-0.8, 0.8] (как в оригинале)
  const float limit = _maxWheelSpeed * 0.8f;
  // Образка максимальной скорости
  _targetLeft = _cutValue(-limit, limit, rawLeft);
  _targetRight = _cutValue(-limit, limit, rawRight);

  _pidL.setpoint = _targetLeft;
  _pidR.setpoint = _targetRight;

  _filterLeft.reset();
  _filterRight.reset();
}


void MotorController::setWheelSpeeds(float left, float right) {
  const float limit = _maxWheelSpeed * 0.8f;
  // Образка максимальной скорости
  _targetLeft = _cutValue(-limit, limit, left);
  _targetRight = _cutValue(-limit, limit, right);

  _pidL.setpoint = _targetLeft;
  _pidR.setpoint = _targetRight;

  _filterLeft.reset();
  _filterRight.reset();
}

void MotorController::update(float currentLeftSpeed, float currentRightSpeed) {
  // Подаём сырые скорости на фильтры
  _filterLeft.add(currentLeftSpeed);
  _filterRight.add(currentRightSpeed);

  // ПИД получает сглаженные значения
  _filteredLeft = _filterLeft.getAverage();
  _filteredRight = _filterRight.getAverage();
  _pidL.input = _filteredLeft;   // м/с
  _pidR.input = _filteredRight;  // м/с

  _outputLeft = _pidL.getResultNow();
  _outputRight = _pidR.getResultNow();

  _motorWrite(_leftChannel, _outputLeft);
  _motorWrite(_rightChannel, _outputRight);
}


void MotorController::setPID(float kpL, float kiL, float kdL,
                             float kpR, float kiR, float kdR) {
  _pidL.Kp = kpL;
  _pidL.Ki = kiL;
  _pidL.Kd = kdL;
  _pidR.Kp = kpR;
  _pidR.Ki = kiR;
  _pidR.Kd = kdR;
}

float MotorController::_mapFloat(float value, float fromLow, float fromHigh,
                                 float toLow, float toHigh) {
  float clamped = value;
  if (value < fromLow) clamped = fromLow;
  else if (value > fromHigh) clamped = fromHigh;

  return (clamped - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void MotorController::_motorWrite(uint8_t channel, float speed) {
  // Применяем инверсию, если нужно
  float s = speed;
  if (channel == _leftChannel && _invertLeft) s = -s;
  if (channel == _rightChannel && _invertRight) s = -s;

  int16_t pwmValue = 0;
  if ((s > 0) && (s > _deathLine)) {
    pwmValue = (int16_t)(s * 4095);
    _pwm.setPin(channel + 1, 0, false);  // канал назад в 0
    _pwm.setPin(channel, pwmValue, false);
  } else if ((s < 0) && (s < -_deathLine)) {
    pwmValue = (int16_t)(-s * 4095);
    _pwm.setPin(channel, 0, false);  // канал вперёд в 0
    _pwm.setPin(channel + 1, pwmValue, false);
  } else {
    _pwm.setPin(channel, 0, false);
    _pwm.setPin(channel + 1, 0, false);
  }
}

float_t MotorController::_cutValue(float_t min, float_t max, float_t value) {
  if (value > max) {
    return max;
  } else if (value < min) {
    return min;
  } else {
    return value;
  }
}