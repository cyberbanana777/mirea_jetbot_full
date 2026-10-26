#include <cmath>
#include "motor_controller.h"

MotorController::MotorController(float wheelBase, float maxWheelSpeed, float dt,
                                 float kpL, float kiL, float kdL,
                                 float kpR, float kiR, float kdR,
                                 uint8_t leftChannel, uint8_t rightChannel,
                                 bool invertLeft, bool invertRight)
  : _pwm(0x60)  // адрес PCA9685 по умолчанию
    ,
    // I_SATURATE - условное интегрирование (антивиндап), D_INPUT - производная по
    // измерению, а не по ошибке: скачок уставки не даёт удара
    _pidL(I_SATURATE | D_INPUT),
    _pidR(I_SATURATE | D_INPUT),
    _wheelBase(wheelBase),
    _maxWheelSpeed(maxWheelSpeed),
    _dt(dt),
    _kpL(kpL), _kiL(kiL), _kdL(kdL),
    _kpR(kpR), _kiR(kiR), _kdR(kdR),
    _targetLeft(0.0f), _targetRight(0.0f),
    _outputLeft(0.0f), _outputRight(0.0f),
    _measLeft(0.0f), _measRight(0.0f),
    _ffGainL(0.0f), _ffDzL(0.0f), _ffGainR(0.0f), _ffDzR(0.0f),
    _openLoop(false), _openDutyL(0.0f), _openDutyR(0.0f),
    _prevSignL(0), _prevSignR(0),
    _prevRefL(0), _prevRefR(0), _frozenL(0), _frozenR(0),
    _leftChannel(leftChannel), _rightChannel(rightChannel),
    _invertLeft(invertLeft), _invertRight(invertRight), _minDuty(0.02f)

{
  _lastDir[0] = 2;
  _lastDir[1] = 2;
}

bool MotorController::begin() {
  delay(10);
  if (!_pwm.begin()) return false;
  _pwm.setPWMFreq(1000);
  delay(10);

  _pidL.setKp(_kpL);
  _pidL.setKi(_kiL);
  _pidL.setKd(_kdL);
  _pidL.setDt(_dt);
  _pidL.outMax = 1.0;
  _pidL.outMin = -1.0;

  _pidR.setKp(_kpR);
  _pidR.setKi(_kiR);
  _pidR.setKd(_kdR);
  _pidR.setDt(_dt);
  _pidR.outMax = 1.0;
  _pidR.outMin = -1.0;

  _lastDir[0] = 2;
  _lastDir[1] = 2;
  return true;
}

bool MotorController::reinitializePWM() {
    _pwm.reset();
    _pwm.setPWMFreq(1000);
    delay(1);
    // После сброса PCA9685 все каналы выключены, так что безопасно
    _lastDir[0] = 2;  // состояние каналов неизвестно: следующая запись выключит обе стороны моста
    _lastDir[1] = 2;
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

  _openLoop = false;
}


void MotorController::setWheelSpeeds(float left, float right) {
  const float limit = _maxWheelSpeed * 0.8f;
  // Образка максимальной скорости
  _targetLeft = _cutValue(-limit, limit, left);
  _targetRight = _cutValue(-limit, limit, right);

  _pidL.setpoint = _targetLeft;
  _pidR.setpoint = _targetRight;

  _openLoop = false;
}

void MotorController::setOpenLoop(float dutyLeft, float dutyRight) {
  _openDutyL = _cutValue(-1.0f, 1.0f, dutyLeft);
  _openDutyR = _cutValue(-1.0f, 1.0f, dutyRight);
  // Регуляторы выключены: уставки и интеграторы обнуляем, чтобы при возврате
  // в замкнутый режим не осталось старого состояния
  _targetLeft = 0.0f;
  _targetRight = 0.0f;
  _pidL.setpoint = 0.0f;
  _pidR.setpoint = 0.0f;
  _pidL.integral = 0.0f;
  _pidR.integral = 0.0f;
  _prevSignL = 0;
  _prevSignR = 0;
  _prevRefL = _prevRefR = 0.0f;
  _frozenL = _frozenR = 0;
  _openLoop = true;
}

void MotorController::setFeedforward(float gainL, float deadzoneL, float gainR, float deadzoneR) {
  _ffGainL = gainL;
  _ffDzL = deadzoneL;
  _ffGainR = gainR;
  _ffDzR = deadzoneR;
}

// Уставка по модулю ниже этого, м/с, считается нулём: мотор выключен, интегратор сброшен
static const float SETPOINT_ZERO_EPS = 0.002f;
// Заморозка интеграла при разгоне: пока колесо едет к уставке и ошибка больше этой доли
// уставки, интегральная часть ПИ не копится (иначе она набирается за время разгона и даёт
// перерегулирование 20-30%). Заморозка ограничена по времени (INT_FREEZE_MAX_TICKS тактов после
// ступеньки уставки): под нагрузкой feedforward из калибровки "в воздухе" занижен, колесо
// не доходит до уставки, и без ограничения интеграл не исправил бы ошибку никогда.
// INT_FREEZE_BAND = 10 выключает заморозку совсем.
static const float INT_FREEZE_BAND = 0.10f;
static const int16_t INT_FREEZE_MAX_TICKS = 30;   // 300 мс при 100 Гц: порядка Tm колеса
// Ступенька уставки: изменение больше этой доли уставки (плавный разгон по cmd_vel не считается)
static const float SETPOINT_STEP_REL = 0.2f;
// Подъём скважности до страгивания: пока колесо стоит, а уставка не нулевая, интегральная часть
// растёт со скоростью STUCK_RAMP_RATE (скважность в секунду) независимо от малой ошибки.
// Иначе на уставке 0.05 об/с интеграл (Ki * ошибка 0.01 м/с = 0.06-0.1 скважности в секунду)
// набирает трение покоя секунды; под нагрузкой на полу страгивание вообще за 0.4 ШИМ.
// 0 выключает.
static const float STUCK_RAMP_RATE = 0.3f;

float MotorController::_wheelStep(uPID &pid, float meas, float gain, float deadzone, int8_t &prevSign,
                                  float &prevRef, int16_t &frozenTicks) {
  const float r = pid.setpoint;
  const int8_t sign = (r > SETPOINT_ZERO_EPS) ? 1 : ((r < -SETPOINT_ZERO_EPS) ? -1 : 0);

  // Начало движения или реверс: интеграл прошлого направления не нужен
  if (sign != prevSign) pid.integral = 0.0f;
  prevSign = sign;

  // Ступенька уставки: заморозке интеграла даётся новое окно времени
  if (fabsf(r - prevRef) > SETPOINT_STEP_REL * fabsf(r) + 0.005f) frozenTicks = 0;
  prevRef = r;

  // Feedforward: сразу выдаёт основную часть управления, ПИ добирает остаток
  const float ff = (sign == 0) ? 0.0f : (float)sign * (deadzone + gain * fabsf(r));

  // Пределы ПИ сдвинуты так, чтобы сумма ff + ПИ не выходила за [-1, 1]:
  // тогда условное интегрирование uPID видит насыщение полного сигнала
  pid.outMax = 1.0f - ff;
  pid.outMin = -1.0f - ff;

  // Вызываем и в покое: так внутреннее состояние (прошлое измерение для D) не устаревает.
  // Разгон/торможение к уставке: пока колесо едет и ещё не дошло до уставки (ошибка того же
  // знака, что и уставка, и больше полосы INT_FREEZE_BAND), интеграл не копится. Feedforward уже
  // выдаёт нужное установившееся управление; интеграл, набранный за время разгона, не нужен
  // и давал бы перерегулирование. Стоящее колесо (трогание) и перелёт (ошибка другого знака)
  // интегрируют как обычно: иначе нечем преодолеть трение покоя и снять перелёт.
  const float integralBefore = pid.integral;
  const float err = r - meas;
  const float absR = fabsf(r);
  const bool moving = fabsf(meas) > 0.1f * absR;
  const bool approaching = (err * (float)sign) > 0.0f && fabsf(err) > INT_FREEZE_BAND * absR;
  const float pi = pid.compute(meas);
  if (moving && approaching && frozenTicks < INT_FREEZE_MAX_TICKS) {
    pid.integral = integralBefore;
    ++frozenTicks;
  }

  // Колесо стоит, а уставка есть: поднимаем скважность, пока оно не тронется
  if (sign != 0 && !moving && STUCK_RAMP_RATE > 0.0f && pid.Ki > 0.01f) {
    pid.integral += (float)sign * STUCK_RAMP_RATE * (_dt * 0.001f) / pid.Ki;
  }

  if (sign == 0) {
    pid.integral = 0.0f;
    return 0.0f;
  }
  return _cutValue(-1.0f, 1.0f, ff + pi);
}

void MotorController::update(float currentLeftSpeed, float currentRightSpeed) {
  _measLeft = currentLeftSpeed;
  _measRight = currentRightSpeed;

  if (_openLoop) {
    _outputLeft = _openDutyL;
    _outputRight = _openDutyR;
  } else {
    _outputLeft = _wheelStep(_pidL, _measLeft, _ffGainL, _ffDzL, _prevSignL, _prevRefL, _frozenL);
    _outputRight = _wheelStep(_pidR, _measRight, _ffGainR, _ffDzR, _prevSignR, _prevRefR, _frozenR);
  }

  _motorWrite(_leftChannel, _outputLeft);
  _motorWrite(_rightChannel, _outputRight);
}


void MotorController::setPID(float kpL, float kiL, float kdL,
                             float kpR, float kiR, float kdR) {
  _pidL.setKp(kpL);
  _pidL.setKi(kiL);
  _pidL.setKd(kdL);

  _pidR.setKp(kpR);
  _pidR.setKi(kiR);
  _pidR.setKd(kdR);

}

void MotorController::_motorWrite(uint8_t channel, float speed) {
  // Применяем инверсию, если нужно
  float s = speed;
  if (channel == _leftChannel && _invertLeft) s = -s;
  if (channel == _rightChannel && _invertRight) s = -s;
  const uint8_t idx = (channel == _leftChannel) ? 0 : 1;

  // Мёртвой зоны здесь больше нет: её компенсирует feedforward. Отсекается только
  // скважность ниже _minDuty, чтобы не пищать и не греть мотор впустую.
  int8_t dir = 0;
  int16_t pwmValue = 0;
  if (s > _minDuty) {
    dir = 1;
    pwmValue = (int16_t)(fminf(s, 1.0f) * 4095);
  } else if (s < -_minDuty) {
    dir = -1;
    pwmValue = (int16_t)(fminf(-s, 1.0f) * 4095);
  }

  // На такте пишем один регистр, а вторую сторону моста выключаем только при смене
  // направления: на шине I2C меньше транзакций (контур теперь 100 Гц, а не 10 Гц)
  if (dir == 0) {
    if (_lastDir[idx] != 0) {
      _pwm.setPin(channel, 0, false);
      _pwm.setPin(channel + 1, 0, false);
    }
  } else if (dir > 0) {
    if (_lastDir[idx] != 1) _pwm.setPin(channel + 1, 0, false);  // канал назад в 0
    _pwm.setPin(channel, pwmValue, false);
  } else {
    if (_lastDir[idx] != -1) _pwm.setPin(channel, 0, false);  // канал вперёд в 0
    _pwm.setPin(channel + 1, pwmValue, false);
  }
  _lastDir[idx] = dir;
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
