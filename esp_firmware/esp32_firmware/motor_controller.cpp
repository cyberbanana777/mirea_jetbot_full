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
    _refTau(0.0f), _refL(0), _refR(0), _tgtPrevL(0), _tgtPrevR(0), _frozenL(0), _frozenR(0),
    _mode(ControlMode::kPi), _wc(5.0f), _wo(25.0f), _b0L(10.0f), _b0R(10.0f), _adrcFf(0),
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

  _openLoop = false;
}


void MotorController::setWheelSpeeds(float left, float right) {
  const float limit = _maxWheelSpeed * 0.8f;
  // Образка максимальной скорости
  _targetLeft = _cutValue(-limit, limit, left);
  _targetRight = _cutValue(-limit, limit, right);

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
  _refL = _refR = 0.0f;
  _tgtPrevL = _tgtPrevR = 0.0f;
  _pidL.integral = 0.0f;
  _pidR.integral = 0.0f;
  _prevSignL = 0;
  _prevSignR = 0;
  _frozenL = _frozenR = 0;
  _esoL = Eso();
  _esoR = Eso();
  _openLoop = true;
}

void MotorController::setRefFilter(float tauSeconds) {
  _refTau = (tauSeconds > 0.0f) ? tauSeconds : 0.0f;
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

void MotorController::_filterRef(float target, float &ref, float &prevTarget, int16_t &frozenTicks, float alpha) {
  // Ступенька заданной скорости (по исходной уставке, а не по фильтрованной): новое окно заморозки интеграла
  if (fabsf(target - prevTarget) > SETPOINT_STEP_REL * fabsf(target) + 0.005f) frozenTicks = 0;
  prevTarget = target;
  if (fabsf(target) < SETPOINT_ZERO_EPS) ref = 0.0f;   // остановка без задержки
  else ref += alpha * (target - ref);
}

float MotorController::_wheelStep(uPID &pid, float meas, float gain, float deadzone, int8_t &prevSign,
                                  int16_t &frozenTicks) {
  const float r = pid.setpoint;
  const int8_t sign = (r > SETPOINT_ZERO_EPS) ? 1 : ((r < -SETPOINT_ZERO_EPS) ? -1 : 0);

  // Начало движения или реверс: интеграл прошлого направления не нужен
  if (sign != prevSign) pid.integral = 0.0f;
  prevSign = sign;

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

// ---------------------------------------------------------------------------------------------
// LADRC (линейный ADRC первого порядка, параметризация через полосы, Z. Gao 2003)
//
// Модель колеса: dv/dt = b0*u + f. Всё, что не b0*u (инерция -v/Tm, мёртвая зона и трение,
// нагрузка, ошибка модели), собрано в "суммарное возмущение" f, которое считается медленно
// меняющимся. Наблюдатель расширенного состояния (ESO), метод Эйлера, шаг h = dt:
//   e  = v_изм - z1
//   z1 += h * (z2 + b0*u + 2*wo*e)        // beta1 = 2*wo
//   z2 += h * (wo^2 * e)                  // beta2 = wo^2: оба полюса ошибки оценки в -wo
// Закон управления: u = (wc*(r - z1) - z2) / b0. Если оценка точна, dv/dt = wc*(r - v),
// то есть замкнутый контур - апериодическое звено с постоянной времени 1/wc для любого колеса.
// В наблюдатель подаётся РЕАЛЬНО поданная (ограниченная) скважность: при насыщении он не
// "думает", что мотор получил больше, и оценка f не забрасывается (естественный антивиндап).
// Необязательная прямая подача по калибровке feedforward (setAdrc, ffMode 1/2) не меняет
// структуру: наблюдатель просто оценивает меньший остаток, и старт с места не ждёт, пока он
// "найдёт" мёртвую зону. Интеграла ошибки здесь нет, поэтому FF не даёт перерегулирования ПИ.
// ---------------------------------------------------------------------------------------------

int8_t MotorController::_signOf(float r) {
  return (r > SETPOINT_ZERO_EPS) ? 1 : ((r < -SETPOINT_ZERO_EPS) ? -1 : 0);
}

float MotorController::_adrcFeedforward(float ref, float gain, float deadzone) const {
  const int8_t sign = _signOf(ref);
  if (!sign || _adrcFf == 0) return 0.0f;
  if (_adrcFf == 1) return (float)sign * deadzone;
  return (float)sign * (deadzone + gain * fabsf(ref));
}

float MotorController::_wheelStepAdrc(float ref, float meas, float b0, float gain, float deadzone, int8_t &prevSign,
                                      Eso &eso) {
  const int8_t sign = _signOf(ref);

  // Старт, реверс или остановка: оценки прошлого направления не нужны (как сброс интеграла у ПИ).
  // z2 = 0: возмущение (в чистом режиме - и мёртвую зону) наблюдатель найдёт заново.
  if (sign != prevSign) {
    eso.z1 = meas;
    eso.z2 = 0.0f;
    eso.u = 0.0f;
  }
  prevSign = sign;
  if (sign == 0) {
    eso.z1 = meas;
    eso.z2 = 0.0f;
    eso.u = 0.0f;
    return 0.0f;
  }

  const float h = _dt * 0.001f;
  // Прямая подача по калибровке: объект видит u = ff + u_eso, т.е. dv/dt = b0*u_eso + (f + b0*ff),
  // и наблюдатель оценивает только остаток, который калибровка не объяснила
  const float ff = _adrcFeedforward(ref, gain, deadzone);

  // Наблюдатель: прогноз по модели + поправка по расхождению с измерением
  const float e = meas - eso.z1;
  eso.z1 += h * (eso.z2 + b0 * eso.u + 2.0f * _wo * e);
  eso.z2 += h * (_wo * _wo * e);

  // Закон управления
  const float u = _cutValue(-1.0f, 1.0f, ff + (_wc * (ref - eso.z1) - eso.z2) / b0);
  eso.u = u - ff;
  return u;
}

void MotorController::_seedAdrc(Eso &eso, float ref, float meas, float uPrev, float b0, float gain, float deadzone) {
  const float ff = _adrcFeedforward(ref, gain, deadzone);
  // z1 = v, а z2 подобран так, что ff + (wc*(r - z1) - z2)/b0 = uPrev
  eso.z1 = meas;
  eso.u = uPrev - ff;
  eso.z2 = _wc * (ref - meas) - b0 * eso.u;
}

void MotorController::_seedPi(uPID &pid, float ref, float meas, float uPrev, float gain, float deadzone,
                              int16_t &frozenTicks) {
  const int8_t sign = _signOf(ref);
  const float ff = sign ? (float)sign * (deadzone + gain * fabsf(ref)) : 0.0f;
  // uPID: выход = Kp*e + Ki*integral (Kd = 0), подбираем integral так, чтобы ff + выход = uPrev
  pid.integral = (pid.Ki > 1e-6f) ? (uPrev - ff - pid.Kp * (ref - meas)) / pid.Ki : 0.0f;
  frozenTicks = INT_FREEZE_MAX_TICKS;   // заморозка разгона не нужна: переход не ступенька
}

void MotorController::setControlMode(ControlMode mode) {
  if (mode == _mode) return;
  if (!_openLoop) {
    if (mode == ControlMode::kLadrc) {
      _seedAdrc(_esoL, _refL, _measLeft, _outputLeft, _b0L, _ffGainL, _ffDzL);
      _seedAdrc(_esoR, _refR, _measRight, _outputRight, _b0R, _ffGainR, _ffDzR);
    } else {
      _seedPi(_pidL, _refL, _measLeft, _outputLeft, _ffGainL, _ffDzL, _frozenL);
      _seedPi(_pidR, _refR, _measRight, _outputRight, _ffGainR, _ffDzR, _frozenR);
    }
  }
  _mode = mode;
}

void MotorController::setAdrc(float wc, float wo, float b0L, float b0R, uint8_t ffMode) {
  _wc = (wc > 0.1f) ? wc : 0.1f;
  _wo = (wo > 0.1f) ? wo : 0.1f;
  _b0L = (b0L > 0.1f) ? b0L : 0.1f;
  _b0R = (b0R > 0.1f) ? b0R : 0.1f;
  _adrcFf = (ffMode > 2) ? 2 : ffMode;
  // Новые параметры на ходу: переподобрать z2 так, чтобы скважность не скакнула
  if (_mode == ControlMode::kLadrc && !_openLoop) {
    if (_signOf(_refL)) _seedAdrc(_esoL, _refL, _measLeft, _outputLeft, _b0L, _ffGainL, _ffDzL);
    if (_signOf(_refR)) _seedAdrc(_esoR, _refR, _measRight, _outputRight, _b0R, _ffGainR, _ffDzR);
  }
}

void MotorController::update(float currentLeftSpeed, float currentRightSpeed) {
  _measLeft = currentLeftSpeed;
  _measRight = currentRightSpeed;

  if (_openLoop) {
    _outputLeft = _openDutyL;
    _outputRight = _openDutyR;
  } else {
    // Фильтр уставки: alpha = dt / (tau + dt), dt в секундах; без фильтра alpha = 1
    const float dtS = _dt * 0.001f;
    const float alpha = (_refTau > 0.0f) ? dtS / (_refTau + dtS) : 1.0f;
    _filterRef(_targetLeft, _refL, _tgtPrevL, _frozenL, alpha);
    _filterRef(_targetRight, _refR, _tgtPrevR, _frozenR, alpha);
    _pidL.setpoint = _refL;
    _pidR.setpoint = _refR;

    if (_mode == ControlMode::kLadrc) {
      _outputLeft = _wheelStepAdrc(_refL, _measLeft, _b0L, _ffGainL, _ffDzL, _prevSignL, _esoL);
      _outputRight = _wheelStepAdrc(_refR, _measRight, _b0R, _ffGainR, _ffDzR, _prevSignR, _esoR);
    } else {
      _outputLeft = _wheelStep(_pidL, _measLeft, _ffGainL, _ffDzL, _prevSignL, _frozenL);
      _outputRight = _wheelStep(_pidR, _measRight, _ffGainR, _ffDzR, _prevSignR, _frozenR);
    }
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
