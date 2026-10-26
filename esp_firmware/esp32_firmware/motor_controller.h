#include <cmath>
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <uPID.h>

class MotorController {
public:
    /**
     * Конструктор
     * @param wheelBase      - длина базы робота (l), м
     * @param maxWheelSpeed  - максимальная скорость колеса (max_vel), м/с
     * @param dt             - период дискретизации ПИ-регуляторов, мс
     * @param kpL, kiL, kdL  - коэффициенты ПИД для левого колеса
     * @param kpR, kiR, kdR  - коэффициенты ПИД для правого колеса
     * @param leftChannel    - базовый канал PCA9685 для левого мотора (по умолч. 0x00)
     * @param rightChannel   - базовый канал PCA9685 для правого мотора (по умолч. 0x02)
     * @param invertLeft     - инвертировать ли левый мотор (true)
     * @param invertRight    - инвертировать ли правый мотор (false)
     */
    MotorController(float wheelBase, float maxWheelSpeed, float dt,
                    float kpL, float kiL, float kdL,
                    float kpR, float kiR, float kdR,
                    uint8_t leftChannel = 0x00, uint8_t rightChannel = 0x02,
                    bool invertLeft = true, bool invertRight = false);

    // Инициализация PCA9685 и установка пределов ПИД
    bool begin();

    bool reinitializePWM();

    // Установка целевой скорости робота (линейная, угловая)
    void setTargetVelocity(float linear, float angular);

    // Прямая установка скоростей колёс (м/с)
    void setWheelSpeeds(float left, float right);

    // Разомкнутый режим для экспериментов: скважность ШИМ каждого колеса, [-1, 1]
    // (+ = колесо вперёд). Регуляторы отключены. Любая команда скорости
    // (setWheelSpeeds / setTargetVelocity, в том числе аварийная остановка)
    // возвращает замкнутый режим.
    void setOpenLoop(float dutyLeft, float dutyRight);
    bool isOpenLoop() const { return _openLoop; }

    // Feedforward: u_ff = sign(v) * (deadzone + gain * |v|), скважность ШИМ
    // gain - скважность на 1 м/с, deadzone - скважность, с которой колесо трогается
    void setFeedforward(float gainL, float deadzoneL, float gainR, float deadzoneR);

    /**
     * Один такт контура скорости (вызывать строго по таймеру dt)
     * @param currentLeftSpeed   - измеренная скорость левого колеса (м/с)
     * @param currentRightSpeed  - измеренная скорость правого колеса (м/с)
     */
    void update(float currentLeftSpeed, float currentRightSpeed);

    // Геттеры текущих целевых скоростей (после ограничений)
    float getTargetLeft() const  { return _targetLeft; }
    float getTargetRight() const { return _targetRight; }

    // Измеренные скорости колёс (м/с), как их видит регулятор
    float getFilteredLeft() const  { return _measLeft; }
    float getFilteredRight() const { return _measRight; }

    // Установка новых коэффициентов ПИД (можно вызывать в любой момент)
    void setPID(float kpL, float kiL, float kdL,
                float kpR, float kiR, float kdR);

    // Текущие управляющие сигналы: скважность ШИМ, [-1, 1]
    float getOutputLeft() const  { return _outputLeft; }
    float getOutputRight() const { return _outputRight; }

private:
    Adafruit_PWMServoDriver _pwm;
    uPID _pidL, _pidR;

    float _wheelBase;
    float _maxWheelSpeed;
    float _dt;

    float _kpL, _kiL, _kdL;
    float _kpR, _kiR, _kdR;

    float _targetLeft, _targetRight;
    float _outputLeft, _outputRight;
    float _measLeft, _measRight;

    // Feedforward
    float _ffGainL, _ffDzL, _ffGainR, _ffDzR;

    bool _openLoop;
    float _openDutyL, _openDutyR;
    int8_t _prevSignL, _prevSignR; // знак уставки на прошлом такте (сброс интегратора при реверсе)
    float _prevRefL, _prevRefR;    // уставка на прошлом такте (обнаружение ступеньки)
    int16_t _frozenL, _frozenR;    // сколько тактов подряд интеграл заморожен при разгоне

    uint8_t _leftChannel, _rightChannel;
    bool _invertLeft, _invertRight;
    // Скважность ниже этой не подаётся на мотор (не пищать и не греть впустую)
    float _minDuty;
    // Какая сторона моста включена в последний раз: -1 назад, 0 выкл, +1 вперёд,
    // 2 - неизвестно (после сброса). Позволяет писать на такте один регистр вместо двух.
    int8_t _lastDir[2];

    // Один такт для одного колеса: feedforward + ПИ, возвращает скважность [-1, 1]
    float _wheelStep(uPID &pid, float meas, float gain, float deadzone, int8_t &prevSign,
                     float &prevRef, int16_t &frozenTicks);

    // Низкоуровневая запись на мотор
    void _motorWrite(uint8_t channel, float speed);

    // Функция обрезки
    float_t _cutValue(float_t min, float_t max, float_t value);
};

#endif
