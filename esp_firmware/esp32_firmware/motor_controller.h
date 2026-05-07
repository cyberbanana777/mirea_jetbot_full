#include <cmath>
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#define PID_INTEGRAL_WINDOW 40
#define PID_OPTIMIZED_I
#include <GyverPID.h>
#include "filter.h"

class MotorController {
public:
    /**
     * Конструктор
     * @param wheelBase      - длина базы робота (l), м
     * @param maxWheelSpeed  - максимальная скорость колеса (max_vel), м/с
     * @param dt             - период дискретизации ПИД-регуляторов, с
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

    // Установка целевой скорости робота (линейная, угловая)
    void setTargetVelocity(float linear, float angular);

    // Прямая установка скоростей колёс (м/с)
    void setWheelSpeeds(float left, float right);

    /**
     * Обновление регуляторов (вызывать строго по таймеру dt)
     * @param currentLeftSpeed   - текущая скорость левого колеса (м/с)
     * @param currentRightSpeed  - текущая скорость правого колеса (м/с)
     */
    void update(float currentLeftSpeed, float currentRightSpeed);

    // Геттеры текущих целевых скоростей (после ограничений)
    float getTargetLeft() const  { return _targetLeft; }
    float getTargetRight() const { return _targetRight; }

    float getFilteredLeft() const  { return _filteredLeft; }
    float getFilteredRight() const { return _filteredRight; }

    // Установка новых коэффициентов ПИД (можно вызывать в любой момент)
    void setPID(float kpL, float kiL, float kdL,
                float kpR, float kiR, float kdR);

    // Получение текущих управляющих сигналов (опционально)
    float getOutputLeft() const  { return _outputLeft; }
    float getOutputRight() const { return _outputRight; }

private:
    Adafruit_PWMServoDriver _pwm;
    GyverPID _pidL, _pidR;
    MovingAverage<5> _filterLeft;
    MovingAverage<5> _filterRight;

    float _wheelBase;
    float _maxWheelSpeed;
    float _dt;

    float _targetLeft, _targetRight;
    float _outputLeft, _outputRight;
    float _filteredLeft, _filteredRight;

    uint8_t _leftChannel, _rightChannel;
    bool _invertLeft, _invertRight;
    float _deathLine;


    // Вспомогательная функция масштабирования с ограничением
    static float _mapFloat(float value, float fromLow, float fromHigh,
                           float toLow, float toHigh);

    // Низкоуровневая запись на мотор
    void _motorWrite(uint8_t channel, float speed);

    // Функция обрезки
    float_t _cutValue(float_t min, float_t max, float_t value);
};

#endif