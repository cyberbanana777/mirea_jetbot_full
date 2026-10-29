#include <cmath>
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <uPID.h>

// Какой регулятор считает скважность в замкнутом режиме
// Имена с префиксом k: в Arduino.h есть макрос PI, который ломает имя ControlMode::PI.
enum class ControlMode : uint8_t {
    kPi = 0,     // feedforward + ПИ с заморозкой интеграла и рампой страгивания
    kLadrc = 1   // линейный ADRC первого порядка (наблюдатель расширенного состояния)
};

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

    // Фильтр уставки: одинаковый для обоих колёс фильтр первого порядка (постоянная времени, с).
    // Оба колеса отслеживают одну и ту же плавную траекторию, поэтому ступенька задания не даёт
    // перерегулирования и переходные процессы колёс совпадают, даже если двигатели разные.
    // 0 - без фильтра. Нулевая уставка (остановка, в том числе аварийная) проходит без задержки.
    void setRefFilter(float tauSeconds);
    float getRefFilter() const { return _refTau; }

    // Feedforward: u_ff = sign(v) * (deadzone + gain * |v|), скважность ШИМ
    // gain - скважность на 1 м/с, deadzone - скважность, с которой колесо трогается
    void setFeedforward(float gainL, float deadzoneL, float gainR, float deadzoneR);

    // Выбор регулятора. Переключение на ходу безударное: состояние нового регулятора
    // (интеграл ПИ или оценки наблюдателя LADRC) подбирается так, чтобы на следующем такте
    // скважность совпала с последней поданной.
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const { return _mode; }

    /**
     * Параметры LADRC (общие wc и wo для обоих колёс - одинаковая желаемая динамика).
     * @param wc     полоса контура, рад/с (lambda = 1/wc)
     * @param wo     полоса наблюдателя ESO, рад/с (beta1 = 2*wo, beta2 = wo^2)
     * @param b0L    ускорение левого колеса на единицу скважности, (м/с^2)/1.0 = K/Tm
     * @param b0R    то же для правого
     * @param ffMode прямая подача по калибровке feedforward ($11), ESO оценивает остаток:
     *               0 - нет (чистый LADRC: всё, включая мёртвую зону, находит наблюдатель),
     *               1 - только мёртвая зона sign(r)*dz,
     *               2 - полный статический feedforward sign(r)*(dz + gain*|r|), как у ПИ
     */
    void setAdrc(float wc, float wo, float b0L, float b0R, uint8_t ffMode);
    float getAdrcWc() const { return _wc; }
    float getAdrcWo() const { return _wo; }
    float getAdrcB0Left() const { return _b0L; }
    float getAdrcB0Right() const { return _b0R; }
    uint8_t getAdrcFfMode() const { return _adrcFf; }
    // Оценка суммарного возмущения f (z2 наблюдателя), м/с^2. В установившемся режиме
    // -z2/b0 - это скважность, которую наблюдатель "нашёл" сам (аналог feedforward + интеграла).
    float getDisturbanceLeft() const  { return _esoL.z2; }
    float getDisturbanceRight() const { return _esoR.z2; }

    /**
     * Один такт контура скорости (вызывать строго по таймеру dt)
     * @param currentLeftSpeed   - измеренная скорость левого колеса (м/с)
     * @param currentRightSpeed  - измеренная скорость правого колеса (м/с)
     */
    void update(float currentLeftSpeed, float currentRightSpeed);

    // Геттеры текущих целевых скоростей (после ограничений)
    float getTargetLeft() const  { return _targetLeft; }
    float getTargetRight() const { return _targetRight; }

    // Уставки после фильтра (то, что реально отрабатывает регулятор), м/с
    float getRefLeft() const  { return _refL; }
    float getRefRight() const { return _refR; }

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
    float _refTau;                 // постоянная времени фильтра уставки, с
    float _refL, _refR;            // уставки после фильтра, м/с
    float _tgtPrevL, _tgtPrevR;    // заданная скорость на прошлом такте (обнаружение ступеньки)
    int16_t _frozenL, _frozenR;    // сколько тактов подряд интеграл заморожен при разгоне

    // LADRC
    struct Eso {
        float z1 = 0.0f;  // оценка скорости, м/с
        float z2 = 0.0f;  // оценка суммарного возмущения f, м/с^2
        float u = 0.0f;   // поданная скважность (после ограничения) за вычетом прямой подачи
    };
    ControlMode _mode;
    float _wc, _wo, _b0L, _b0R;
    uint8_t _adrcFf;   // 0 - без прямой подачи, 1 - мёртвая зона, 2 - полный feedforward
    Eso _esoL, _esoR;

    uint8_t _leftChannel, _rightChannel;
    bool _invertLeft, _invertRight;
    // Скважность ниже этой не подаётся на мотор (не пищать и не греть впустую)
    float _minDuty;
    // Какая сторона моста включена в последний раз: -1 назад, 0 выкл, +1 вперёд,
    // 2 - неизвестно (после сброса). Позволяет писать на такте один регистр вместо двух.
    int8_t _lastDir[2];

    // Один такт для одного колеса: feedforward + ПИ, возвращает скважность [-1, 1]
    float _wheelStep(uPID &pid, float meas, float gain, float deadzone, int8_t &prevSign,
                     int16_t &frozenTicks);

    // Один такт LADRC для одного колеса, возвращает скважность [-1, 1]
    float _wheelStepAdrc(float ref, float meas, float b0, float gain, float deadzone, int8_t &prevSign, Eso &eso);
    // Прямая подача LADRC по калибровке feedforward (режим _adrcFf)
    float _adrcFeedforward(float ref, float gain, float deadzone) const;

    // Безударный переход: подготовить состояние нового регулятора по последней скважности
    void _seedAdrc(Eso &eso, float ref, float meas, float uPrev, float b0, float gain, float deadzone);
    void _seedPi(uPID &pid, float ref, float meas, float uPrev, float gain, float deadzone, int16_t &frozenTicks);

    // Знак уставки с зоной нечувствительности около нуля: -1, 0, +1
    static int8_t _signOf(float r);

    // Фильтр уставки одного колеса: target -> ref; сообщает о ступеньке заданной скорости
    void _filterRef(float target, float &ref, float &prevTarget, int16_t &frozenTicks, float alpha);

    // Низкоуровневая запись на мотор
    void _motorWrite(uint8_t channel, float speed);

    // Функция обрезки
    float_t _cutValue(float_t min, float_t max, float_t value);
};

#endif
