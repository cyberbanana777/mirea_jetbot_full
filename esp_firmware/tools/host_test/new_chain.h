// Цепочка "оценка скорости + регулятор" из прошивки для симуляций (повторяет controlStep() из .ino)
#pragma once
#include "Arduino.h"
#include "sim_run.h"
#include "speed_estimator.h"
#define private public
#include "motor_controller.h"
#undef private

static const float PI_F = 3.14159f;

struct NewChain {
  MotorController ctl;
  MTSpeedEstimator estL, estR;
  double periodUs = 10000;
  NewChain(const MTSpeedParams &sp, float kp, float ki, float kd, float ffGain, float ffDz)
      : ctl(0.1775f, 1.0f, 10.0f, kp, ki, kd, kp, ki, kd), estL(sp), estR(sp) {
    ctl.begin();
    ctl.setFeedforward(ffGain, ffDz, ffGain, ffDz);
  }
  void command(double ms) { ctl.setWheelSpeeds((float)ms, (float)ms); }
  void tick(uint32_t now, const EdgeSim &eL, const EdgeSim &eR) {
    // то же, что controlStep() в esp32_firmware.ino
    const float revR = 1.0f * estR.update(eR.edges, eR.tLast, now);
    const float revL = -1.0f * estL.update(eL.edges, eL.tLast, now);
    ctl.update(revL * 2 * PI_F * (float)WHEEL_R, revR * 2 * PI_F * (float)WHEEL_R);
  }
  double dutyR() { return ((double)ctl._pwm.pins[2] - ctl._pwm.pins[3]) / 4095.0; }
  double dutyL() { return ((double)ctl._pwm.pins[1] - ctl._pwm.pins[0]) / 4095.0; }  // левый мотор зеркальный
  double i2cWrites() { return (double)ctl._pwm.writes; }
};

