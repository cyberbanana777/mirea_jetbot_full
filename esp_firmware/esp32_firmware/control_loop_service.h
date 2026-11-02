#ifndef CONTROL_LOOP_SERVICE_H
#define CONTROL_LOOP_SERVICE_H

#include <Arduino.h>
#include "encoders.h"
#include "speed_estimator.h"
#include "motor_controller.h"
#include "odometry.h"

// References to application-owned state. No time scheduling or safety decisions here.
struct ControlLoopContext {
  Encoders &encoders;
  MTSpeedEstimator &speedR;
  MTSpeedEstimator &speedL;
  MotorController &controller;
  DiffDriveOdometry &odometry;
  bool &telemetryOn;
  uint8_t &telemetryDiv;
  float &linearVelocity;
  float &angularVelocity;
  float &leftVelocity;
  float &rightVelocity;
};

class ControlLoopService {
public:
  explicit ControlLoopService(ControlLoopContext &ctx) : ctx_(ctx) {}
  void controlStep();
  void odometryStep();
private:
  static float convertTicksToFreq(long ticks, long timeout_micros, char dir);
  static float convertTicksToDistance(long ticks, char dir);
  ControlLoopContext &ctx_;
};

#endif
