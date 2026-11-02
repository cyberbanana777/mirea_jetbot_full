#ifndef SAFETY_POLICY_H
#define SAFETY_POLICY_H
#include <math.h>
// Fails closed if battery telemetry or motor PWM hardware is unavailable.
struct SafetyPolicy {
  static bool mustStop(bool commandTimedOut, bool inaOk, bool pwmOk,
                       float voltage, float minVoltage) {
    return commandTimedOut || !inaOk || !pwmOk ||
           !isfinite(voltage) || voltage < minVoltage;
  }
};
#endif
