#ifndef CONTROL_SCHEDULER_H
#define CONTROL_SCHEDULER_H

#include <Arduino.h>

// Scheduling decisions only. The caller keeps the original execution order.
// Arithmetic intentionally matches the validated v2.2.3 loop, including
// the skip-ahead policy after a missed control period.
class ControlScheduler {
public:
  static bool controlDue(unsigned long now_us, unsigned long &next_us,
                         unsigned long period_us) {
    if ((long)(now_us - next_us) < 0) return false;
    next_us += period_us;
    if ((long)(now_us - next_us) >= 0) {
      next_us = now_us + period_us;
    }
    return true;
  }

  static bool odometryDue(unsigned long now_us, unsigned long &last_us,
                          unsigned long period_us) {
    if ((unsigned long)(now_us - last_us) < period_us) return false;
    last_us = now_us;
    return true;
  }

  static bool i2cWindowOpen(unsigned long next_us, unsigned long min_window_us) {
    return (long)(next_us - micros()) > (long)min_window_us;
  }
};

#endif
