#include "safety_manager.h"
#include "safety_policy.h"

void SafetyManager::check() {
  const bool host_silent = (millis() - ctx_.lastMotionMs > ctx_.commandTimeoutMs);
  if (SafetyPolicy::mustStop(host_silent, ctx_.inaOk, ctx_.pwmOk,
                             ctx_.busVoltage, ctx_.minVoltage) &&
      !ctx_.controller.isStopped()) {
    ctx_.controller.emergencyStop();
  }
  if (host_silent && !ctx_.watchdogLatched) {
    ++ctx_.watchdogStops;
    ctx_.watchdogLatched = true;
  }
  if (host_silent) {
    ctx_.telemetryEnabled = false;
  }
}
