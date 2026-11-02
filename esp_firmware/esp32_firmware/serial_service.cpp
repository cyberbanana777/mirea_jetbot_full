#include "serial_service.h"
#include "command_policy.h"
#include "safety_policy.h"

void SerialService::poll() {
  ctx_.mainReceiver.update();
  ctx_.secondaryReceiver.update();
}

void SerialService::processCommands() {
  for (uint8_t frames = 0; frames < 4 && ctx_.mainReceiver.available(); ++frames) {
    const char *raw = ctx_.mainReceiver.getMessage();
    const ParsedCommand cmd = CommandParser::parse(raw);
    if (!CommandPolicy::validShape(cmd)) {
      ++ctx_.rejected;
      ctx_.mainReceiver.update();
      continue;
    }
    if (CommandPolicy::refreshesMotionWatchdog(cmd.type)) {
      if (SafetyPolicy::mustStop(false, ctx_.inaOk, ctx_.pwmOk,
                                 ctx_.busVoltage, ctx_.minVoltage) ||
          !CommandPolicy::validMotionValues(cmd)) {
        ++ctx_.rejected;
        ctx_.mainReceiver.update();
        continue;
      }
      ctx_.lastMotionMs = millis();
      ctx_.watchdogLatched = false;
    }
    ++ctx_.accepted;
    dispatchCommand(cmd, ctx_.commands);
    ctx_.mainReceiver.update();
  }
}

void SerialService::processSecondary() {
  if (!ctx_.secondaryReceiver.available()) return;
  const char *raw = ctx_.secondaryReceiver.getMessage();
  ctx_.secondaryHandler(raw);
  ctx_.secondaryReceiver.update();
}
