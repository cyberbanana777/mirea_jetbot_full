#ifndef SERIAL_SERVICE_H
#define SERIAL_SERVICE_H

#include <Arduino.h>
#include "serial_receiver.h"
#include "command_dispatcher.h"

// Non-owning references; the main application owns receivers, counters and state.
struct SerialServiceContext {
  SerialReceiver &mainReceiver;
  SerialReceiver &secondaryReceiver;
  CommandContext &commands;
  unsigned long &lastMotionMs;
  bool &watchdogLatched;
  uint32_t &accepted;
  uint32_t &rejected;
  bool &inaOk;
  bool &pwmOk;
  float &busVoltage;
  const float minVoltage;
  void (*secondaryHandler)(const char *);
};

class SerialService {
public:
  explicit SerialService(SerialServiceContext &context) : ctx_(context) {}
  void poll();
  void processCommands();
  void processSecondary();
private:
  SerialServiceContext &ctx_;
};

#endif
