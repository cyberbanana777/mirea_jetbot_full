#include "command_dispatcher.h"
#include "command_policy.h"
#include "protocol_wrapper.h"
#include "config_service.h"
#include "diagnostics_service.h"
#include <math.h>

void dispatchCommand(const ParsedCommand& cmd, CommandContext& ctx) {
    switch (cmd.type)
    {
    case Command::STOP:
      ctx.motion.controller.emergencyStop();
      break;

    case Command::SET_TWIST:
      if (cmd.count >= 2)
        ctx.motion.controller.setTargetVelocity(cmd.args[0], cmd.args[1]);
      break;

    case Command::SET_WHEEL_SPEEDS:
      if (cmd.count >= 2)
        ctx.motion.controller.setWheelSpeeds(cmd.args[0], cmd.args[1]);
      break;

    case Command::SET_DUTY:
      // Разомкнутый режим для калибровки: шлите не реже раза в секунду,
      // иначе сработает аварийная остановка
      if (cmd.count >= 2 && isfinite(cmd.args[0]) && isfinite(cmd.args[1]))
        ctx.motion.controller.setOpenLoop(cmd.args[0], cmd.args[1]);
      break;

    case Command::SET_TELEMETRY:
      if (cmd.count >= 1)
      {
        ctx.motion.telemetry_on = (cmd.args[0] >= 0.5f);
        ctx.motion.telemetry_div = 0;
      }
      break;

    case Command::SET_PID_TEMP:
    case Command::SET_PID_FLASH:
    case Command::REQUEST_PID:
    case Command::SET_FF:
    case Command::REQUEST_FF:
    case Command::SET_REF_FILTER:
    case Command::SET_CTRL_MODE:
    case Command::SET_ADRC:
    case Command::REQUEST_ADRC:
    case Command::SET_SPEED_WINDOW:
    case Command::REQUEST_PID_TX:
    case Command::SET_PID_TX:
      handleConfigCommand(cmd, ctx.config);
      break;

    case Command::REQUEST_INFO:
    case Command::REQUEST_DIAGNOSTICS:
      handleDiagnosticsCommand(cmd, ctx.diagnostics);
      break;

    case Command::UNKNOWN:
      break;
    }
 }
