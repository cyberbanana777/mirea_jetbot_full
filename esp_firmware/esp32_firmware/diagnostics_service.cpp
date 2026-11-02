#include "command_dispatcher.h"
#include "command_policy.h"
#include "protocol_wrapper.h"
#include <math.h>

void handleDiagnosticsCommand(const ParsedCommand& cmd, DiagnosticsContext& ctx) {
  switch (cmd.type) {
    case Command::REQUEST_INFO:
      deviceInfoPublish();
      break;

    case Command::REQUEST_DIAGNOSTICS:
      diagnosticSnapshotPublish(millis(), ctx.accepted_commands,
          ctx.rejected_commands, ctx.watchdog_stop_count,
          ctx.controller.isStopped(), ctx.inaOk, ctx.pwmOk);
      break;

    default:
      break;
  }
}
