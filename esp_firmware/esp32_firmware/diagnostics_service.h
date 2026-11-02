#ifndef DIAGNOSTICS_SERVICE_H
#define DIAGNOSTICS_SERVICE_H
struct ParsedCommand;
struct DiagnosticsContext;
void handleDiagnosticsCommand(const ParsedCommand& cmd, DiagnosticsContext& ctx);
#endif
