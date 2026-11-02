#ifndef CONFIG_SERVICE_H
#define CONFIG_SERVICE_H
struct ParsedCommand;
struct ConfigContext;
void handleConfigCommand(const ParsedCommand& cmd, ConfigContext& ctx);
#endif
