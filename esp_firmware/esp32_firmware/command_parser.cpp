#include "command_parser.h"
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {
Command decodeCommand(long id) {
  switch (id) {
    case 101: return Command::SET_TWIST;
    case 102: return Command::SET_WHEEL_SPEEDS;
    case 103: return Command::STOP;
    case 104: return Command::SET_DUTY;
    case 201: return Command::SET_PID_TEMP;
    case 202: return Command::REQUEST_PID;
    case 210: return Command::SET_PID_TX;
    case 211: return Command::REQUEST_PID_TX;
    case 203: return Command::SET_ADRC;
    case 204: return Command::SET_PID_FLASH;
    case 205: return Command::SET_FF;
    case 206: return Command::REQUEST_FF;
    case 207: return Command::SET_REF_FILTER;
    case 208: return Command::SET_CTRL_MODE;
    case 209: return Command::REQUEST_ADRC;
    case 403: return Command::SET_TELEMETRY;
    case 404: return Command::SET_SPEED_WINDOW;
    case 402: return Command::REQUEST_DIAGNOSTICS;
    case 501: return Command::REQUEST_INFO;
    default: return Command::UNKNOWN;
  }
}
}

ParsedCommand CommandParser::parse(const char* input) {
  ParsedCommand invalid;
  if (!input || input[0] != '$') return invalid;
  const char* p = input + 1;
  // strtol, unlike atoi, rejects non-numeric IDs and detects overflow.
  if (*p < '0' || *p > '9') return invalid;
  errno = 0;
  char* end = nullptr;
  const long id = strtol(p, &end, 10);
  if (errno == ERANGE || end == p || *end != ';') return invalid;
  const Command type = decodeCommand(id);
  if (type == Command::UNKNOWN) return invalid;
  p = end + 1;
  ParsedCommand parsed;
  parsed.type = type;
  while (*p != '#') {
    if (!*p) return invalid;
    if (parsed.count >= 7 || *p == ';') return invalid;
    errno = 0;
    const float value = strtof(p, &end);
    if (end == p || errno == ERANGE || !isfinite(value)) return invalid;
    if (*end != ';' && *end != '#') return invalid;
    parsed.args[parsed.count++] = value;
    p = end;
    if (*p == ';') ++p;
  }
  // A complete frame must end at '#', except for CR/LF whitespace.
  ++p;
  while (*p == '\r' || *p == '\n') ++p;
  if (*p != '\0') return invalid;
  return parsed;
}
