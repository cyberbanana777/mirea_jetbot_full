#include <cassert>
#include <cstdio>
#include "command_parser.h"
#include "command_policy.h"
int main() {
  auto p=[](const char *s){return CommandParser::parse(s);};
  assert(p("$101;0.3;0.1;#").type==Command::SET_TWIST);
  assert(p("$1;0.3;0.1;#").type==Command::UNKNOWN);
  assert(p("$103;#").type==Command::STOP);
  assert(p("$202;#").type==Command::REQUEST_PID);
  assert(p("$500;2;#").type==Command::UNKNOWN);
  assert(p("$101;nan;0;#").type==Command::UNKNOWN);
  assert(CommandPolicy::validShape(p("$101;0;0;#")));
  assert(!CommandPolicy::validShape(p("$101;0;0;0;#")));
  puts("ESP32 v2 parser tests PASS");
}
