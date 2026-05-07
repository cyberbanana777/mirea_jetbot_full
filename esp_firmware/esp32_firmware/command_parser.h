// command_parser.h
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include <stdint.h>

// Типы команд (соответствуют msg_type 1..5)
enum class Command : uint8_t {
  UNKNOWN = 0,
  SET_TWIST = 1,         // линейная + угловая скорость
  SET_WHEEL_SPEEDS = 2,  // скорости колёс
  SET_PID_TEMP = 3,      // установить ПИД только в ОЗУ
  SET_PID_FLASH = 4,     // установить ПИД и записать во flash
  REQUEST_PID = 5        // запросить текущие коэффициенты
};

struct ParsedCommand {
  Command type = Command::UNKNOWN;
  float args[6] = { 0 };  // максимум 6 аргументов (для PID)
  uint8_t count = 0;      // реальное количество полученных аргументов
};

class CommandParser {
public:
  /**
     * Разбирает строку протокола, заполняет ParsedCommand.
     * @param input строка вида "$<type>;<arg1>;...;<argN>;#"
     * @return заполненная структура
     */
  static ParsedCommand parse(const char* input);
  static ParsedCommand parse(const String& input) {
    return parse(input.c_str());
  }
};

#endif