// command_parser.h
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include <stdint.h>

// Типы команд (соответствуют msg_type)
enum class Command : uint8_t {
  UNKNOWN = 0,
  SET_TWIST = 1,         // линейная + угловая скорость
  SET_WHEEL_SPEEDS = 2,  // скорости колёс
  SET_PID_TEMP = 3,      // установить ПИД только в ОЗУ
  SET_PID_FLASH = 4,     // установить ПИД и записать во flash
  REQUEST_PID = 5,       // запросить текущие коэффициенты
  // --- Калибровка и отладка контура скорости (см. tools/README_speed.md) ---
  SET_DUTY = 10,         // $10;duty_L;duty_R;#   разомкнутый режим: скважность ШИМ [-1, 1]
  SET_FF = 11,           // $11;gain_L;dz_L;gain_R;dz_R;save#  feedforward (save = 1 - записать во flash)
  SET_TELEMETRY = 12,    // $12;on;#              телеметрия $3 на 100 Гц, одометрия $1 на паузе
  REQUEST_FF = 13,       // $13;#                 запросить feedforward ($4), фильтр уставки ($6) и регулятор ($7)
  SET_REF_FILTER = 14,   // $14;tau_s;save#       постоянная времени фильтра уставки, с (save = 1 - во flash)
  // --- Выбор регулятора и LADRC ---
  SET_CTRL_MODE = 15,    // $15;mode;save#        0 - ПИ, 1 - LADRC (безударно, можно на ходу)
  SET_ADRC = 16,         // $16;wc;wo;b0_L;b0_R;ff;save#  параметры LADRC (ff: 0 нет, 1 мёртвая зона, 2 полный FF)
  REQUEST_ADRC = 17,     // $17;#                 запросить регулятор и параметры LADRC ($7)
  // --- Отладка ---
  SET_SPEED_WINDOW = 18  // $18;n;align#          окно оценки скорости: минимум фронтов (1..8), align 1 - целые метки; до перезагрузки; $18;# - запрос. Ответ $8
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
