// command_parser.h
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>
#include <stdint.h>

// Типы команд. Значения enum — внутренние и НЕ совпадают с идентификаторами
// кадров в протоколе v2: соответствие задаётся decodeCommand() в command_parser.cpp.
// В комментариях указан wire-ID кадра ($…#).
enum class Command : uint8_t {
  UNKNOWN = 0,
  SET_TWIST = 1,         // $101;linear;angular;#  линейная + угловая скорость, м/с и рад/с
  SET_WHEEL_SPEEDS = 2,  // $102;v_L;v_R;#        скорости колёс, м/с
  SET_PID_TEMP = 3,      // $201;KpL;KiL;KdL;KpR;KiR;KdR;#  установить ПИД только в ОЗУ
  SET_PID_FLASH = 4,     // $204;KpL;KiL;KdL;KpR;KiR;KdR;#  установить ПИД и записать во flash
  REQUEST_PID = 5,       // $202;#                запросить текущие коэффициенты (ответ $202)
  // --- Калибровка и отладка контура скорости (см. tools/README_speed.md) ---
  SET_DUTY = 10,         // $104;duty_L;duty_R;#   разомкнутый режим: скважность ШИМ [-1, 1]
  SET_FF = 11,           // $205;gain_L;dz_L;gain_R;dz_R;save#  feedforward (save = 1 - записать во flash)
  SET_TELEMETRY = 12,    // $403;on;#              телеметрия $401 на 100 Гц, одометрия $301 на паузе
  REQUEST_FF = 13,       // $206;#                 запросить feedforward ($205), фильтр уставки ($207) и регулятор ($203)
  SET_REF_FILTER = 14,   // $207;tau_s;save#       постоянная времени фильтра уставки, с (save = 1 - во flash)
  // --- Выбор регулятора и LADRC ---
  SET_CTRL_MODE = 15,    // $208;mode;save#        0 - ПИ, 1 - LADRC (безударно, можно на ходу)
  SET_ADRC = 16,         // $203;wc;wo;b0_L;b0_R;ff;save#  параметры LADRC (ff: 0 нет, 1 мёртвая зона, 2 полный FF)
  REQUEST_ADRC = 17,     // $209;#                 запросить регулятор и параметры LADRC ($203)
  // --- Отладка ---
  SET_SPEED_WINDOW = 18, // $404;n;align#          окно оценки скорости
  STOP = 19,             // $103;#                 аварийная остановка
  REQUEST_DIAGNOSTICS = 20, // $402;#
  REQUEST_PID_TX = 22,  // $211;request_id;#
  SET_PID_TX = 23, // $210;request_id;KpL;KiL;KdL;KpR;KiR;KdR;# (RAM only)
  REQUEST_INFO = 21,        // $501;#
  REQUEST_I2C_DIAGNOSTICS = 24 // $405;# (read-only)

};

struct ParsedCommand {
  Command type = Command::UNKNOWN;
  float args[7] = { 0 };  // transactional PID needs request_id + 6 gains
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
