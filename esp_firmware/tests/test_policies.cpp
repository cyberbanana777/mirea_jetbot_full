// Прямые host-тесты «чистых» политик прошивки.
//
// Проверяются две политики, не зависящие от оборудования:
//   * command_policy.h — форма команды (число аргументов), диапазоны ПИД и
//     request_id, правило продления watchdog движения;
//   * safety_policy.h  — SafetyPolicy::mustStop, условие аварийной
//     остановки (fail-closed).
//
// До этого файла политики проверялись только косвенно: validMotionValues
// упоминалась в тестах ровно один раз (test_end_to_end_v2.cpp), а mustStop,
// validPid, validRequestId, validTransactionalPid и refreshesMotionWatchdog
// не встречались ни в одном тесте.
//
// Контракт, который фиксирует тест, описан в esp_firmware/PROTOCOL.md:
// таблица «Jetson -> ESP32» (число аргументов), раздел «Safety» (watchdog
// продлевают только ID 101/102/104; проверка формы идёт до обновления
// watchdog) и раздел «v2.2 transactional PID» (request_id — целое 1..65535,
// коэффициенты ПИД — конечные, 0..1000, иначе NACK с ошибкой 1).
//
// Сборка: esp_firmware/tools/host_test/Makefile, цель test_policies.
// Плата, ROS 2 и библиотека uPID не нужны.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "command_parser.h"
#include "command_policy.h"
#include "safety_policy.h"

namespace
{

const float kNaN = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();

// Команда с заданным числом аргументов. Первый аргумент равен 1.0
// (корректный request_id), остальные — 0.5: так одна таблица покрывает и
// обычные команды, и транзакционные ($210/$211), где args[0] — request_id.
ParsedCommand with_args(Command type, int count)
{
  ParsedCommand cmd;
  cmd.type = type;
  cmd.count = 0;
  for (int i = 0; i < count; ++i) {
    assert(i < 7);
    cmd.args[cmd.count++] = (i == 0) ? 1.0f : 0.5f;
  }
  return cmd;
}

// --- 1. SafetyPolicy::mustStop — условие аварийной остановки ---------------

void test_must_stop_fail_closed()
{
  // Штатный режим: связь есть, датчики исправны, заряд выше минимума.
  assert(!SafetyPolicy::mustStop(false, true, true, 12.0f, 10.0f));
  // Ровно минимальное напряжение — ещё не повод останавливаться.
  assert(!SafetyPolicy::mustStop(false, true, true, 10.0f, 10.0f));

  // Каждое отдельное нарушение останавливает робота (fail-closed).
  assert(SafetyPolicy::mustStop(true, true, true, 12.0f, 10.0f));   // таймаут
  assert(SafetyPolicy::mustStop(false, false, true, 12.0f, 10.0f));  // INA
  assert(SafetyPolicy::mustStop(false, true, false, 12.0f, 10.0f));  // ШИМ
  assert(SafetyPolicy::mustStop(false, true, true, 9.99f, 10.0f));   // заряд
  assert(SafetyPolicy::mustStop(false, true, true, 0.0f, 10.0f));     // заряд
  assert(SafetyPolicy::mustStop(false, true, true, -1.0f, 10.0f));    // заряд

  // Нечисловой или бесконечный заряд — тоже повод остановиться.
  assert(SafetyPolicy::mustStop(false, true, true, kNaN, 10.0f));
  assert(SafetyPolicy::mustStop(false, true, true, kInf, 10.0f));
  assert(SafetyPolicy::mustStop(false, true, true, -kInf, 10.0f));

  // Комбинация нарушений: результат не зависит от их числа.
  assert(SafetyPolicy::mustStop(true, false, false, kNaN, 10.0f));
}

// --- 2. CommandPolicy::validPid — диапазон коэффициентов ------------------

void test_valid_pid()
{
  // Границы диапазона 0..1000 включительно.
  assert(CommandPolicy::validPid(0, 0, 0, 0, 0, 0));
  assert(CommandPolicy::validPid(1000, 1000, 1000, 1000, 1000, 1000));
  assert(CommandPolicy::validPid(0.85f, 1.5f, 0.05f, 0.8f, 1.5f, 0.04f));

  // Любое из шести полей вне диапазона отвергает команду целиком.
  const float bad[] = {-0.001f, 1000.001f, kNaN, kInf, -kInf};
  for (float value : bad) {
    assert(!CommandPolicy::validPid(value, 1, 1, 1, 1, 1));
    assert(!CommandPolicy::validPid(1, value, 1, 1, 1, 1));
    assert(!CommandPolicy::validPid(1, 1, value, 1, 1, 1));
    assert(!CommandPolicy::validPid(1, 1, 1, value, 1, 1));
    assert(!CommandPolicy::validPid(1, 1, 1, 1, value, 1));
    assert(!CommandPolicy::validPid(1, 1, 1, 1, 1, value));
  }
}

// --- 3. CommandPolicy::validRequestId --------------------------------------

void test_valid_request_id()
{
  // request_id — целое число от 1 до 65535 (uint16 в ESP32).
  assert(CommandPolicy::validRequestId(1.0f));
  assert(CommandPolicy::validRequestId(42.0f));
  assert(CommandPolicy::validRequestId(65535.0f));

  assert(!CommandPolicy::validRequestId(0.0f));
  assert(!CommandPolicy::validRequestId(65536.0f));
  assert(!CommandPolicy::validRequestId(-1.0f));
  assert(!CommandPolicy::validRequestId(1.5f));
  assert(!CommandPolicy::validRequestId(2.0001f));
  assert(!CommandPolicy::validRequestId(kNaN));
  assert(!CommandPolicy::validRequestId(kInf));
}

// --- 4. CommandPolicy::validTransactionalPid --------------------------------

void test_valid_transactional_pid()
{
  // $210: request_id + шесть коэффициентов.
  ParsedCommand ok = with_args(Command::SET_PID_TX, 7);
  ok.args[0] = 42.0f;
  assert(CommandPolicy::validTransactionalPid(ok));
  assert(CommandPolicy::validShape(ok));

  // Неверный request_id: команда невалидна и на уровне формы.
  ParsedCommand bad_id = with_args(Command::SET_PID_TX, 7);
  bad_id.args[0] = 0.0f;
  assert(!CommandPolicy::validTransactionalPid(bad_id));
  assert(!CommandPolicy::validShape(bad_id));

  // Неверный коэффициент при верном request_id.
  ParsedCommand bad_gain = with_args(Command::SET_PID_TX, 7);
  bad_gain.args[3] = 1001.0f;
  assert(!CommandPolicy::validTransactionalPid(bad_gain));

  // Неверное число аргументов (нужно ровно семь).
  assert(!CommandPolicy::validTransactionalPid(with_args(Command::SET_PID_TX, 6)));
  assert(!CommandPolicy::validTransactionalPid(with_args(Command::SET_PID_TX, 5)));
}

// --- 5. CommandPolicy::refreshesMotionWatchdog ----------------------------

void test_refreshes_motion_watchdog()
{
  // Watchdog движения продлевают только команды движения (ID 101/102/104).
  assert(CommandPolicy::refreshesMotionWatchdog(Command::SET_TWIST));
  assert(CommandPolicy::refreshesMotionWatchdog(Command::SET_WHEEL_SPEEDS));
  assert(CommandPolicy::refreshesMotionWatchdog(Command::SET_DUTY));

  // Остановка и любые запросы/настройки watchdog не продлевают.
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::STOP));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_TELEMETRY));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::REQUEST_PID));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::REQUEST_PID_TX));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::REQUEST_FF));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::REQUEST_ADRC));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::REQUEST_DIAGNOSTICS));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::REQUEST_INFO));
  assert(!CommandPolicy::refreshesMotionWatchdog(
    Command::REQUEST_I2C_DIAGNOSTICS));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_PID_TEMP));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_PID_FLASH));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_PID_TX));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_FF));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_ADRC));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_REF_FILTER));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_CTRL_MODE));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::SET_SPEED_WINDOW));
  assert(!CommandPolicy::refreshesMotionWatchdog(Command::UNKNOWN));
}

// --- 6. CommandPolicy::validMotionValues -----------------------------------

void test_valid_motion_values()
{
  const ParsedCommand twist = with_args(Command::SET_TWIST, 2);
  assert(CommandPolicy::validMotionValues(twist));

  // Нечисловые значения отвергаются: до watchdog дело не доходит.
  ParsedCommand nan_cmd = with_args(Command::SET_TWIST, 2);
  nan_cmd.args[0] = kNaN;
  assert(!CommandPolicy::validMotionValues(nan_cmd));

  ParsedCommand inf_cmd = with_args(Command::SET_TWIST, 2);
  inf_cmd.args[1] = kInf;
  assert(!CommandPolicy::validMotionValues(inf_cmd));

  // Неверное число аргументов.
  assert(!CommandPolicy::validMotionValues(with_args(Command::SET_TWIST, 1)));
  assert(!CommandPolicy::validMotionValues(with_args(Command::SET_TWIST, 3)));

  // Команда, не продлевающая watchdog, motion-значений не несёт.
  assert(!CommandPolicy::validMotionValues(with_args(Command::SET_TELEMETRY, 2)));

  // R-07 (решение владельца от 2026-11-11): диапазон motion-значений НЕ
  // проверяется. Команда с конечным, но заведомо out-of-range значением
  // принимается политикой и обрезается позже, в motor_controller.
  // Тест фиксирует именно это (осознанное) поведение: если диапазон
  // проверки решит добавить, тест нужно будет изменить вместе с PROTOCOL.md.
  ParsedCommand out_of_range = with_args(Command::SET_DUTY, 2);
  out_of_range.args[0] = 5.0f;
  out_of_range.args[1] = -5.0f;
  assert(CommandPolicy::validMotionValues(out_of_range));
}

// --- 7. CommandPolicy::validShape — число аргументов каждой команды -------
//
// Таблица сверена с PROTOCOL.md, раздел «Jetson -> ESP32».

struct ShapeCase
{
  Command type;
  int id;
  int min_args;
  int max_args;
};

const ShapeCase kShapeCases[] = {
  {Command::SET_TWIST, 101, 2, 2},
  {Command::SET_WHEEL_SPEEDS, 102, 2, 2},
  {Command::STOP, 103, 0, 0},
  {Command::SET_DUTY, 104, 2, 2},
  {Command::SET_PID_TEMP, 201, 6, 6},
  {Command::REQUEST_PID, 202, 0, 0},
  {Command::SET_ADRC, 203, 4, 6},
  {Command::SET_PID_FLASH, 204, 6, 6},
  {Command::SET_FF, 205, 4, 5},
  {Command::REQUEST_FF, 206, 0, 0},
  {Command::SET_REF_FILTER, 207, 1, 2},
  {Command::SET_CTRL_MODE, 208, 1, 2},
  {Command::REQUEST_ADRC, 209, 0, 0},
  {Command::SET_PID_TX, 210, 7, 7},
  {Command::REQUEST_PID_TX, 211, 1, 1},
  {Command::REQUEST_DIAGNOSTICS, 402, 0, 0},
  {Command::SET_TELEMETRY, 403, 1, 1},
  {Command::SET_SPEED_WINDOW, 404, 0, 2},
  {Command::REQUEST_I2C_DIAGNOSTICS, 405, 0, 0},
  {Command::REQUEST_INFO, 501, 0, 0},
};

void test_shape_accepts_exactly_documented_field_count()
{
  for (const ShapeCase & c : kShapeCases) {
    for (int n = 0; n <= 7; ++n) {
      const bool expected = (n >= c.min_args && n <= c.max_args);
      const bool actual = CommandPolicy::validShape(with_args(c.type, n));
      if (actual != expected) {
        fprintf(stderr,
                "FAIL $%d: %d аргументов — ожидалось %s, получено %s\n",
                c.id, n, expected ? "принять" : "отклонить",
                actual ? "принято" : "отклонено");
        abort();
      }
    }
  }

  // Неизвестная команда не принимается ни с каким числом аргументов.
  for (int n = 0; n <= 7; ++n) {
    assert(!CommandPolicy::validShape(with_args(Command::UNKNOWN, n)));
  }
}

// --- 8. Форма команды на настоящих кадрах ----------------------------------

void test_shape_on_real_frames()
{
  auto shape = [](const char * frame) {
    return CommandPolicy::validShape(CommandParser::parse(frame));
  };

  // Команды движения.
  assert(shape("$101;0.3;0.1;#"));
  assert(!shape("$101;0.3;#"));
  assert(!shape("$101;0.3;0.1;0.2;#"));
  assert(shape("$102;0.10;0.12;#"));
  assert(!shape("$102;0.10;#"));
  assert(shape("$104;0.50;-0.50;#"));
  assert(!shape("$104;#"));

  // Остановка: аргументов нет.
  assert(shape("$103;#"));
  assert(!shape("$103;0;#"));

  // Конфигурация ПИД: ровно шесть коэффициентов.
  assert(shape("$201;0.85;1.5;0.05;0.8;1.5;0.04;#"));
  assert(!shape("$201;0.85;1.5;0.05;0.8;1.5;#"));
  assert(shape("$204;0.85;1.5;0.05;0.8;1.5;0.04;#"));
  assert(!shape("$204;0.85;1.5;0.05;0.8;1.5;0.04;1;#"));

  // LADRC: от четырёх до шести аргументов.
  assert(shape("$203;6;30;10;8;#"));
  assert(shape("$203;6;30;10;8;1;#"));
  assert(shape("$203;6;30;10;8;1;0;#"));
  assert(!shape("$203;6;30;10;#"));
  assert(!shape("$203;6;30;10;8;1;0;1;#"));

  // Feedforward: четыре или пять аргументов.
  assert(shape("$205;0.7;0.3;0.8;0.31;#"));
  assert(shape("$205;0.7;0.3;0.8;0.31;1;#"));
  assert(!shape("$205;0.7;0.3;0.8;#"));
  assert(!shape("$205;0.7;0.3;0.8;0.31;1;1;#"));

  // Фильтр уставки и режим регулятора: один или два аргумента.
  assert(shape("$207;0.2;#"));
  assert(shape("$207;0.2;1;#"));
  assert(!shape("$207;#"));
  assert(shape("$208;0;#"));
  assert(shape("$208;0;1;#"));
  assert(!shape("$208;#"));

  // Транзакционные команды: request_id обязателен и корректен.
  assert(shape("$210;7;0.85;1.5;0.05;0.8;1.5;0.04;#"));
  assert(!shape("$210;0;0.85;1.5;0.05;0.8;1.5;0.04;#"));
  assert(!shape("$210;7;0.85;1.5;0.05;0.8;1.5;#"));
  assert(shape("$211;7;#"));
  assert(!shape("$211;0;#"));
  assert(!shape("$211;#"));

  // Окно оценки скорости: ноль, одно или два аргумента.
  assert(shape("$404;#"));
  assert(shape("$404;4;#"));
  assert(shape("$404;4;1;#"));
  assert(!shape("$404;4;1;2;#"));

  // Телеметрия и запросы без аргументов.
  assert(shape("$403;1;#"));
  assert(!shape("$403;#"));
  assert(shape("$402;#"));
  assert(shape("$405;#"));
  assert(shape("$501;#"));
  assert(!shape("$402;1;#"));
}

}  // namespace

int main()
{
  test_must_stop_fail_closed();
  test_valid_pid();
  test_valid_request_id();
  test_valid_transactional_pid();
  test_refreshes_motion_watchdog();
  test_valid_motion_values();
  test_shape_accepts_exactly_documented_field_count();
  test_shape_on_real_frames();

  puts("ESP32 v2 policy tests PASS");
  return 0;
}
