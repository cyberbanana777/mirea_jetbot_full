// Сквозной host-тест контракта «мост ↔ прошивка» для протокола v2.
//
// Проверяет обе стороны без ROS и без платы:
//   1) кадр, который мост отправляет в прошивку, разбирается парсером прошивки
//      (esp_firmware/esp32_firmware/command_parser.cpp) как ожидаемая команда;
//   2) кадр, который прошивка отправляет в мост, разбирается парсером моста
//      (serial_bridge_cpp/frame_parser.hpp) с ожидаемым числом полей.
//
// Сборка: см. esp_firmware/tools/host_test/Makefile (цель test_end_to_end_v2).

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "command_parser.h"
#include "command_policy.h"
#include "serial_bridge_cpp/frame_parser.hpp"
#include "serial_bridge_cpp/protocol.hpp"

using serial_bridge_cpp::format_stop;
using serial_bridge_cpp::format_twist;
using serial_bridge_cpp::parse_fields;
using serial_bridge_cpp::FrameParser;

namespace
{

// Мост -> прошивка: команда должна быть принята прошивкой как нужная команда.
void check_bridge_command(
  const std::string & frame, Command expected, bool watchdog_refreshes)
{
  const ParsedCommand parsed = CommandParser::parse(frame.c_str());
  assert(parsed.type == expected);
  assert(CommandPolicy::validShape(parsed));
  assert(
    CommandPolicy::refreshesMotionWatchdog(parsed.type) == watchdog_refreshes);
}

// Прошивка -> мост: кадр должен разбираться мостом с ожидаемым числом полей.
void check_firmware_frame(
  const std::string & frame, int expected_id, std::size_t expected_fields)
{
  std::vector<double> fields;
  assert(parse_fields(frame, fields));
  assert(fields.size() == expected_fields);
  assert(static_cast<int>(fields[0]) == expected_id);
}

}  // namespace

// --- 1. Команды моста принимаются прошивкой -------------------------------

static void test_bridge_twist_command()
{
  // Точный формат, который шлёт мост для /cmd_vel.
  const std::string frame = format_twist(0.25, -0.5);
  assert(frame == "$101;0.250;-0.500;#");

  const ParsedCommand parsed = CommandParser::parse(frame.c_str());
  assert(parsed.type == Command::SET_TWIST);
  assert(parsed.count == 2);
  assert(std::fabs(parsed.args[0] - 0.25f) < 1e-6f);
  assert(std::fabs(parsed.args[1] + 0.5f) < 1e-6f);
  assert(CommandPolicy::validShape(parsed));
  assert(CommandPolicy::validMotionValues(parsed));
  // SET_TWIST продлевает watchdog движения.
  assert(CommandPolicy::refreshesMotionWatchdog(parsed.type));
}

static void test_bridge_stop_command()
{
  // Кадр, который мост шлёт по таймауту /cmd_vel.
  const std::string frame = format_stop();
  assert(frame == "$103;#");

  const ParsedCommand parsed = CommandParser::parse(frame.c_str());
  assert(parsed.type == Command::STOP);
  assert(parsed.count == 0);
  assert(CommandPolicy::validShape(parsed));
  // STOP останавливает робота, но НЕ продлевает watchdog:
  // при молчании хоста прошивка всё равно сработает по своему таймеру.
  assert(!CommandPolicy::refreshesMotionWatchdog(parsed.type));
}

static void test_bridge_other_motion_commands()
{
  // Команды, которые может слать внешний издатель через serial/cmd.
  check_bridge_command("$102;0.10;0.12;#", Command::SET_WHEEL_SPEEDS, true);
  check_bridge_command("$104;0.50;-0.50;#", Command::SET_DUTY, true);
}

static void test_v1_ids_are_rejected()
{
  // Прошивка не понимает ID v1: мост не должен их отправлять.
  assert(CommandParser::parse("$1;0.25;-0.5;#").type == Command::UNKNOWN);
  assert(CommandParser::parse("$2;0.10;0.12;#").type == Command::UNKNOWN);
  assert(CommandParser::parse("$3;1;1;1;1;1;1;#").type == Command::UNKNOWN);
  assert(CommandParser::parse("$4;1;1;1;1;1;1;#").type == Command::UNKNOWN);
  assert(CommandParser::parse("$5;#").type == Command::UNKNOWN);
}

// --- 1a. Контракт кадров из tools/host_test/test_glue.cpp --------------------
//
// test_glue.cpp собирается только с библиотекой uPID (R-12), поэтому тот же
// набор строк проверяется здесь: без платы и без внешних зависимостей.

static void test_glue_command_frames()
{
  // Команды движения: принимаются и продлевают watchdog движения.
  check_bridge_command("$102;0.1;0.1;#", Command::SET_WHEEL_SPEEDS, true);
  check_bridge_command("$104;0.5;0.5;#", Command::SET_DUTY, true);
  check_bridge_command("$104;-0.3;0.3;#", Command::SET_DUTY, true);
  check_bridge_command("$104;2.0;0.0;#", Command::SET_DUTY, true);
  check_bridge_command("$101;0.3;0.0;#", Command::SET_TWIST, true);

  // Телеметрия: команда не продлевает watchdog.
  check_bridge_command("$403;1;#", Command::SET_TELEMETRY, false);
  check_bridge_command("$403;0;#", Command::SET_TELEMETRY, false);

  // Конфигурация: запросы и установки. Ни одна из них не продлевает watchdog.
  check_bridge_command("$202;#", Command::REQUEST_PID, false);
  check_bridge_command("$206;#", Command::REQUEST_FF, false);
  check_bridge_command("$209;#", Command::REQUEST_ADRC, false);
  check_bridge_command("$205;0.7;0.3;0.8;0.31;0;#", Command::SET_FF, false);
  check_bridge_command("$207;0.2;0#", Command::SET_REF_FILTER, false);
  check_bridge_command("$208;0;0#", Command::SET_CTRL_MODE, false);
  check_bridge_command("$203;6;30;10;8;1;0#", Command::SET_ADRC, false);
  check_bridge_command("$404;#", Command::SET_SPEED_WINDOW, false);
  check_bridge_command("$404;4;0;#", Command::SET_SPEED_WINDOW, false);
  check_bridge_command("$404;99;#", Command::SET_SPEED_WINDOW, false);

  // Нефинитное значение и мусор отвергаются парсером прошивки.
  assert(CommandParser::parse("$203;nan;30;10;8;0;0#").type == Command::UNKNOWN);
  assert(CommandParser::parse("$99;1;2;#garbage$7;1;#").type == Command::UNKNOWN);
  // ID v1 не принимаются (в test_glue.cpp это отдельная проверка шага 6).
  assert(CommandParser::parse("$1;0.2;0.0;#").type == Command::UNKNOWN);
  assert(CommandParser::parse("$5;#").type == Command::UNKNOWN);
}

static void test_glue_response_frames()
{
  // Ответы прошивки, которые ждёт test_glue.cpp: мост обязан их разобрать
  // (сейчас $203/$205/$207/$404 только логируются как известные без маппинга).
  std::vector<double> fields;

  assert(parse_fields("$205;0.70000;0.30000;0.80000;0.31000;#", fields));
  assert(fields.size() == 5);
  assert(static_cast<int>(fields[0]) == 205);

  assert(parse_fields("$207;0.20000;#", fields));
  assert(fields.size() == 2);
  assert(static_cast<int>(fields[0]) == 207);

  assert(parse_fields("$203;0;6.000;30.000;10.0000;8.0000;1;#", fields));
  assert(fields.size() == 7);
  assert(static_cast<int>(fields[0]) == 203);

  assert(parse_fields("$404;2;1;#", fields));
  assert(fields.size() == 3);
  assert(static_cast<int>(fields[0]) == 404);
}

// --- 2. Кадры прошивки принимаются мостом ---------------------------------

static void test_firmware_odometry_frame()
{
  // Формат protocol_wrapper.cpp: odomPublish().
  check_firmware_frame(
    "$301;0.12345;-1.50000;3.14159;0.20000;-0.10000;0.15000;0.25000;#",
    serial_bridge_cpp::MSG_ODOMETRY, serial_bridge_cpp::ODOMETRY_FIELDS);
}

static void test_firmware_pid_frame()
{
  // Формат protocol_wrapper.cpp: regulatorsCoefficientsPublish().
  check_firmware_frame(
    "$202;0.85000;1.50000;0.05000;0.80000;1.50000;0.04000;#",
    serial_bridge_cpp::MSG_PID, serial_bridge_cpp::PID_FIELDS);
}

static void test_firmware_frame_field_count()
{
  // parse_fields разбирает поля, но не проверяет их число: это делает узел
  // моста, требуя ТОЧНОЕ число полей для каждого типа кадра.
  std::vector<double> fields;
  assert(parse_fields("$301;0.1;0.2;0.3;0.4;0.5;0.6;0.7;#", fields));
  assert(fields.size() == serial_bridge_cpp::ODOMETRY_FIELDS);

  // Нехватка поля: мост такой кадр не публикует.
  assert(parse_fields("$202;1;2;3;4;5;#", fields));
  assert(fields.size() != serial_bridge_cpp::PID_FIELDS);
  assert(fields.size() == serial_bridge_cpp::PID_FIELDS - 1);

  // Избыток поля: тоже не совпадает с форматом.
  assert(parse_fields("$301;1;2;3;4;5;6;7;8;#", fields));
  assert(fields.size() != serial_bridge_cpp::ODOMETRY_FIELDS);
  assert(fields.size() == serial_bridge_cpp::ODOMETRY_FIELDS + 1);
}

static void test_non_finite_values_rejected()
{
  // Мост не должен публиковать nan/inf в одометрии и ПИД.
  std::vector<double> fields;
  assert(!parse_fields("$301;nan;#", fields));
  assert(!parse_fields("$301;inf;#", fields));
  assert(!parse_fields("$202;-inf;1;1;1;1;1;#", fields));
}

static void test_odometry_frame_exact_field_count()
{
  // Эталонный кадр одометрии: ровно 8 полей (тип + 7 значений).
  std::vector<double> fields;
  assert(parse_fields("$301;0.1;0.2;0.3;0.4;0.5;0.6;0.7;#", fields));
  assert(fields.size() == serial_bridge_cpp::ODOMETRY_FIELDS);
}

// --- 3. Поток UART целиком (парсер моста) -----------------------------------

static void test_stream_of_frames()
{
  FrameParser parser;
  std::vector<std::string> frames;
  const std::string data =
    "$301;0.00000;0.00000;0.00000;0.00000;0.00000;0.00000;0.00000;#\r\n"
    "$202;0.85000;1.50000;0.05000;0.80000;1.50000;0.04000;#\r\n"
    "$401;1000;0.1000;0.1000;0.100;0.1000;0.1000;0.100;#\r\n";
  parser.feed(
    data.data(), data.size(),
    [&](const std::string & f) {frames.push_back(f);});
  assert(frames.size() == 3);
  assert(frames[0].compare(0, 4, "$301") == 0);
  assert(frames[1].compare(0, 4, "$202") == 0);
  assert(frames[2].compare(0, 4, "$401") == 0);

  // Каждый из них должен разбираться мостом.
  for (const std::string & frame : frames) {
    std::vector<double> fields;
    assert(parse_fields(frame, fields));
  }
}

static void test_raw_motion_id_detection()
{
  // Мост отличает raw-кадры движения от прочих: по ним отменяется таймер
  // простоя /cmd_vel.
  assert(serial_bridge_cpp::raw_frame_id("$101;0.1;0.2;#") == 101);
  assert(serial_bridge_cpp::raw_frame_id("$102;0.1;0.2;#") == 102);
  assert(serial_bridge_cpp::raw_frame_id("$104;0.1;0.2;#") == 104);
  assert(serial_bridge_cpp::raw_frame_id("$103;#") == 103);
  assert(serial_bridge_cpp::raw_frame_id("$202;#") == 202);
  assert(serial_bridge_cpp::raw_frame_id("текст") == -1);
  assert(serial_bridge_cpp::raw_frame_id("$abc;1;#") == -1);

  // Только 101, 102 и 104 продлевают watchdog движения.
  assert(serial_bridge_cpp::is_motion_id(101));
  assert(serial_bridge_cpp::is_motion_id(102));
  assert(serial_bridge_cpp::is_motion_id(104));
  assert(!serial_bridge_cpp::is_motion_id(103));
  assert(!serial_bridge_cpp::is_motion_id(202));
}

int main()
{
  test_bridge_twist_command();
  test_bridge_stop_command();
  test_bridge_other_motion_commands();
  test_v1_ids_are_rejected();
  test_glue_command_frames();
  test_glue_response_frames();

  test_firmware_odometry_frame();
  test_firmware_pid_frame();
  test_firmware_frame_field_count();
  test_non_finite_values_rejected();
  test_odometry_frame_exact_field_count();

  test_stream_of_frames();
  test_raw_motion_id_detection();

  puts("End-to-end v2 contract tests PASS");
  return 0;
}
