// Границы длины кадра протокола v2 (риск R-11).
//
// Что проверяется:
//  1) Команда, которую вправе отправить хост, всегда помещается в приёмный
//     буфер SerialReceiver (BUFFER_SIZE - 1 = 127 байт). Иначе прошивка
//     молча отбросила бы кадр: serial_receiver.cpp при переполнении сбрасывает
//     приём, а счётчик rejected_commands такой кадр не пополняет.
//  2) Максимум полей команды совпадает с таблицей PROTOCOL.md и с реальным
//     CommandPolicy::validShape прошивки (число полей выводится из кода, а
//     не дублируется в тесте).
//  3) Кадр ровно в 127 байт принимается целиком и разбирается парсером.
//  4) Кадр в 128 байт отбрасывается, после него приём восстанавливается.
//
// Тест собирается без библиотеки uPID: нужны только serial_receiver.cpp и
// command_parser.cpp.

#include "command_parser.h"
#include "command_policy.h"
#include "serial_receiver.h"

#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool ok, const std::string& what)
{
  ++g_checks;
  if (!ok) {
    std::printf("FAIL: %s\n", what.c_str());
    ++g_failures;
  }
}

// Команды хост -> ESP32 по таблице "Jetson -> ESP32" в PROTOCOL.md.
// fields — максимальное число полей, которое принимает прошивка.
struct Spec {
  int id;
  int fields;
  const char* name;
};

const Spec kCommands[] = {
  {101, 2, "twist"},
  {102, 2, "wheel targets"},
  {103, 0, "stop"},
  {104, 2, "open loop"},
  {201, 6, "PID RAM"},
  {202, 0, "request PID"},
  {203, 6, "LADRC"},
  {204, 6, "PID + NVS"},
  {205, 5, "feedforward"},
  {206, 0, "request config"},
  {207, 2, "reference filter"},
  {208, 2, "control mode"},
  {209, 0, "request LADRC"},
  {210, 7, "transactional PID"},
  {211, 1, "PID request id"},
  {402, 0, "diagnostics"},
  {403, 1, "telemetry toggle"},
  {404, 2, "speed window"},
  {405, 0, "I2C diagnostics"},
  {501, 0, "protocol version"},
};

// Худшая длина одного числового поля:
//   format(value, '.9g') из esp_firmware/tools/uart_bench/protocol.py —
//   float64, 9 значащих цифр: "-1.00000028e+187" -> 16 символов.
constexpr size_t kWorstFieldWidth = 16;
// format(value, '.5f') из web_regulators_configurator при |v| <= 1000:
//   "-1000.00000" -> 11 символов.
constexpr size_t kFixedFieldWidth = 11;

std::string frame_of_fields(int id, size_t fields)
{
  std::string s = "$" + std::to_string(id) + ";";
  for (size_t i = 0; i < fields; ++i) {
    s += "1.0;";
  }
  s += "#";
  return s;
}

// Кадр ровно заданной длины: первое поле дополняется нулями (значение
// остаётся корректным float), остальные поля — "1.0".
std::string frame_of_length(int id, size_t fields, size_t total)
{
  const std::string prefix = "$" + std::to_string(id) + ";";
  const size_t base = prefix.size() + fields * 4 + 1;  // "1.0;" на каждое поле + '#'
  if (fields == 0 || total < base) {
    return frame_of_fields(id, fields);
  }
  // Базовый кадр отличается от нужного длиной первого поля: в базе оно
  // занимает 4 символа ("1.0;"), значит новое поле должно быть длиннее на extra.
  // Значение остаётся ровно 1.0 (дополнение нулями после точки), поэтому поле
  // остаётся валидным и как request_id, и как коэффициент ПИД.
  const size_t extra = total - base;
  const std::string first = "1." + std::string(extra + 1, '0');
  std::string s = prefix + first + ";";
  for (size_t i = 1; i < fields; ++i) {
    s += "1.0;";
  }
  s += "#";
  return s;
}

// Максимальное число полей, которое принимает прошивка для данного ID.
// Выводится из CommandPolicy::validShape, а не дублируется в тесте.
int accepted_field_count(int id)
{
  int best = -1;
  for (size_t n = 0; n <= 7; ++n) {
    const ParsedCommand parsed = CommandParser::parse(frame_of_fields(id, n).c_str());
    if (parsed.type == Command::UNKNOWN) {
      continue;
    }
    if (CommandPolicy::validShape(parsed) && static_cast<int>(n) > best) {
      best = static_cast<int>(n);
    }
  }
  return best;
}

void test_contract_limits()
{
  std::printf("ID  полей  худший кадр (байт)  запас  команда\n");
  size_t worst = 0;
  int worst_id = 0;
  for (const Spec& spec : kCommands) {
    const int fields = accepted_field_count(spec.id);
    check(fields >= 0, "ID " + std::to_string(spec.id) + " принимается прошивкой");
    check(fields == spec.fields,
          "ID " + std::to_string(spec.id) + ": максимум полей в коде " +
          std::to_string(fields) + ", в PROTOCOL.md " + std::to_string(spec.fields));

    // 1 + "$" ... "+ len(id) + ";" + поля + "#"
    const size_t len = 1 + std::to_string(spec.id).size() + 1 +
                       static_cast<size_t>(fields) * (kWorstFieldWidth + 1) + 1;
    std::printf("%-4d %-6d %-19zu %-6zu %s\n", spec.id, fields, len,
                SerialReceiver::BUFFER_SIZE - 1 - len, spec.name);
    check(len <= SerialReceiver::BUFFER_SIZE - 1,
          "кадр $" + std::to_string(spec.id) + " (" + std::to_string(len) +
          " байт) не помещается в буфер приёма (" +
          std::to_string(SerialReceiver::BUFFER_SIZE - 1) + ")");
    if (len > worst) {
      worst = len;
      worst_id = spec.id;
    }

    // Тот же кадр при фиксированном формате ".5f" (web_regulators_configurator).
    const size_t fixed = 1 + std::to_string(spec.id).size() + 1 +
                         static_cast<size_t>(fields) * (kFixedFieldWidth + 1) + 1;
    check(fixed <= SerialReceiver::BUFFER_SIZE - 1,
          "кадр $" + std::to_string(spec.id) + " в формате .5f не помещается в буфер приёма");
  }
  std::printf("Худший кадр: $%d, %zu байт из %zu доступных (запас %zu байт)\n",
              worst_id, worst, SerialReceiver::BUFFER_SIZE - 1,
              SerialReceiver::BUFFER_SIZE - 1 - worst);
  // Запас должен оставаться положительным: смена формата (например, на repr()
  // в Python) сразу упрётся в приёмный буфер прошивки.
  check(SerialReceiver::BUFFER_SIZE - 1 - worst >= 1,
        "запас в приёмном буфере меньше одного байта");
}

void test_frame_limit_behaviour()
{
  constexpr size_t kLimit = SerialReceiver::BUFFER_SIZE - 1;  // 127

  // Кадр ровно в предел принимается целиком.
  {
    const std::string frame = frame_of_length(210, 7, kLimit);
    check(frame.size() == kLimit, "тестовый кадр имеет ожидаемую длину");
    HardwareSerial serial;
    serial.rx = frame;
    SerialReceiver receiver(serial);
    receiver.update();
    check(receiver.available(), "кадр в 127 байт принят");
    const std::string got = receiver.getMessage();
    check(got == frame, "кадр в 127 байт принят без искажений");
    const ParsedCommand parsed = CommandParser::parse(got.c_str());
    check(parsed.type == Command::SET_PID_TX, "кадр в 127 байт разобран как $210");
    check(parsed.count == 7, "у $210 приняты все 7 полей");
  }

  // Кадр на байт длиннее отбрасывается.
  {
    const std::string too_long = frame_of_length(210, 7, kLimit + 1);
    check(too_long.size() == kLimit + 1, "тестовый кадр имеет ожидаемую длину");
    HardwareSerial serial;
    serial.rx = too_long;
    SerialReceiver receiver(serial);
    receiver.update();
    check(!receiver.available(), "кадр в 128 байт отброшен");
  }

  // После переполнения приём восстанавливается: длинный кадр не мешает
  // следующему короткому.
  {
    const std::string too_long = frame_of_length(210, 7, kLimit + 1);
    const std::string next = frame_of_fields(103, 0);
    HardwareSerial serial;
    serial.rx = too_long + next;
    SerialReceiver receiver(serial);
    receiver.update();
    check(receiver.available(), "после переполнения приём восстановлен");
    check(std::string(receiver.getMessage()) == next,
          "после переполнения принят следующий кадр целиком");
  }

  // Мусор перед '$' игнорируется, длинный кадр не «съедает» следующий.
  {
    const std::string frame = frame_of_length(210, 7, kLimit);
    HardwareSerial serial;
    serial.rx = "garbage" + frame;
    SerialReceiver receiver(serial);
    receiver.update();
    check(receiver.available(), "кадр после мусора принят");
    check(std::string(receiver.getMessage()) == frame, "кадр после мусора принят целиком");
  }

  // Восемь полей прошивка не принимает: предел 7 задан парсером.
  {
    const std::string frame = frame_of_fields(210, 8);
    const ParsedCommand parsed = CommandParser::parse(frame.c_str());
    check(parsed.type == Command::UNKNOWN, "кадр с 8 полями отвергнут парсером");
  }
}

}  // namespace

int main()
{
  test_contract_limits();
  test_frame_limit_behaviour();
  if (g_failures == 0) {
    std::printf("Границы длины кадра: PASS (%zu проверок)\n", static_cast<size_t>(g_checks));
    return 0;
  }
  std::printf("Границы длины кадра: FAIL (%d из %zu проверок)\n", g_failures,
              static_cast<size_t>(g_checks));
  return 1;
}
