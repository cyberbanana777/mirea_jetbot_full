// Сквозная проверка "склейки" прошивки: компилируется настоящий esp32_firmware.ino, на заглушках
// вызываются setup()/loop() с виртуальным временем, команды подаются через Serial.
// Проверяется: команды $2/$10/$11/$12/$13, телеметрия 100 Гц, пауза одометрии, аварийная остановка по тишине.
#include "Arduino.h"
#include <stdio.h>
#include <string>
#include <functional>
#include <algorithm>
#include <vector>
#include <stdlib.h>
#include "GyverOLED.h"
#define private public
#include "motor_controller.h"
#undef private

void setup();
void loop();
extern MotorController controller;
extern bool telemetry_on;
extern float FF_GAIN_L, FF_DEADZONE_L;
extern float KP_L, KI_L, KD_L, KP_R, KI_R, KD_R;
extern unsigned long control_next_us;
static long g_minWindow = 1L << 30;
static int g_chunks = 0;
static void onChunk() { ++g_chunks; g_minWindow = std::min(g_minWindow, (long)(control_next_us - g_micros)); g_micros += 1700; /* кусок 64 байта ~1.7 мс на 400 кГц */ }

static int failures = 0;
static void check(bool ok, const char *what) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what); if (!ok) ++failures; }

static void run(double seconds) {   // виртуальное время, шаг 50 мкс
  const unsigned long end = g_micros + (unsigned long)(seconds * 1e6);
  while (g_micros < end) { loop(); g_micros += 50; }
}
static void cmd(const char *s) { Serial.rx += s; }
static size_t count(const std::string &hay, const char *needle) {
  size_t n = 0, p = 0; while ((p = hay.find(needle, p)) != std::string::npos) { ++n; p += strlen(needle); } return n;
}

int main() {
  g_micros = 1000000;
  setup();
  run(0.5);   // INA219 прочитан (12 В), восстановление после старта

  printf("1) $2 - уставка 0.1 м/с на колёсах, энкодеры стоят: регулятор должен дать тягу вперёд\n");
  Serial.tx.clear();
  cmd("$2;0.1;0.1;#");
  run(0.3);
  check(!controller.isOpenLoop(), "замкнутый режим");
  check(controller.getTargetLeft() > 0.09f && controller.getTargetRight() > 0.09f, "уставки приняты");
  check(controller._pwm.pins[2] > 1000 && controller._pwm.pins[3] == 0, "правый мотор: ШИМ вперёд, назад выключен");
  check(controller._pwm.pins[1] > 1000 && controller._pwm.pins[0] == 0, "левый мотор (инверсия): ШИМ на втором канале");
  check(count(Serial.tx, "$1;") > 10, "одометрия $1 публикуется (~50 Гц)");

  printf("2) $12;1 - телеметрия: ~100 строк/с, одометрия на паузе\n");
  Serial.tx.clear();
  cmd("$12;1;#");
  run(0.5);   // (команды повторять не нужно: 0.5 с < тайм-аута 1 с)
  const size_t nTel = count(Serial.tx, "$3;");
  printf("   строк $3 за 0.5 с: %zu\n", nTel);
  check(nTel >= 45 && nTel <= 55, "телеметрия ~100 Гц");
  check(count(Serial.tx, "$1;") <= 1, "одометрия $1 не публикуется (допустима одна строка до обработки команды)");
  cmd("$12;0;#");
  Serial.tx.clear();
  run(0.2);
  check(count(Serial.tx, "$3;") <= 1 && count(Serial.tx, "$1;") > 5, "после $12;0 снова одометрия, телеметрии нет");

  printf("3) $10 - разомкнутый режим 0.5\n");
  cmd("$10;0.5;0.5;#");
  run(0.1);
  check(controller.isOpenLoop(), "разомкнутый режим включён");
  check(controller._pwm.pins[2] >= 2040 && controller._pwm.pins[2] <= 2055, "правый: 50% = ~2047");
  check(controller._pwm.pins[1] >= 2040 && controller._pwm.pins[1] <= 2055, "левый (инверсия): 50% на втором канале");
  cmd("$10;-0.3;0.3;#");
  run(0.1);
  check(controller._pwm.pins[0] > 1000 && controller._pwm.pins[1] == 0, "левый назад: смена стороны моста");
  cmd("$10;2.0;0.0;#");
  run(0.05);
  check(controller._pwm.pins[1] == 4095, "ШИМ ограничен 100%");

  printf("4) тишина > 1 с: аварийная остановка, телеметрия выключается\n");
  cmd("$12;1;#");
  run(0.2);
  check(telemetry_on, "телеметрия включена");
  run(1.3);
  check(!controller.isOpenLoop(), "замкнутый режим возвращён");
  check(controller._pwm.pins[0] == 0 && controller._pwm.pins[1] == 0 && controller._pwm.pins[2] == 0 && controller._pwm.pins[3] == 0, "все каналы выключены");
  check(!telemetry_on, "телеметрия выключена");

  printf("5) $11 и $13 - feedforward\n");
  Serial.tx.clear();
  cmd("$11;0.7;0.3;0.8;0.31;0;#");
  run(0.05);
  cmd("$13;#");
  run(0.05);
  check(FF_GAIN_L > 0.699f && FF_GAIN_L < 0.701f && FF_DEADZONE_L > 0.299f && FF_DEADZONE_L < 0.301f, "параметры приняты");
  check(Serial.tx.find("$4;0.70000;0.30000;0.80000;0.31000;#") != std::string::npos, "ответ $4 верен");
  cmd("$11;0.7;0.99;0.8;0.31;0;#");   // dz > 0.9 - обрезается
  run(0.05);
  check(FF_DEADZONE_L <= 0.9f, "мёртвая зона ограничена 0.9");
  cmd("$11;nan;0.3;0.8;0.31;0;#");
  run(0.05);
  check(FF_GAIN_L > 0.699f && FF_GAIN_L < 0.701f, "NaN отвергнут");

  printf("5b) $14 - фильтр уставки\n");
  Serial.tx.clear();
  cmd("$14;0.2;0#");
  run(0.05);
  cmd("$13;#");
  run(0.05);
  check(Serial.tx.find("$6;0.20000;#") != std::string::npos, "ответ $6 с новой постоянной");
  cmd("$14;-5;0#");
  run(0.05);
  check(controller.getRefFilter() >= 0.0f, "отрицательная постоянная не принимается");
  cmd("$14;0.15;0#");
  run(0.05);
  cmd("$2;0.2;0.2#");
  run(0.3);
  check(controller.getRefLeft() > 0.0f && controller.getRefLeft() < 0.2f * 0.95f, "задание плавно нарастает (не скачок)");
  for (int i = 0; i < 3; ++i) { run(0.5); cmd("$2;0.2;0.2#"); }   // аварийный стоп через 1 с без команд
  check(fabsf(controller.getRefLeft() - controller.getTargetLeft()) < 0.01f * controller.getTargetLeft(), "задание сходится к уставке");
  cmd("$2;0;0#");
  run(0.05);
  check(controller.getRefLeft() == 0.0f && controller.getRefRight() == 0.0f, "нулевая уставка сбрасывает задание сразу");

  printf("6) старые команды: $1 (twist), $5 (запрос ПИД), мусор\n");
  Serial.tx.clear();
  cmd("$1;0.2;0.0;#");
  run(0.05);
  check(controller.getTargetLeft() > 0.19f && controller.getTargetRight() > 0.19f, "$1 принят");
  cmd("$5;#");
  run(0.05);
  {
    char want[160];   // ожидаем заводские значения из config.h, что бы там ни стояло
    snprintf(want, sizeof want, "$2;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;#", KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
    check(Serial.tx.find(want) != std::string::npos, "ответ $2 с текущими коэффициентами из config.h");
  }
  cmd("$99;1;2;#garbage$7;1;#");
  run(0.05);
  check(controller.getTargetLeft() > 0.19f, "мусор и неизвестные типы игнорируются");

  printf("7) дисплей: кадр уходит по кускам только в свободном окне между тактами\n");
  g_onOledChunk = onChunk;
  cmd("$2;0;0;#");   // обновить таймер связи
  g_chunks = 0; g_minWindow = 1L << 30;
  run(0.9);
  g_onOledChunk = nullptr;
  printf("   кусков за 0.9 с: %d, минимальное время до следующего такта: %ld мкс\n", g_chunks, g_minWindow);
  check(g_chunks >= 28 && g_chunks <= 40, "кадр отправляется целиком (8 кусков) ~4-5 раз за 0.9 с");
  check(g_minWindow >= 4000L /* I2C_WINDOW_MIN_US из config.h */, "ни один кусок не отправлен ближе порога к такту контура");

  printf("8) джиттер такта контура при работающем дисплее, INA219 и проверках I2C (с реалистичной длительностью операций)\n");
  Serial.tx.clear();
  g_onOledChunk = onChunk;
  cmd("$12;1;#");
  run(0.9);
  g_onOledChunk = nullptr;
  {
    std::vector<long> ts; size_t p = 0;
    while ((p = Serial.tx.find("$3;", p)) != std::string::npos) { ts.push_back(atol(Serial.tx.c_str() + p + 3)); p += 3; }
    long worst = 0; for (size_t i = 1; i < ts.size(); ++i) worst = std::max(worst, labs((ts[i] - ts[i - 1]) - 10000L));
    printf("   тактов: %zu, наибольшее отклонение периода от 10 мс: %ld мкс\n", ts.size(), worst);
    check(ts.size() >= 88, "тактов не меньше 88 за 0.9 с (нет пропусков)");
    check(worst <= 700, "отклонение периода <= 0.7 мс");
  }

  printf("\n%s (%d не прошло)\n", failures ? "ЕСТЬ ОШИБКИ" : "ВСЕ ПРОВЕРКИ ПРОШЛИ", failures);
  return failures ? 1 : 0;
}
