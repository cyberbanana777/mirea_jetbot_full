// Проверка, что рефакторинг прерываний энкодера (encoders.cpp) не изменил декодирование:
// на случайных последовательностях изменений A/B новая и прежняя (orig/encoders.cpp) реализации
// должны давать одинаковые счётчики. Заодно проверяем счётчик (total, время фронта).
#include "Arduino.h"
#include <stdio.h>
#include <stdlib.h>
#define private public
#include "encoders.h"          // новая
#undef ENCODERS_H
#define Encoders OldEncoders
#include "orig/encoders.h"     // прежняя, класс переименован в OldEncoders
#undef Encoders
#undef private

int main() {
  Encoders::begin(34, 35, 32, 33);
  OldEncoders::begin(34, 35, 32, 33);
  srand(12345);
  long steps = 0, mism = 0;
  for (int i = 0; i < 200000; ++i) {
    g_micros += 1 + rand() % 50;
    const int pin = 32 + rand() % 4;          // какой вывод изменился: 32,33 - левый, 34,35 - правый
    g_pins[pin] ^= 1;
    switch (pin) {
      case 34: Encoders::_readR_A(); OldEncoders::_readR_A(); break;
      case 35: Encoders::_readR_B(); OldEncoders::_readR_B(); break;
      case 32: Encoders::_readL_A(); OldEncoders::_readL_A(); break;
      case 33: Encoders::_readL_B(); OldEncoders::_readL_B(); break;
    }
    ++steps;
    if (rand() % 97 == 0) {   // чтение с обнулением должно совпадать
      if (Encoders::getRightAndReset() != OldEncoders::getRightAndReset()) ++mism;
      if (Encoders::getLeftAndReset() != OldEncoders::getLeftAndReset()) ++mism;
    }
    if (Encoders::_rightPos != OldEncoders::_rightPos || Encoders::_leftPos != OldEncoders::_leftPos) ++mism;
  }
  // total не обнуляется и равен сумме шагов; время - момент последнего фронта
  int32_t totR, totL; uint32_t tR, tL;
  Encoders::snapshotRight(totR, tR);
  Encoders::snapshotLeft(totL, tL);
  printf("шагов: %ld, расхождений с прежней реализацией: %ld\n", steps, mism);
  printf("snapshot: right total=%d t=%u, left total=%d t=%u (g_micros=%lu)\n", totR, tR, totL, tL, g_micros);

  // Цикл Грея в одну сторону даёт +4 на оборот метки, в другую -4, время фронта обновляется
  g_pins[34] = 1; g_pins[35] = 1;
  int32_t t0; uint32_t tt0; Encoders::snapshotRight(t0, tt0);
  const int seqA[4] = {34, 35, 34, 35};      // 11 -> 01 -> 00 -> 10 -> 11 (прямое направление в этой реализации)
  for (int k = 0; k < 4; ++k) { g_micros += 100; g_pins[seqA[k]] ^= 1; (seqA[k] == 34 ? Encoders::_readR_A : Encoders::_readR_B)(); }
  int32_t t1; uint32_t tt1; Encoders::snapshotRight(t1, tt1);
  printf("цикл 11->01->00->10->11: total изменился на %d, время последнего фронта %u (g_micros=%lu)\n", t1 - t0, tt1, g_micros);
  const bool ok = (mism == 0) && (t1 - t0 == 4) && (tt1 == (uint32_t)g_micros);
  printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
