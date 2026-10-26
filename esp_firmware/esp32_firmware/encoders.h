#include <sys/_stdint.h>
#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>


class Encoders {
public:
  static bool begin(int PIN_R_A, int PIN_R_B, int PIN_L_A, int PIN_L_B);
  // Счётчики для одометрии: чтение с обнулением
  static int32_t getLeftAndReset();
  static int32_t getRightAndReset();
  static void reset();

  // Снимок для оценки скорости (speed_estimator.h): накопленная позиция (фронтов,
  // никогда не обнуляется) и время последнего фронта, мкс. Обе величины читаются
  // атомарно, поэтому относятся к одному и тому же фронту.
  static void snapshotLeft(int32_t &total, uint32_t &lastEdgeUs);
  static void snapshotRight(int32_t &total, uint32_t &lastEdgeUs);

private:
  static volatile int32_t _leftPos;
  static volatile int32_t _rightPos;
  static volatile int32_t _leftTotal;
  static volatile int32_t _rightTotal;
  static volatile uint32_t _leftLastUs;
  static volatile uint32_t _rightLastUs;
  static volatile bool _ReadPIN_L_A_A;
  static volatile bool _ReadPIN_L_A_B;
  static volatile bool _ReadPIN_L_B_A;
  static volatile bool _ReadPIN_L_B_B;
  static volatile bool _ReadPIN_R_A_A;
  static volatile bool _ReadPIN_R_A_B;
  static volatile bool _ReadPIN_R_B_A;
  static volatile bool _ReadPIN_R_B_B;
  static volatile int _PIN_R_A;
  static volatile int _PIN_R_B;
  static volatile int _PIN_L_A;
  static volatile int _PIN_L_B;
  static portMUX_TYPE _mux;


  static void IRAM_ATTR _readR_A();
  static void IRAM_ATTR _readR_B();
  static void IRAM_ATTR _readL_A();
  static void IRAM_ATTR _readL_B();
  // Учёт одного фронта: шаг d = +1/-1 из конечного автомата
  static void IRAM_ATTR _countR(int8_t d);
  static void IRAM_ATTR _countL(int8_t d);
};

#endif
