#include <sys/_stdint.h>
#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>


class Encoders {
public:
  static bool begin(int PIN_R_A, int PIN_R_B, int PIN_L_A, int PIN_L_B);
  static int32_t getLeftAndReset();
  static int32_t getRightAndReset();
  static void reset();

private:
  static volatile int32_t _leftPos;
  static volatile int32_t _rightPos;
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


  static void IRAM_ATTR _readR_A();
  static void IRAM_ATTR _readR_B();
  static void IRAM_ATTR _readL_A();
  static void IRAM_ATTR _readL_B();
};

#endif