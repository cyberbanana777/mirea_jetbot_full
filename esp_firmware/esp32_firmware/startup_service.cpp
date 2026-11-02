#include "startup_service.h"
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "command_policy.h"

void StartupService::begin()
{

  // Светодиод ошибок настраиваем и гасим первым делом
  ctx_.error_manager.begin(ERROR_LED_PIN);

  Serial.begin(115200);
  Serial1.begin(115200);

  ctx_.error_manager.check_exec(Wire.begin(), "Error! Failed to init I2C bus.");
  Wire.setTimeOut(50); // 50 мс таймаут на транзакцию
  Wire.setClock(I2C_CLOCK_HZ);

  const bool oledDetected = ctx_.probe(0x3C);
  ctx_.error_manager.soft_check_exec(oledDetected, "Warning! OLED-display is unaviable through I2C bus");
  const bool inaDetected = ctx_.probe(0x41);
  ctx_.error_manager.soft_check_exec(inaDetected, "Warning! INA219 sensor is unaviable through I2C bus");
  const bool pwmDetected = ctx_.probe(0x60);
  ctx_.error_manager.soft_check_exec(pwmDetected, "Warning! PWM-controller is unaviable through I2C bus");

  ctx_.error_manager.check_exec(ctx_.encoders.begin(PIN_R_A, PIN_R_B, PIN_L_A, PIN_L_B), "Error! Failed to init encoders");
  const bool pwmInitialized = ctx_.controller.begin();
  ctx_.pwmOk = pwmDetected && pwmInitialized;
  ctx_.error_manager.check_exec(pwmInitialized, "Error! Failed to init motor controller");

  const bool inaInitialized = ctx_.INA.begin();
  ctx_.inaOk = inaDetected && inaInitialized;
  ctx_.error_manager.check_exec(inaInitialized, "Error! Failed to init INA219 module.");

  const bool oledInitialized = ctx_.display.begin((char *)ctx_.version);
  ctx_.oledOk = oledDetected && oledInitialized;
  ctx_.error_manager.check_exec(oledInitialized, "Error! OLED init failed.");
  Wire.setClock(I2C_CLOCK_HZ); // display.begin() вызывает Wire.begin() ещё раз

  delay(100);

  ctx_.error_manager.check_exec(ctx_.flashStorage.begin(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R), "Error! NVS init failed.");
  const float defaults[6] = {KP_L, KI_L, KD_L, KP_R, KI_R, KD_R};
  ctx_.flashStorage.loadPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
  if (!CommandPolicy::validPid(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R)) {
    KP_L = defaults[0]; KI_L = defaults[1]; KD_L = defaults[2];
    KP_R = defaults[3]; KI_R = defaults[4]; KD_R = defaults[5];
  }
  ctx_.controller.setPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);

  // Feedforward: значения из config.h, если калибровка ещё не сохранялась во flash
  ctx_.flashStorage.loadFF(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);
  ctx_.controller.setFeedforward(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);

  // Фильтр уставки: значение из config.h, если не менялось командой $14 с сохранением
  ctx_.flashStorage.loadRefTau(REF_TAU_S);
  ctx_.controller.setRefFilter(REF_TAU_S);

  // Регулятор (ПИ или LADRC) и параметры LADRC: значения из config.h, если не сохранялись ($15/$16)
  ctx_.flashStorage.loadAdrc(ADRC_WC, ADRC_WO, ADRC_B0_L, ADRC_B0_R, ADRC_FF);
  ADRC_FF = constrain(roundf(ADRC_FF), 0.0f, 2.0f);
  ctx_.controller.setAdrc(ADRC_WC, ADRC_WO, ADRC_B0_L, ADRC_B0_R, (uint8_t)ADRC_FF);
  ctx_.flashStorage.loadCtrlMode(CTRL_MODE);
  ctx_.controller.setControlMode(CTRL_MODE == 1 ? ControlMode::kLadrc : ControlMode::kPi);

  ctx_.controller.emergencyStop();

  ctx_.emergency_timer = millis();
  // Отсчёт одометрии и контура скорости с конца инициализации, а не с включения питания
  ctx_.encoders.reset();
  ctx_.speedL.reset();
  ctx_.speedR.reset();
  ctx_.odom_tmr = micros();
  ctx_.control_next_us = micros() + CONTROL_PERIOD_US;
}
