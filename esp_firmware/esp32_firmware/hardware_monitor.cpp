#include "hardware_monitor.h"
#include <Wire.h>

void HardwareMonitor::samplePower() {
  // Keep the exact order/formulas of the previous 200 ms INA219 sample.
  ctx_.busVoltage = ctx_.ina.getBusVoltage();
  ctx_.shuntVoltage = ctx_.ina.getShuntVoltage_mV();
  ctx_.batteryPercent = (ctx_.busVoltage - ctx_.minVoltage) / (ctx_.maxVoltage - ctx_.minVoltage) * 100;
  ctx_.current = ctx_.shuntVoltage / 0.1 / 1000;
}

void HardwareMonitor::checkI2CDevices() {
  auto &inaOk = ctx_.inaOk;
  auto &pwmOk = ctx_.pwmOk;
  auto &oledOk = ctx_.oledOk;
  auto &inaErrors = ctx_.inaErrors;
  auto &pwmErrors = ctx_.pwmErrors;
  auto &oledErrors = ctx_.oledErrors;
  auto &INA = ctx_.ina;
  auto &controller = ctx_.controller;
  auto &display = ctx_.display;

  bool reinitDone = false; // после переинициализации Wire.begin() мог сбросить частоту шины

  // -------- INA219 (0x41) --------
  bool inaResponds = ctx_.probe(0x41);
  if (inaResponds)
  {
    if (!inaOk)
    {
      // Устройство вернулось без переинициализации (редко, но бывает)
      inaOk = true;
      inaErrors = 0;
    }
  }
  else
  {
    inaErrors++;
    if (inaErrors >= ctx_.maxI2cErrors)
    {
      inaOk = false;
      // Попытка переинициализации
      if (INA.begin())
      {
        inaOk = true;
        inaErrors = 0;
        reinitDone = true;
        Serial.println("INA219 reinitialized");
      }
      else
      {
        Serial.println("INA219 reinit failed");
        // Оставляем inaOk=false, счётчик сбросим после успеха
        inaErrors = ctx_.maxI2cErrors; // не копить бесконечно
      }
    }
  }

  // -------- PCA9685 (0x60) --------
  bool pwmResponds = ctx_.probe(0x60);
  if (pwmResponds)
  {
    if (!pwmOk)
    {
      pwmOk = true;
      pwmErrors = 0;
    }
  }
  else
  {
    pwmErrors++;
    if (pwmErrors >= ctx_.maxI2cErrors)
    {
      pwmOk = false;
      if (controller.reinitializePWM())
      {
        pwmOk = true;
        pwmErrors = 0;
        reinitDone = true;
        // Восстанавливаем управление моторами
        controller.emergencyStop();
        // Но update() заново рассчитает ПИД и запишет моторы, что нам и нужно
        Serial.println("PCA9685 reinitialized");
      }
      else
      {
        Serial.println("PCA9685 reinit failed");
        pwmErrors = ctx_.maxI2cErrors;
      }
    }
  }

  // -------- OLED (0x3C) --------
  bool oledResponds = ctx_.probe(0x3C);
  if (oledResponds)
  {
    if (!oledOk)
    {
      oledOk = true;
      oledErrors = 0;
    }
  }
  else
  {
    oledErrors++;
    if (oledErrors >= ctx_.maxI2cErrors)
    {
      oledOk = false;
      if (display.reinitialize((char *)ctx_.version))
      {
        oledOk = true;
        oledErrors = 0;
        reinitDone = true;
        Serial.println("OLED reinitialized");
      }
      else
      {
        Serial.println("OLED reinit failed");
        oledErrors = ctx_.maxI2cErrors;
      }
    }
  }

  if (reinitDone)
    Wire.setClock(ctx_.i2cClockHz);
}
