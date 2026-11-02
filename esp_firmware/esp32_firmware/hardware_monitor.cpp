#include "hardware_monitor.h"
#include <Wire.h>

void HardwareMonitor::observeStartup() {
  // StartupService has already set these flags from probe && begin.
  ctx_.inaDiagnostics.state = ctx_.inaOk ? I2CDeviceState::kHealthy : I2CDeviceState::kNotDetected;
  ctx_.pwmDiagnostics.state = ctx_.pwmOk ? I2CDeviceState::kHealthy : I2CDeviceState::kNotDetected;
  ctx_.oledDiagnostics.state = ctx_.oledOk ? I2CDeviceState::kHealthy : I2CDeviceState::kNotDetected;
}

void HardwareMonitor::recordProbe(I2CDeviceDiagnostics &diag, bool responds) {
  if (!responds) {
    if (diag.probeFailures != UINT32_MAX) ++diag.probeFailures;
    diag.state = I2CDeviceState::kFault;
  } else if (diag.state == I2CDeviceState::kFault) {
    // A device responding again is not the same as a completed reinitialization.
    diag.state = I2CDeviceState::kHealthy;
  }
}

void HardwareMonitor::recordRecoveryAttempt(I2CDeviceDiagnostics &diag) {
  if (diag.recoveryAttempts != UINT32_MAX) ++diag.recoveryAttempts;
}

void HardwareMonitor::recordRecoveryResult(I2CDeviceDiagnostics &diag, bool successful) {
  if (successful) {
    if (diag.successfulRecoveries != UINT32_MAX) ++diag.successfulRecoveries;
    diag.state = I2CDeviceState::kRecovered;
  } else {
    diag.state = I2CDeviceState::kFault;
  }
}

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
  recordProbe(ctx_.inaDiagnostics, inaResponds);
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
      recordRecoveryAttempt(ctx_.inaDiagnostics);
      if (INA.begin())
      {
        inaOk = true;
        inaErrors = 0;
        reinitDone = true;
        recordRecoveryResult(ctx_.inaDiagnostics, true);
        Serial.println("INA219 reinitialized");
      }
      else
      {
        recordRecoveryResult(ctx_.inaDiagnostics, false);
        Serial.println("INA219 reinit failed");
        // Оставляем inaOk=false, счётчик сбросим после успеха
        inaErrors = ctx_.maxI2cErrors; // не копить бесконечно
      }
    }
  }

  // -------- PCA9685 (0x60) --------
  bool pwmResponds = ctx_.probe(0x60);
  recordProbe(ctx_.pwmDiagnostics, pwmResponds);
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
      recordRecoveryAttempt(ctx_.pwmDiagnostics);
      if (controller.reinitializePWM() && ctx_.probe(0x60))
      {
        pwmOk = true;
        pwmErrors = 0;
        reinitDone = true;
        // Восстанавливаем управление моторами
        controller.emergencyStop();
        // Но update() заново рассчитает ПИД и запишет моторы, что нам и нужно
        recordRecoveryResult(ctx_.pwmDiagnostics, true);
        Serial.println("PCA9685 reinitialized");
      }
      else
      {
        recordRecoveryResult(ctx_.pwmDiagnostics, false);
        Serial.println("PCA9685 reinit failed");
        pwmErrors = ctx_.maxI2cErrors;
      }
    }
  }

  // -------- OLED (0x3C) --------
  bool oledResponds = ctx_.probe(0x3C);
  recordProbe(ctx_.oledDiagnostics, oledResponds);
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
      recordRecoveryAttempt(ctx_.oledDiagnostics);
      if (display.reinitialize((char *)ctx_.version))
      {
        oledOk = true;
        oledErrors = 0;
        reinitDone = true;
        recordRecoveryResult(ctx_.oledDiagnostics, true);
        Serial.println("OLED reinitialized");
      }
      else
      {
        recordRecoveryResult(ctx_.oledDiagnostics, false);
        Serial.println("OLED reinit failed");
        oledErrors = ctx_.maxI2cErrors;
      }
    }
  }

  if (reinitDone)
    Wire.setClock(ctx_.i2cClockHz);
}
