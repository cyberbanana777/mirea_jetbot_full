// Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

/**
  @author Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
  @date 28.10.2026
  @version 2.4.0
*/
const char *VERSION = "2.4.0";

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <stdio.h>
#include <INA219.h>

#include "config.h"
#include "encoders.h"
#include "errors.h"
#include "protocol_wrapper.h"
#include "motor_controller.h"
#include "flash_storage.h"
#include "command_parser.h"
#include "display_manager.h"
#include "serial_receiver.h"
#include "odometry.h"
#include "speed_estimator.h"
#include "command_policy.h"
#include "command_dispatcher.h"
#include "safety_policy.h"
#include "control_scheduler.h"
#include "serial_service.h"
#include "control_loop_service.h"

Encoders encoders;
ErrorManager error_manager;
MotorController controller(WHEEL_BASE, MAX_CONSTRUCTIVE_VELOCITY, DT,
                           KP_L, KI_L, KD_L,
                           KP_R, KI_R, KD_R);

INA219 INA(0x41);
// Пространство имён NVS изменено с "Memory1" на "Memory2": коэффициенты ПИД, сохранённые
// прежней прошивкой (под контур 10 Гц), в новом контуре не годятся. Чтобы вернуть старые
// значения, верните "Memory1" (они лежат во flash нетронутыми).
FlashStorageManager flashStorage("Memory2");
DisplayManager display(update_voltage_timer_timeout); // 200 мс интервал обновления
SerialReceiver mainReceiver(Serial);                  // основной канал (USB)
SerialReceiver jetsonReceiver(Serial1);               // Jetson Nano

/**
 * @brief Проверяет наличие устройства на заданном I²C-адресе.
 * @param address - 7-битный адрес устройства (без сдвига, как передаётся в Wire)
 * @return true, если устройство ответило ACK, иначе false
 */
bool i2cProbe(uint8_t address)
{
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  return (error == 0); // 0 означает успех (получен ACK)
}

// Для таймеров
unsigned long odom_tmr = 0;   // момент последнего расчёта одометрии, мкс
unsigned long control_next_us = 0; // момент следующего такта контура скорости, мкс
unsigned long emergency_timer = 0; // last accepted motion command only
uint32_t accepted_commands = 0;
uint32_t rejected_commands = 0;
uint32_t watchdog_stop_count = 0;
bool watchdog_stop_latched = false;
uint64_t update_timer;

// Одометрия (50 Гц): считается по тикам энкодеров, как и раньше
DiffDriveOdometry odometry(WHEEL_BASE);

// Оценка скорости колёс по фронтам (M/T-метод), по одному оценщику на колесо
const MTSpeedParams SPEED_PARAMS = {1.0f / ENC_EDGES_PER_REV, SPEED_TIMEOUT_US,
                                    SPEED_MIN_EDGES, SPEED_MAX_SANE_REV_S, SPEED_FILTER_K,
                                    SPEED_ALIGN_QUAD};
MTSpeedEstimator speedR(SPEED_PARAMS);
MTSpeedEstimator speedL(SPEED_PARAMS);

// Телеметрия контура скорости ($12): включается командой, гаснет без связи
bool telemetry_on = false;
uint8_t telemetry_div = 0;




// Скорости для отправки на Jetson: обновляются на каждом такте контура (100 Гц)
float linear_vel_ = 0.0;
float angular_vel_ = 0.0;
float left_vel_ = 0.0;
float right_vel_ = 0.0;

ControlLoopContext controlLoopContext = {
  encoders, speedR, speedL, controller, odometry,
  telemetry_on, telemetry_div, linear_vel_, angular_vel_,
  left_vel_, right_vel_
};
ControlLoopService controlLoop(controlLoopContext);

// Сеть Jetson Nano из сообщения $6 (только для показа на OLED-дисплее;
// сама ESP32 к Wi-Fi не подключается)
String wifi_ssid = "";
String wifi_password = "";
String wifi_ip = "";

float bus_voltage = 0.0f, shunt_v = 0.0f, current = 0.0f;
int pros;

// состояние устройств
bool inaOk = true;
bool pwmOk = true;
bool oledOk = true;

// Construct after all referenced hardware status flags have been declared.
CommandContext commandContext = {
  {controller, telemetry_on, telemetry_div},
  {controller, flashStorage, speedL, speedR,
   KP_L, KI_L, KD_L, KP_R, KI_R, KD_R,
   FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R,
   REF_TAU_S, ADRC_WC, ADRC_WO, ADRC_B0_L, ADRC_B0_R, ADRC_FF,
   CTRL_MODE, accepted_commands, rejected_commands},
  {controller, accepted_commands, rejected_commands,
   watchdog_stop_count, inaOk, pwmOk}
};

// счётчики ошибок подряд
uint8_t inaErrors = 0;
uint8_t pwmErrors = 0;
uint8_t oledErrors = 0;
const uint8_t MAX_I2C_ERRORS = 0; // после скольких ошибок инициализировать заново

unsigned long lastI2CCheck = 0;
const unsigned long I2C_CHECK_INTERVAL = 500; // 0.5 секунды

void checkI2CDevices()
{
  bool reinitDone = false; // после переинициализации Wire.begin() мог сбросить частоту шины

  // -------- INA219 (0x41) --------
  bool inaResponds = i2cProbe(0x41);
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
    if (inaErrors >= MAX_I2C_ERRORS)
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
        inaErrors = MAX_I2C_ERRORS; // не копить бесконечно
      }
    }
  }

  // -------- PCA9685 (0x60) --------
  bool pwmResponds = i2cProbe(0x60);
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
    if (pwmErrors >= MAX_I2C_ERRORS)
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
        pwmErrors = MAX_I2C_ERRORS;
      }
    }
  }

  // -------- OLED (0x3C) --------
  bool oledResponds = i2cProbe(0x3C);
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
    if (oledErrors >= MAX_I2C_ERRORS)
    {
      oledOk = false;
      if (display.reinitialize((char *)VERSION))
      {
        oledOk = true;
        oledErrors = 0;
        reinitDone = true;
        Serial.println("OLED reinitialized");
      }
      else
      {
        Serial.println("OLED reinit failed");
        oledErrors = MAX_I2C_ERRORS;
      }
    }
  }

  if (reinitDone)
    Wire.setClock(I2C_CLOCK_HZ);
}

void parseJetsonMessage(const char *fullMessage)
{
  if (!fullMessage || fullMessage[0] != '$')
    return;
  // Копируем во временный буфер, чтобы не портить оригинал (если нужно)
  char buf[128];
  strncpy(buf, fullMessage, 127);
  buf[127] = '\0';

  // Удаляем '$' и '#'
  char *start = buf + 1;
  char *end = strchr(start, '#');
  if (end)
    *end = '\0';

  char *parts[4];
  int count = 0;
  char *token = strtok(start, ";");
  while (token && count < 4)
  {
    parts[count++] = token;
    token = strtok(nullptr, ";");
  }
  if (count >= 4 && strcmp(parts[0], "6") == 0)
  {
    wifi_ssid = parts[1];
    wifi_password = parts[2];
    wifi_ip = parts[3];
  }
}


// Service owns no state; all referenced objects are constructed above.
SerialServiceContext serialServiceContext = {
  mainReceiver, jetsonReceiver, commandContext,
  emergency_timer, watchdog_stop_latched, accepted_commands, rejected_commands,
  inaOk, pwmOk, bus_voltage, min_voltage, parseJetsonMessage
};
SerialService serialService(serialServiceContext);

/**
* @brief Преобразет тики в частоту вращения (об/с)
* @param ticks - счётчик тактов из прерываний
* @param dir - направеление. Может быть 'r', 'l'. При 'l'  происходит инвертирование значения,
необходиомое для нормальной работы логики. Чтобы при движении вперёд или назад
знаки угловых скоростей колёс были одинаковые
* @return result, обороты в секунду типа float32_t.
*/
// The 100 Hz control step and 50 Hz odometry step live in ControlLoopService.
bool i2cWindowOpen()
{
  return ControlScheduler::i2cWindowOpen(control_next_us, I2C_WINDOW_MIN_US);
}

void setup()
{

  // Светодиод ошибок настраиваем и гасим первым делом
  error_manager.begin(ERROR_LED_PIN);

  Serial.begin(115200);
  Serial1.begin(115200);

  error_manager.check_exec(Wire.begin(), "Error! Failed to init I2C bus.");
  Wire.setTimeOut(50); // 50 мс таймаут на транзакцию
  Wire.setClock(I2C_CLOCK_HZ);

  error_manager.soft_check_exec(i2cProbe(0x3C), "Warning! OLED-display is unaviable through I2C bus");
  error_manager.soft_check_exec(i2cProbe(0x41), "Warning! INA219 sensor is unaviable through I2C bus");
  error_manager.soft_check_exec(i2cProbe(0x60), "Warning! PWM-controller is unaviable through I2C bus");

  error_manager.check_exec(encoders.begin(PIN_R_A, PIN_R_B, PIN_L_A, PIN_L_B), "Error! Failed to init encoders");
  error_manager.check_exec(controller.begin(), "Error! Failed to init motor controller");

  error_manager.check_exec(INA.begin(), "Error! Failed to init INA219 module.");

  error_manager.check_exec(display.begin((char *)VERSION), "Error! OLED init failed.");
  Wire.setClock(I2C_CLOCK_HZ); // display.begin() вызывает Wire.begin() ещё раз

  delay(100);

  error_manager.check_exec(flashStorage.begin(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R), "Error! NVS init failed.");
  const float defaults[6] = {KP_L, KI_L, KD_L, KP_R, KI_R, KD_R};
  flashStorage.loadPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
  if (!CommandPolicy::validPid(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R)) {
    KP_L = defaults[0]; KI_L = defaults[1]; KD_L = defaults[2];
    KP_R = defaults[3]; KI_R = defaults[4]; KD_R = defaults[5];
  }
  controller.setPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);

  // Feedforward: значения из config.h, если калибровка ещё не сохранялась во flash
  flashStorage.loadFF(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);
  controller.setFeedforward(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);

  // Фильтр уставки: значение из config.h, если не менялось командой $14 с сохранением
  flashStorage.loadRefTau(REF_TAU_S);
  controller.setRefFilter(REF_TAU_S);

  // Регулятор (ПИ или LADRC) и параметры LADRC: значения из config.h, если не сохранялись ($15/$16)
  flashStorage.loadAdrc(ADRC_WC, ADRC_WO, ADRC_B0_L, ADRC_B0_R, ADRC_FF);
  ADRC_FF = constrain(roundf(ADRC_FF), 0.0f, 2.0f);
  controller.setAdrc(ADRC_WC, ADRC_WO, ADRC_B0_L, ADRC_B0_R, (uint8_t)ADRC_FF);
  flashStorage.loadCtrlMode(CTRL_MODE);
  controller.setControlMode(CTRL_MODE == 1 ? ControlMode::kLadrc : ControlMode::kPi);

  controller.emergencyStop();

  emergency_timer = millis();
  // Отсчёт одометрии и контура скорости с конца инициализации, а не с включения питания
  encoders.reset();
  speedL.reset();
  speedR.reset();
  odom_tmr = micros();
  control_next_us = micros() + CONTROL_PERIOD_US;
}

void loop()
{

  // Приём с обоих каналов
  serialService.poll();

  const bool host_silent = (millis() - emergency_timer > emergency_timer_timeout);
  if (SafetyPolicy::mustStop(host_silent, inaOk, pwmOk, bus_voltage, min_voltage) &&
      !controller.isStopped()) {
    controller.emergencyStop();
  }
  if (host_silent && !watchdog_stop_latched) {
    ++watchdog_stop_count;
    watchdog_stop_latched = true;
  }
  if (host_silent)
  {
    telemetry_on = false; // нет связи: канал нужен одометрии
  }

  unsigned long now = micros();

  // Контур скорости: каждые CONTROL_PERIOD_US (10 мс, 100 Гц)
  if (ControlScheduler::controlDue(now, control_next_us, CONTROL_PERIOD_US))
  {
    controlLoop.controlStep();
    now = micros();
  }

  // Одометрия: каждые odom_timer_timeout (20 мс, 50 Гц)
  if (ControlScheduler::odometryDue(now, odom_tmr, odom_timer_timeout))
  {
    controlLoop.odometryStep();
  }

  // Обработчики UART сохраняют прежний порядок и ограничение 4 кадра за loop().
  serialService.processCommands();
  serialService.processSecondary();

  // Медленные операции I2C - только в свободном окне между тактами контура
  if ((millis() - update_timer >= update_voltage_timer_timeout) && inaOk && i2cWindowOpen())
  {
    update_timer = millis();
    // Чтение напряжения и тока
    bus_voltage = INA.getBusVoltage();
    shunt_v = INA.getShuntVoltage_mV();
    pros = (bus_voltage - min_voltage) / (max_voltage - min_voltage) * 100;
    current = shunt_v / 0.1 / 1000;
  }
  // Мониторинг устройств I2C (не чаще 2 секунд)
  if ((millis() - lastI2CCheck >= I2C_CHECK_INTERVAL) && i2cWindowOpen())
  {
    lastI2CCheck = millis();
    checkI2CDevices();
  }

  // Обновление дисплея (само проверяет интервал; кадр уходит по кускам, когда шина свободна)
  if (oledOk)
  {
    display.update(wifi_ssid, wifi_ip, wifi_password, bus_voltage, current, pros, i2cWindowOpen());
  }

  // Индикация ошибок во время работы: светодиод часто мигает, пока
  // устройство на I2C не отвечает или напряжение батареи ниже минимума
  // (в этом случае моторы и так остановлены аварийно).
  bool lowBattery = inaOk && bus_voltage < min_voltage;
  error_manager.setFault(!inaOk || !pwmOk || !oledOk || lowBattery);
  error_manager.update();
}
