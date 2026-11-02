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
#include "hardware_monitor.h"
#include "safety_manager.h"
#include "startup_service.h"

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
bool inaOk = false;
bool pwmOk = false;
bool oledOk = false;

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

HardwareMonitorContext hardwareMonitorContext = {
  INA, controller, display, inaOk, pwmOk, oledOk,
  inaErrors, pwmErrors, oledErrors,
  bus_voltage, shunt_v, current, pros,
  min_voltage, max_voltage, MAX_I2C_ERRORS, I2C_CLOCK_HZ, VERSION, i2cProbe
};
HardwareMonitor hardwareMonitor(hardwareMonitorContext);


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

// Shares the motion watchdog state with SerialService; no new timer or latch.
SafetyManagerContext safetyManagerContext = {
  controller, emergency_timer, emergency_timer_timeout,
  watchdog_stop_count, watchdog_stop_latched, telemetry_on,
  inaOk, pwmOk, bus_voltage, min_voltage
};
SafetyManager safetyManager(safetyManagerContext);

// Startup takes references to the existing application-owned hardware and timers.
StartupContext startupContext = {
  error_manager, encoders, controller, INA, display, flashStorage,
  emergency_timer, speedL, speedR, odom_tmr, control_next_us,
  inaOk, pwmOk, oledOk, VERSION, i2cProbe
};
StartupService startupService(startupContext);


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
  startupService.begin();
}

void loop()
{

  // Приём с обоих каналов
  serialService.poll();

  safetyManager.check();

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
    hardwareMonitor.samplePower();
  }
  // Мониторинг устройств I2C (не чаще 2 секунд)
  if ((millis() - lastI2CCheck >= I2C_CHECK_INTERVAL) && i2cWindowOpen())
  {
    lastI2CCheck = millis();
    hardwareMonitor.checkI2CDevices();
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
