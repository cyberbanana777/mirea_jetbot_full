// Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

/**
  @author Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
  @date 25.10.2026
  @version 2.3.0
*/
const char *VERSION = "2.3.0";

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
unsigned long emergency_timer = 0;
uint64_t update_timer;

// Одометрия (50 Гц): считается по тикам энкодеров, как и раньше
DiffDriveOdometry odometry(WHEEL_BASE);

// Оценка скорости колёс по фронтам (M/T-метод), по одному оценщику на колесо
const MTSpeedParams SPEED_PARAMS = {1.0f / ENC_EDGES_PER_REV, SPEED_TIMEOUT_US,
                                    SPEED_MIN_EDGES, SPEED_MAX_SANE_REV_S, SPEED_FILTER_K};
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

// Сеть Jetson Nano из сообщения $6 (только для показа на OLED-дисплее;
// сама ESP32 к Wi-Fi не подключается)
String wifi_ssid = "";
String wifi_password = "";
String wifi_ip = "";

float bus_voltage, shunt_v, current;
int pros;

// состояние устройств
bool inaOk = true;
bool pwmOk = true;
bool oledOk = true;

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
        controller.setTargetVelocity(0.0, 0.0);
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
  if (count >= 4)
  {
    wifi_ssid = parts[1];
    wifi_password = parts[2];
    wifi_ip = parts[3];
  }
}

/**
* @brief Преобразет тики в частоту вращения (об/с)
* @param ticks - счётчик тактов из прерываний
* @param dir - направеление. Может быть 'r', 'l'. При 'l'  происходит инвертирование значения,
необходиомое для нормальной работы логики. Чтобы при движении вперёд или назад
знаки угловых скоростей колёс были одинаковые
* @return result, обороты в секунду типа float32_t.
*/
float convert_ticks_to_freq(long ticks, long timeout_micros, char dir)
{
  // 4 импульса на 1 шаг, 270 шагов на оборот (ENC_EDGES_PER_REV из config.h)
  const float turn_per_tick = 1.0 / ENC_EDGES_PER_REV;
  const double micros_to_sec = 1.0 / 1000000.0;
  float timeout_sec = timeout_micros * micros_to_sec;
  float result = turn_per_tick * ticks / timeout_sec;

  // для левого колеса инвертируем частоту
  if (dir == 'l')
  {
    result = result * (-1);
  }
  else if (dir == 'r')
  {
    result = result * 1;
  }

  return result;
}

/**
 * @brief Путь колеса по числу тиков энкодера, м (вперёд > 0).
 * Использует convert_ticks_to_freq с периодом 1 с: частота за 1 с = число
 * оборотов, поэтому константы энкодера и знак колеса задаются в одном месте.
 */
float convert_ticks_to_distance(long ticks, char dir)
{
  const long one_second_micros = 1000000;
  float turns = convert_ticks_to_freq(ticks, one_second_micros, dir);
  return turns * 2 * Pi * WHEEL_RADIUS;
}

/**
 * @brief Можно ли сейчас занимать шину I2C медленной операцией (дисплей, INA219,
 * проверка устройств). Да, если до следующего такта контура осталось достаточно
 * времени: PCA9685 с моторами сидит на той же шине, и такт нельзя задерживать.
 */
bool i2cWindowOpen()
{
  return (long)(control_next_us - micros()) > (long)I2C_WINDOW_MIN_US;
}

/**
 * @brief Один такт контура скорости (100 Гц): оценка скорости колёс по фронтам
 * энкодеров и обновление регуляторов (feedforward + ПИ).
 */
void controlStep()
{
  // Снимок (позиция, время последнего фронта) для каждого колеса, затем время "сейчас":
  // возраст последнего фронта получается неотрицательным
  int32_t posR, posL;
  uint32_t lastR, lastL;
  encoders.snapshotRight(posR, lastR);
  encoders.snapshotLeft(posL, lastL);
  const uint32_t now = micros();

  // об/с -> м/с; знак колеса приводится так, чтобы вперёд было > 0 у обоих
  const float revR = ENC_SIGN_R * speedR.update(posR, lastR, now);
  const float revL = ENC_SIGN_L * speedL.update(posL, lastL, now);
  const float rightLinVel = revR * 2 * Pi * WHEEL_RADIUS;
  const float leftLinVel = revL * 2 * Pi * WHEEL_RADIUS;

  // Регуляторы выдают сигнал на моторы
  controller.update(leftLinVel, rightLinVel);

  // Скорости для отправки на Jetson
  right_vel_ = controller.getFilteredRight();
  left_vel_ = controller.getFilteredLeft();
  linear_vel_ = (right_vel_ + left_vel_) / 2.0;
  angular_vel_ = (right_vel_ - left_vel_) / WHEEL_BASE; // для дифференциального привода

  if (telemetry_on && ++telemetry_div >= TELEMETRY_DECIMATION)
  {
    telemetry_div = 0;
    telemetryPublish(now,
                     controller.getRefLeft(), left_vel_, controller.getOutputLeft(),
                     controller.getRefRight(), right_vel_, controller.getOutputRight());
  }
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
  flashStorage.loadPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
  controller.setPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);

  // Feedforward: значения из config.h, если калибровка ещё не сохранялась во flash
  flashStorage.loadFF(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);
  controller.setFeedforward(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);

  // Фильтр уставки: значение из config.h, если не менялось командой $14 с сохранением
  flashStorage.loadRefTau(REF_TAU_S);
  controller.setRefFilter(REF_TAU_S);

  controller.setWheelSpeeds(0.0, 0.0);

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
  mainReceiver.update();
  jetsonReceiver.update();

  const bool host_silent = (millis() - emergency_timer > emergency_timer_timeout);
  if (host_silent || (bus_voltage < min_voltage))
  {
    // Аварийная остановка (возвращает и замкнутый режим, если шёл эксперимент $10)
    controller.setWheelSpeeds(0.0, 0.0);
  }
  if (host_silent)
  {
    telemetry_on = false; // нет связи: канал нужен одометрии
  }

  unsigned long now = micros();

  // Контур скорости: каждые CONTROL_PERIOD_US (10 мс, 100 Гц)
  if ((long)(now - control_next_us) >= 0)
  {
    control_next_us += CONTROL_PERIOD_US;
    if ((long)(now - control_next_us) >= 0)
    {
      // Опоздали больше чем на период (долгая операция): пропущенные такты не догоняем
      control_next_us = now + CONTROL_PERIOD_US;
    }
    controlStep();
    now = micros();
  }

  unsigned long odom_delta = now - odom_tmr;

  // Одометрия: каждые odom_timer_timeout (20 мс, 50 Гц)
  if (odom_delta >= odom_timer_timeout)
  {
    odom_tmr = now;

    // 1. Тики энкодеров за период одометрии
    int32_t rightTicks = encoders.getRightAndReset();
    int32_t leftTicks = encoders.getLeftAndReset();

    // 2. Поза по пройденному колёсами пути (м)
    odometry.update(convert_ticks_to_distance(leftTicks, 'l'),
                    convert_ticks_to_distance(rightTicks, 'r'));

    // 3. Публикация одометрии (50 Гц). Скорости колёс берутся из контура скорости
    //    (right_vel_ и др. обновляются на каждом его такте). Пока включена
    //    телеметрия, канал отдан ей, и одометрия не публикуется.
    if (!telemetry_on)
    {
      odomPublish(odometry.x(), odometry.y(), odometry.heading(),
                  linear_vel_, angular_vel_, left_vel_, right_vel_);
    }
  }

  // Обработка входного значения
  if (mainReceiver.available())
  {
    emergency_timer = millis();
    const char *raw_0 = mainReceiver.getMessage(); // теперь это C-строка
    ParsedCommand cmd = CommandParser::parse(raw_0);

    switch (cmd.type)
    {
    case Command::SET_TWIST:
      if (cmd.count >= 2)
        controller.setTargetVelocity(cmd.args[0], cmd.args[1]);
      break;

    case Command::SET_WHEEL_SPEEDS:
      if (cmd.count >= 2)
        controller.setWheelSpeeds(cmd.args[0], cmd.args[1]);
      break;

    case Command::SET_PID_TEMP:
      if (cmd.count >= 6)
      {
        KP_L = cmd.args[0];
        KI_L = cmd.args[1];
        KD_L = cmd.args[2];
        KP_R = cmd.args[3];
        KI_R = cmd.args[4];
        KD_R = cmd.args[5];
        controller.setPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
      }
      break;

    case Command::SET_PID_FLASH:
      if (cmd.count >= 6)
      {
        KP_L = cmd.args[0];
        KI_L = cmd.args[1];
        KD_L = cmd.args[2];
        KP_R = cmd.args[3];
        KI_R = cmd.args[4];
        KD_R = cmd.args[5];
        controller.setPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
        flashStorage.savePID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
      }
      break;

    case Command::REQUEST_PID:
      regulatorsCoefficientsPublish(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
      break;

    case Command::SET_DUTY:
      // Разомкнутый режим для калибровки: шлите не реже раза в секунду,
      // иначе сработает аварийная остановка
      if (cmd.count >= 2 && isfinite(cmd.args[0]) && isfinite(cmd.args[1]))
        controller.setOpenLoop(cmd.args[0], cmd.args[1]);
      break;

    case Command::SET_FF:
      if (cmd.count >= 4 && isfinite(cmd.args[0]) && isfinite(cmd.args[1]) &&
          isfinite(cmd.args[2]) && isfinite(cmd.args[3]))
      {
        FF_GAIN_L = constrain(cmd.args[0], 0.0f, 20.0f);
        FF_DEADZONE_L = constrain(cmd.args[1], 0.0f, 0.9f);
        FF_GAIN_R = constrain(cmd.args[2], 0.0f, 20.0f);
        FF_DEADZONE_R = constrain(cmd.args[3], 0.0f, 0.9f);
        controller.setFeedforward(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);
        if (cmd.count >= 5 && cmd.args[4] >= 0.5f)
          flashStorage.saveFF(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);
      }
      break;

    case Command::SET_TELEMETRY:
      if (cmd.count >= 1)
      {
        telemetry_on = (cmd.args[0] >= 0.5f);
        telemetry_div = 0;
      }
      break;

    case Command::REQUEST_FF:
      feedforwardPublish(FF_GAIN_L, FF_DEADZONE_L, FF_GAIN_R, FF_DEADZONE_R);
      refFilterPublish(REF_TAU_S);
      break;

    case Command::SET_REF_FILTER:
      if (cmd.count >= 1 && isfinite(cmd.args[0]))
      {
        REF_TAU_S = constrain(cmd.args[0], 0.0f, 2.0f);
        controller.setRefFilter(REF_TAU_S);
        if (cmd.count >= 2 && cmd.args[1] >= 0.5f)
          flashStorage.saveRefTau(REF_TAU_S);
      }
      break;

    case Command::UNKNOWN:
      break;
    }
  }

  // Сообщение от Jetson с информацией о его сети (для OLED-дисплея)
  if (jetsonReceiver.available())
  {
    const char *raw_1 = jetsonReceiver.getMessage();
    parseJetsonMessage(raw_1);
  }

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
