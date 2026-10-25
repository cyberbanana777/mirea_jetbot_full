// Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

/**
  @author Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
  @date 25.10.2026
  @version 2.1.0
*/
const char *VERSION = "2.1.0";

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <stdio.h>
#include <INA219.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

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

static const char *WIFI_SSID_FALLBACK = "YOUR_WIFI_SSID";
static const char *WIFI_PASS_FALLBACK = "YOUR_WIFI_PASSWORD";

String applied_wifi_ssid = "";
String applied_wifi_password = "";

bool wifi_cfg_dirty = true;
unsigned long last_wifi_attempt = 0;
const unsigned long WIFI_RETRY_MS = 10000;

bool ota_started = false;
uint32_t last_ota_time = 0;
// как устройство будет видно в Arduino IDE
String ota_hostname = "jetbot-esp32";

Encoders encoders;
ErrorManager error_manager;
MotorController controller(WHEEL_BASE, MAX_CONSTRUCTIVE_VELOCITY, DT,
                           KP_L, KI_L, KD_L,
                           KP_R, KI_R, KD_R);

INA219 INA(0x41);
FlashStorageManager flashStorage("Memory1");
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
unsigned long emergency_timer = 0;
uint64_t update_timer;

// Одометрия (50 Гц) и накопители тиков для ПИД (10 Гц)
DiffDriveOdometry odometry(WHEEL_BASE);
uint8_t odom_counter = 0;        // сколько периодов одометрии прошло с последнего ПИД
int32_t pid_left_ticks = 0;      // тики за текущий период ПИД
int32_t pid_right_ticks = 0;
unsigned long pid_delta = 0;     // длительность текущего периода ПИД, мкс

// Скорости для отправки на Jetson: обновляются раз в период ПИД
float linear_vel_ = 0.0;
float angular_vel_ = 0.0;
float left_vel_ = 0.0;
float right_vel_ = 0.0;

// Переменные для хранения принятых данных
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
        Serial.println("OLED reinitialized");
      }
      else
      {
        Serial.println("OLED reinit failed");
        oledErrors = MAX_I2C_ERRORS;
      }
    }
  }
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
    wifi_cfg_dirty = true;
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
  const float steps_per_tick = 1.0 / 4.0;  // 4 импульса на 1 шаг
  const float turn_per_step = 1.0 / 270.0; // 270 шагов в 1 полном обороте
  const double micros_to_sec = 1.0 / 1000000.0;
  float timeout_sec = timeout_micros * micros_to_sec;
  float result = turn_per_step * steps_per_tick * ticks / timeout_sec;

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

void startArduinoOTA()
{
  ArduinoOTA.setHostname(ota_hostname.c_str());

  // Если нужен пароль на OTA, раскомментируй:
  // ArduinoOTA.setPassword(ota_password.c_str());

  ArduinoOTA
      .onStart([]()
               {
                 String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
                 Serial.println("OTA start: " + type); // безопаснее на время обновления
               })
      .onEnd([]()
             { Serial.println("\nOTA end"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  {
      if (millis() - last_ota_time > 500) {
        Serial.printf("OTA progress: %u%%\n", (progress * 100U) / total);
        last_ota_time = millis();
      } })
      .onError([](ota_error_t error)
               {
      Serial.printf("OTA error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
  ota_started = true;

  Serial.print("ArduinoOTA ready, hostname: ");
  Serial.println(ota_hostname);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void serviceWiFi()
{
  String desired_ssid = wifi_ssid.length() ? wifi_ssid : String(WIFI_SSID_FALLBACK);
  String desired_pass = wifi_password.length() ? wifi_password : String(WIFI_PASS_FALLBACK);

  if (desired_ssid.length() == 0)
  {
    return;
  }

  // Уже подключены к нужной сети
  if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == desired_ssid)
  {
    if (!ota_started)
    {
      startArduinoOTA();
    }
    return;
  }

  // Нужна новая попытка подключения после обновления конфигурации
  if (wifi_cfg_dirty && millis() - last_wifi_attempt > 3000)
  {
    ota_started = false;

    WiFi.disconnect(true, true);
    delay(200);

    WiFi.mode(WIFI_STA);
    WiFi.begin(desired_ssid.c_str(), desired_pass.c_str());

    applied_wifi_ssid = desired_ssid;
    applied_wifi_password = desired_pass;
    last_wifi_attempt = millis();
    wifi_cfg_dirty = false;

    Serial.print("WiFi reconnect requested, SSID = ");
    Serial.println(desired_ssid);
    return;
  }

  // Периодическая повторная попытка, если не подключились
  if (WiFi.status() != WL_CONNECTED &&
      applied_wifi_ssid.length() > 0 &&
      millis() - last_wifi_attempt > WIFI_RETRY_MS)
  {
    ota_started = false;

    WiFi.disconnect(true, true);
    delay(200);

    WiFi.mode(WIFI_STA);
    WiFi.begin(applied_wifi_ssid.c_str(), applied_wifi_password.c_str());
    last_wifi_attempt = millis();

    Serial.print("WiFi retry, SSID = ");
    Serial.println(applied_wifi_ssid);
    return;
  }

  // Если подключились после begin()
  if (WiFi.status() == WL_CONNECTED)
  {

    if (!ota_started)
    {
      startArduinoOTA();
    }
  }
}

void setup()
{

  Serial.begin(115200);
  Serial1.begin(115200);

  error_manager.begin(ERROR_LED_PIN);

  error_manager.check_exec(Wire.begin(), "Error! Failed to init I2C bus.");
  Wire.setTimeOut(50); // 50 мс таймаут на транзакцию

  error_manager.soft_check_exec(i2cProbe(0x3C), "Warning! OLED-display is unaviable through I2C bus");
  error_manager.soft_check_exec(i2cProbe(0x41), "Warning! INA219 sensor is unaviable through I2C bus");
  error_manager.soft_check_exec(i2cProbe(0x60), "Warning! PWM-controller is unaviable through I2C bus");

  error_manager.check_exec(encoders.begin(PIN_R_A, PIN_R_B, PIN_L_A, PIN_L_B), "Error! Failed to init encoders");
  error_manager.check_exec(controller.begin(), "Error! Failed to init motor controller");

  error_manager.check_exec(INA.begin(), "Error! Failed to init INA219 module.");

  error_manager.check_exec(display.begin((char *)VERSION), "Error! OLED init failed.");

  delay(100);
  serviceWiFi();

  error_manager.check_exec(flashStorage.begin(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R), "Error! NVS init failed.");
  flashStorage.loadPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
  controller.setPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);

  controller.setWheelSpeeds(0.0, 0.0);

  emergency_timer = millis();
  // Отсчёт одометрии и ПИД с конца инициализации, а не с включения питания
  encoders.reset();
  odom_tmr = micros();
}

void loop()
{

  if (ota_started)
  {
    ArduinoOTA.handle();
  }
  serviceWiFi();
  // Приём с обоих каналов
  mainReceiver.update();
  jetsonReceiver.update();

  if ((millis() - emergency_timer > emergency_timer_timeout) || (bus_voltage < min_voltage))
  {
    // Аварийная остановка
    controller.setWheelSpeeds(0.0, 0.0);
  }

  unsigned long now = micros();
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

    // 3. Тики копятся для ПИД: регулятор работает раз в ODOM_RATE_DIVIDER
    //    периодов (100 мс), как и раньше - под этот период подобраны коэффициенты
    pid_left_ticks += leftTicks;
    pid_right_ticks += rightTicks;
    pid_delta += odom_delta;

    if (++odom_counter >= ODOM_RATE_DIVIDER)
    {
      // 3.1. Тики за период ПИД -> частота вращения (об/с) -> скорость колеса (м/с)
      float rightFreq = convert_ticks_to_freq(pid_right_ticks, pid_delta, 'r');
      float leftFreq = convert_ticks_to_freq(pid_left_ticks, pid_delta, 'l');
      float rightLinVel = rightFreq * 2 * Pi * WHEEL_RADIUS;
      float leftLinVel = leftFreq * 2 * Pi * WHEEL_RADIUS;

      // 3.2. Обновляем ПИД-регуляторы и выдаём сигнал на моторы
      controller.update(leftLinVel, rightLinVel);

      // 3.3. Сглаженные скорости для отправки на Jetson
      right_vel_ = controller.getFilteredRight();
      left_vel_ = controller.getFilteredLeft();
      linear_vel_ = (right_vel_ + left_vel_) / 2.0;
      angular_vel_ = (right_vel_ - left_vel_) / WHEEL_BASE; // для дифференциального привода

      odom_counter = 0;
      pid_left_ticks = 0;
      pid_right_ticks = 0;
      pid_delta = 0;
    }

    // 4. Публикация одометрии (50 Гц)
    odomPublish(odometry.x(), odometry.y(), odometry.heading(),
                linear_vel_, angular_vel_, left_vel_, right_vel_);
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

    case Command::UNKNOWN:
      break;
    }
  }

  // Обработка сообщения от Jetson с информацией о wifi
  if (jetsonReceiver.available())
  {
    const char *raw_1 = jetsonReceiver.getMessage();
    parseJetsonMessage(raw_1);
  }

  if ((millis() - update_timer >= update_voltage_timer_timeout) && inaOk)
  {
    update_timer = millis();
    // Чтение напряжения и тока
    bus_voltage = INA.getBusVoltage();
    shunt_v = INA.getShuntVoltage_mV();
    pros = (bus_voltage - min_voltage) / (max_voltage - min_voltage) * 100;
    current = shunt_v / 0.1 / 1000;
  }
  // Мониторинг устройств I2C (не чаще 2 секунд)
  if (millis() - lastI2CCheck >= I2C_CHECK_INTERVAL)
  {
    lastI2CCheck = millis();
    checkI2CDevices();
  }

  // Обновление дисплея (само проверяет интервал)
  if (oledOk)
  {
    display.update(wifi_ssid, wifi_ip, wifi_password, bus_voltage, current, pros);
  }
}
