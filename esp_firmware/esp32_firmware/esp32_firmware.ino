// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

/**
  @author Alice Zenina and Alexander Grachev RTU MIREA (Russia)
  @date 07.05.2026
  @version 2.0.2
*/
const char* VERSION = "2.0.2";
/**
  @todo

=== Было бы неплохо ===:

1. Индикация наличия связи (heartbeat) на дисплее
  Мигающая точка или значок, показывающий, что команды поступают.
*/

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <stdio.h>
#include "INA219.h"

#include "config.h"
#include "encoders.h"
#include "errors.h"
#include "protocol_wrapper.h"
#include "motor_controller.h"
#include "flash_storage.h"
#include "command_parser.h"
#include "display_manager.h"
#include "serial_receiver.h"


Encoders encoders;
ErrorManager error_manager;
MotorController controller(WHEEL_BASE, MAX_CONSTRUCTIVE_VELOCITY, DT,
                           KP_L, KI_L, KD_L,
                           KP_R, KI_R, KD_R);

INA219 INA(0x41);
FlashStorageManager flashStorage("Memory1");
DisplayManager display(update_voltage_timer_timeout);  // 200 мс интервал обновления
SerialReceiver mainReceiver(Serial);                   // основной канал (USB)
SerialReceiver jetsonReceiver(Serial1);                // Jetson Nano


/**
 * @brief Проверяет наличие устройства на заданном I²C-адресе.
 * @param address - 7-битный адрес устройства (без сдвига, как передаётся в Wire)
 * @return true, если устройство ответило ACK, иначе false
 */
bool i2cProbe(uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  return (error == 0);  // 0 означает успех (получен ACK)
}

// Для таймеров
unsigned long tmr = 0;
unsigned long delta = 0;
unsigned long emergency_timer = 0;
uint64_t update_timer;

double x_pos_ = 0.0;
double y_pos_ = 0.0;
double heading_ = 0.0;

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
const uint8_t MAX_I2C_ERRORS = 3;  // после скольких ошибок инициализировать заново

unsigned long lastI2CCheck = 0;
const unsigned long I2C_CHECK_INTERVAL = 500;  // 0.5 секунды

void checkI2CDevices() {
  // -------- INA219 (0x41) --------
  bool inaResponds = i2cProbe(0x41);
  if (inaResponds) {
    if (!inaOk) {
      // Устройство вернулось без переинициализации (редко, но бывает)
      inaOk = true;
      inaErrors = 0;
    }
  } else {
    inaErrors++;
    if (inaErrors >= MAX_I2C_ERRORS) {
      inaOk = false;
      // Попытка переинициализации
      if (INA.begin()) {
        inaOk = true;
        inaErrors = 0;
        Serial.println("INA219 reinitialized");
      } else {
        Serial.println("INA219 reinit failed");
        // Оставляем inaOk=false, счётчик сбросим после успеха
        inaErrors = MAX_I2C_ERRORS;  // не копить бесконечно
      }
    }
  }

  // -------- PCA9685 (0x60) --------
  bool pwmResponds = i2cProbe(0x60);
  if (pwmResponds) {
    if (!pwmOk) {
      pwmOk = true;
      pwmErrors = 0;
    }
  } else {
    pwmErrors++;
    if (pwmErrors >= MAX_I2C_ERRORS) {
      pwmOk = false;
      if (controller.reinitializePWM()) {
        pwmOk = true;
        pwmErrors = 0;
        // Восстанавливаем управление моторами
        controller.setTargetVelocity(0.0, 0.0);
        // Но update() заново рассчитает ПИД и запишет моторы, что нам и нужно
        Serial.println("PCA9685 reinitialized");
      } else {
        Serial.println("PCA9685 reinit failed");
        pwmErrors = MAX_I2C_ERRORS;
      }
    }
  }

  // -------- OLED (0x3C) --------
  bool oledResponds = i2cProbe(0x3C);
  if (oledResponds) {
    if (!oledOk) {
      oledOk = true;
      oledErrors = 0;
    }
  } else {
    oledErrors++;
    if (oledErrors >= MAX_I2C_ERRORS) {
      oledOk = false;
      if (display.reinitialize((char*)VERSION)) {
        oledOk = true;
        oledErrors = 0;
        Serial.println("OLED reinitialized");
      } else {
        Serial.println("OLED reinit failed");
        oledErrors = MAX_I2C_ERRORS;
      }
    }
  }
}

void parseJetsonMessage(const char* fullMessage) {
  if (!fullMessage || fullMessage[0] != '$') return;
  // Копируем во временный буфер, чтобы не портить оригинал (если нужно)
  char buf[128];
  strncpy(buf, fullMessage, 127);
  buf[127] = '\0';

  // Удаляем '$' и '#'
  char* start = buf + 1;
  char* end = strchr(start, '#');
  if (end) *end = '\0';

  char* parts[4];
  int count = 0;
  char* token = strtok(start, ";");
  while (token && count < 4) {
    parts[count++] = token;
    token = strtok(nullptr, ";");
  }
  if (count >= 4) {
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
float convert_ticks_to_freq(long ticks, long timeout_micros, char dir) {
  const float steps_per_tick = 1.0 / 4.0;   // 4 импульса на 1 шаг
  const float turn_per_step = 1.0 / 270.0;  // 270 шагов в 1 полном обороте
  const double micros_to_sec = 1.0 / 1000000.0;
  float timeout_sec = timeout_micros * micros_to_sec;
  float result = turn_per_step * steps_per_tick * ticks / timeout_sec;

  // для левого колеса инвертируем частоту
  if (dir == 'l') {
    result = result * (-1);
  } else if (dir == 'r') {
    result = result * 1;
  }

  return result;
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  error_manager.begin(ERROR_LED_PIN);

  error_manager.check_exec(Wire.begin(), "Error! Failed to init I2C bus.");
  Wire.setTimeOut(50);  // 50 мс таймаут на транзакцию

  error_manager.soft_check_exec(i2cProbe(0x3C), "Warning! OLED-display is unaviable through I2C bus");
  error_manager.soft_check_exec(i2cProbe(0x41), "Warning! INA219 sensor is unaviable through I2C bus");
  error_manager.soft_check_exec(i2cProbe(0x60), "Warning! PWM-controller is unaviable through I2C bus");

  error_manager.check_exec(encoders.begin(PIN_R_A, PIN_R_B, PIN_L_A, PIN_L_B), "Error! Failed to init encoders");
  error_manager.check_exec(controller.begin(), "Error! Failed to init motor controller");
  error_manager.check_exec(INA.begin(), "Error! Failed to init INA219 module.");

  error_manager.check_exec(display.begin((char*)VERSION), "Error! OLED init failed.");

  delay(100);

  error_manager.check_exec(flashStorage.begin(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R), "Error! NVS init failed.");
  flashStorage.loadPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);
  controller.setPID(KP_L, KI_L, KD_L, KP_R, KI_R, KD_R);

  controller.setWheelSpeeds(0.0, 0.0);
  emergency_timer = millis();
}


void loop() {
  // Приём с обоих каналов
  mainReceiver.update();
  jetsonReceiver.update();

  if ((millis() - emergency_timer > emergency_timer_timeout) || (bus_voltage < min_voltage)) {
    // Аварийная остановка
    controller.setWheelSpeeds(0.0, 0.0);
  }

  delta = micros() - tmr;

  if (delta >= timer_timeout) {
    tmr = micros();

    int32_t rightTicks = encoders.getRightAndReset();
    int32_t leftTicks = encoders.getLeftAndReset();

    // 2. Преобразуем тики в частоту вращения (об/с)
    float rightFreq = convert_ticks_to_freq(rightTicks, delta, 'r');
    float leftFreq = convert_ticks_to_freq(leftTicks, delta, 'l');

    // 3. Вычисляем линейные скорости колёс (м/с)
    float rightLinVel = rightFreq * 2 * Pi * WHEEL_RADIUS;
    float leftLinVel = leftFreq * 2 * Pi * WHEEL_RADIUS;

    // 4. Обновляем ПИД-регуляторы и выдаём сигнал на моторы
    controller.update(leftLinVel, rightLinVel);
    rightLinVel = controller.getFilteredRight();
    leftLinVel = controller.getFilteredLeft();

    // 5. Одометрия
    double linearVel = (rightLinVel + leftLinVel) / 2.0;
    double angularVel = (rightLinVel - leftLinVel) / WHEEL_BASE;  // для дифференциального привода
    double dtSec = timer_timeout / 1e6;                           // период в секундах
    double deltaHeading = angularVel * dtSec;
    double deltaX = linearVel * cos(heading_) * dtSec;
    double deltaY = linearVel * sin(heading_) * dtSec;

    x_pos_ += deltaX;
    y_pos_ += deltaY;
    heading_ += deltaHeading;

    // 6. Публикация данных одометрии
    odomPublish(x_pos_, y_pos_, heading_, linearVel, angularVel,
                leftLinVel, rightLinVel);
  }


  //Обработка входного значения
  if (mainReceiver.available()) {
    emergency_timer = millis();
    const char* raw_0 = mainReceiver.getMessage();  // теперь это C-строка
    ParsedCommand cmd = CommandParser::parse(raw_0);

    switch (cmd.type) {
      case Command::SET_TWIST:
        if (cmd.count >= 2)
          controller.setTargetVelocity(cmd.args[0], cmd.args[1]);
        break;

      case Command::SET_WHEEL_SPEEDS:
        if (cmd.count >= 2)
          controller.setWheelSpeeds(cmd.args[0], cmd.args[1]);
        break;

      case Command::SET_PID_TEMP:
        if (cmd.count >= 6) {
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
        if (cmd.count >= 6) {
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
  if (jetsonReceiver.available()) {
    const char* raw_1 = jetsonReceiver.getMessage();
    parseJetsonMessage(raw_1);
  }


  if ((millis() - update_timer >= update_voltage_timer_timeout) && inaOk) {
    update_timer = millis();
    // Чтение напряжения и тока
    bus_voltage = INA.getBusVoltage();
    shunt_v = INA.getShuntVoltage_mV();
    pros = (bus_voltage - min_voltage) / (max_voltage - min_voltage) * 100;
    current = shunt_v / 0.1 / 1000;
  }
  // Мониторинг устройств I2C (не чаще 2 секунд)
  if (millis() - lastI2CCheck >= I2C_CHECK_INTERVAL) {
    lastI2CCheck = millis();
    checkI2CDevices();
  }

  // Обновление дисплея (само проверяет интервал)
  if (oledOk) {
    display.update(wifi_ssid, wifi_ip, wifi_password, bus_voltage, current, pros);
  }
}
