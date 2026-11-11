#ifndef PROTOCOL_WRAPPER_H
#define PROTOCOL_WRAPPER_H

#include <Arduino.h>
#include "hardware_monitor.h"

// Main UART grouped API v2.2; Serial1 has a separate protocol.
void deviceInfoPublish();
// 405: three device groups (INA219, PCA9685, OLED), each state/failures/attempts/recoveries.
void i2cDiagnosticsPublish(const I2CDeviceDiagnostics &ina, const I2CDeviceDiagnostics &pwm,
                           const I2CDeviceDiagnostics &oled);
// Transactional config: 502 ACK(seq,command_id), 503 NACK(seq,command_id,error).
// 211 PID snapshot(seq, six gains). Error 1: invalid parameter.
void configAckPublish(uint16_t seq, uint16_t command);
void configNackPublish(uint16_t seq, uint16_t command, uint8_t error);
void transactionalPidPublish(uint16_t seq, double kpL, double kiL, double kdL,
                             double kpR, double kiR, double kdR);
void diagnosticSnapshotPublish(uint32_t uptime_ms, uint32_t accepted, uint32_t rejected,
                               uint32_t watchdog_stops, bool motor_stopped,
                               bool ina_ok, bool pwm_ok);

// кадр $301: одометрия
void odomPublish (
  double x_pos_,
  double y_pos_,
  double heading_,
  double linear_vel_x,
  double angular_vel_z,
  double left_wheel_velocity,
  double right_wheel_velocity
  );

// кадр $202: коэффициенты ПИД
void regulatorsCoefficientsPublish (double Kp_L, double Ki_L, double Kd_L,
                                    double Kp_R, double Ki_R, double Kd_R);

// кадр $401: телеметрия контура скорости (только пока включена командой $403;1;#).
// t_us - время такта, мкс (micros(), переполняется); уставка и скорость - м/с;
// duty - скважность ШИМ [-1, 1]. Строка не отправляется, если в буфере UART нет места.
void telemetryPublish (uint32_t t_us,
                       float ref_L, float vel_L, float duty_L,
                       float ref_R, float vel_R, float duty_R);

// кадр $205: параметры feedforward (ответ на $206)
void feedforwardPublish (double gain_L, double deadzone_L,
                         double gain_R, double deadzone_R);

// кадр $207: постоянная времени фильтра уставки, с (ответ на $206)
void refFilterPublish (double tau_s);

// кадр $203: выбранный регулятор и параметры LADRC (ответ на $206 и $209)
// $203;mode;wc;wo;b0_L;b0_R;ff;#   mode: 0 - ПИ, 1 - LADRC; ff: 0 - без прямой подачи, 1 - мёртвая зона, 2 - полный FF
void adrcPublish (int mode, double wc, double wo, double b0_L, double b0_R, int ff);

// кадр $404: окно оценки скорости (ответ на $404): $404;min_edges;align;#. Отладка шума скорости (tools/noisetest.py)
void speedWindowPublish (int min_edges, int align);

#endif