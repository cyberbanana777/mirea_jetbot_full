#ifndef PROTOCOL_WRAPPER_H
#define PROTOCOL_WRAPPER_H

#include <Arduino.h>

// mmsg type = 1
void odomPublish (
  double x_pos_,
  double y_pos_,
  double heading_,
  double linear_vel_x,
  double angular_vel_z,
  double left_wheel_velocity,
  double right_wheel_velocity
  );

// mmsg type = 2
void regulatorsCoefficientsPublish (double Kp_L, double Ki_L, double Kd_L,
                                    double Kp_R, double Ki_R, double Kd_R);

// mmsg type = 3: телеметрия контура скорости (только пока включена командой $12).
// t_us - время такта, мкс (micros(), переполняется); уставка и скорость - м/с;
// duty - скважность ШИМ [-1, 1]. Строка не отправляется, если в буфере UART нет места.
void telemetryPublish (uint32_t t_us,
                       float ref_L, float vel_L, float duty_L,
                       float ref_R, float vel_R, float duty_R);

// mmsg type = 4: параметры feedforward (ответ на $13)
void feedforwardPublish (double gain_L, double deadzone_L,
                         double gain_R, double deadzone_R);

// mmsg type = 6: постоянная времени фильтра уставки, с (ответ на $13)
void refFilterPublish (double tau_s);

#endif