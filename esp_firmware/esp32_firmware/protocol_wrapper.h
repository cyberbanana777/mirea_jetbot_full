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

#endif