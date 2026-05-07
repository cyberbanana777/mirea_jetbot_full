#include "protocol_wrapper.h"
#include <Arduino.h>

String feedback_msg_str = "";

void odomPublish (
  double x_pos_,
  double y_pos_,
  double heading_,
  double linear_vel_x,
  double angular_vel_z,
  double left_wheel_velocity,
  double right_wheel_velocity
  ) {
  feedback_msg_str = 
  "$1;" +
  String(x_pos_, 5) + ';' +
  String(y_pos_, 5) + ';' +
  String(heading_, 5) + ';' +
  String(linear_vel_x, 5) + ';' +
  String(angular_vel_z, 5) + ';' + 
  String(left_wheel_velocity, 5) + ';' + 
  String(right_wheel_velocity, 5) + ';' + 
  "#";
  Serial.println(feedback_msg_str);
}

void regulatorsCoefficientsPublish (double Kp_L, double Ki_L, double Kd_L,
                                    double Kp_R, double Ki_R, double Kd_R) {
  feedback_msg_str = 
  "$2;" +
  String(Kp_L, 5) + ';' +
  String(Ki_L, 5) + ';' +
  String(Kd_L, 5) + ';' +
  String(Kp_R, 5) + ';' +
  String(Ki_R, 5) + ';' + 
  String(Kd_R, 5) + ';' + 
  "#";
  Serial.println(feedback_msg_str);
}