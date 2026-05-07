#include "protocol_wrapper.h"

void odomPublish(double x_pos_, double y_pos_, double heading_,
                 double linear_vel_x, double angular_vel_z,
                 double left_wheel_velocity, double right_wheel_velocity) {
    static char buf[200];
    int len = snprintf(buf, sizeof(buf),
                       "$1;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;#\r\n",
                       x_pos_, y_pos_, heading_,
                       linear_vel_x, angular_vel_z,
                       left_wheel_velocity, right_wheel_velocity);
    if (len > 0 && len < (int)sizeof(buf)) {
        Serial.write(buf, len);
    }
}

void regulatorsCoefficientsPublish(double Kp_L, double Ki_L, double Kd_L,
                                   double Kp_R, double Ki_R, double Kd_R) {
    static char buf[150];
    int len = snprintf(buf, sizeof(buf),
                       "$2;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;#\r\n",
                       Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R);
    if (len > 0 && len < (int)sizeof(buf)) {
        Serial.write(buf, len);
    }
}