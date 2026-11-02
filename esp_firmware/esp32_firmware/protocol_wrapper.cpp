#include "protocol_wrapper.h"


void odomPublish(double x_pos_, double y_pos_, double heading_,
                 double linear_vel_x, double angular_vel_z,
                 double left_wheel_velocity, double right_wheel_velocity) {
    static char buf[200];
    int len = snprintf(buf, sizeof(buf),
                       "$%u;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;#\r\n",
                       (unsigned int)301, x_pos_, y_pos_, heading_,
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
                       "$%u;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;#\r\n",
                       (unsigned int)202, Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R);
    if (len > 0 && len < (int)sizeof(buf)) {
        Serial.write(buf, len);
    }
}

void telemetryPublish(uint32_t t_us,
                      float ref_L, float vel_L, float duty_L,
                      float ref_R, float vel_R, float duty_R) {
    static char buf[110];
    int len = snprintf(buf, sizeof(buf),
                       "$%u;%lu;%.4f;%.4f;%.3f;%.4f;%.4f;%.3f;#\r\n",
                       (unsigned int)401, (unsigned long)t_us,
                       ref_L, vel_L, duty_L, ref_R, vel_R, duty_R);
    // Не блокируем контур: если буфер UART занят, эту строку пропускаем
    if (len > 0 && len < (int)sizeof(buf) && Serial.availableForWrite() >= len) {
        Serial.write(buf, len);
    }
}

void feedforwardPublish(double gain_L, double deadzone_L,
                        double gain_R, double deadzone_R) {
    static char buf[110];
    int len = snprintf(buf, sizeof(buf),
                       "$%u;%.5f;%.5f;%.5f;%.5f;#\r\n",
                       (unsigned int)205, gain_L, deadzone_L, gain_R, deadzone_R);
    if (len > 0 && len < (int)sizeof(buf)) {
        Serial.write(buf, len);
    }
}

void refFilterPublish(double tau_s) {
    static char buf[40];
    int len = snprintf(buf, sizeof(buf), "$%u;%.5f;#\r\n", (unsigned int)207, tau_s);
    if (len > 0 && len < (int)sizeof(buf)) {
        Serial.write(buf, len);
    }
}

void adrcPublish(int mode, double wc, double wo, double b0_L, double b0_R, int ff) {
    static char buf[90];
    int len = snprintf(buf, sizeof(buf), "$%u;%d;%.3f;%.3f;%.4f;%.4f;%d;#\r\n",
                       (unsigned int)203, mode, wc, wo, b0_L, b0_R, ff);
    if (len > 0 && len < (int)sizeof(buf)) {
        Serial.write(buf, len);
    }
}

void speedWindowPublish(int min_edges, int align) {
    static char buf[28];
    int len = snprintf(buf, sizeof(buf), "$%u;%d;%d;#\r\n", (unsigned int)404, min_edges, align);
    if (len > 0 && len < (int)sizeof(buf)) {
        Serial.write(buf, len);
    }
}

// Numeric-only frames maintain compatibility with the strict Python decoder.
void deviceInfoPublish() {
    static const char msg[] = "$501;2;2;#\r\n"; // major=2, minor=1
    if (Serial.availableForWrite() >= (int)(sizeof(msg) - 1))
        Serial.write((const uint8_t*)msg, sizeof(msg) - 1);
}

void diagnosticSnapshotPublish(uint32_t uptime_ms, uint32_t accepted, uint32_t rejected,
                               uint32_t watchdog_stops, bool motor_stopped,
                               bool ina_ok, bool pwm_ok) {
    char buf[120];
    const int n = snprintf(buf, sizeof(buf), "$402;%lu;%lu;%lu;%lu;%u;%u;%u;#\r\n",
      (unsigned long)uptime_ms, (unsigned long)accepted, (unsigned long)rejected,
      (unsigned long)watchdog_stops, motor_stopped ? 1U : 0U, ina_ok ? 1U : 0U, pwm_ok ? 1U : 0U);
    if (n > 0 && n < (int)sizeof(buf) && Serial.availableForWrite() >= n)
        Serial.write((const uint8_t*)buf, n);
}

void configAckPublish(uint16_t seq, uint16_t command) {
    char buf[42];
    int n = snprintf(buf, sizeof(buf), "$502;%u;%u;#\r\n", (unsigned)seq, (unsigned)command);
    if (n > 0 && n < (int)sizeof(buf)) Serial.write((const uint8_t*)buf, n);
}
void configNackPublish(uint16_t seq, uint16_t command, uint8_t error) {
    char buf[46];
    int n = snprintf(buf, sizeof(buf), "$503;%u;%u;%u;#\r\n", (unsigned)seq, (unsigned)command, (unsigned)error);
    if (n > 0 && n < (int)sizeof(buf)) Serial.write((const uint8_t*)buf, n);
}
void transactionalPidPublish(uint16_t seq, double kpL, double kiL, double kdL,
                             double kpR, double kiR, double kdR) {
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "$211;%u;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;#\r\n",
        (unsigned)seq, kpL, kiL, kdL, kpR, kiR, kdR);
    if (n > 0 && n < (int)sizeof(buf)) Serial.write((const uint8_t*)buf, n);
}

// Read-only I2C snapshot. Numeric-only, <=192 bytes even at saturated counters.
void i2cDiagnosticsPublish(const I2CDeviceDiagnostics &ina, const I2CDeviceDiagnostics &pwm,
                           const I2CDeviceDiagnostics &oled) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "$405;%u;%lu;%lu;%lu;%u;%lu;%lu;%lu;%u;%lu;%lu;%lu;#\r\n",
        (unsigned)ina.state, (unsigned long)ina.probeFailures,
        (unsigned long)ina.recoveryAttempts, (unsigned long)ina.successfulRecoveries,
        (unsigned)pwm.state, (unsigned long)pwm.probeFailures,
        (unsigned long)pwm.recoveryAttempts, (unsigned long)pwm.successfulRecoveries,
        (unsigned)oled.state, (unsigned long)oled.probeFailures,
        (unsigned long)oled.recoveryAttempts, (unsigned long)oled.successfulRecoveries);
    if (n > 0 && n < (int)sizeof(buf) && Serial.availableForWrite() >= n)
        Serial.write((const uint8_t*)buf, n);
}
