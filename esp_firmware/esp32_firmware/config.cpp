// Exactly one definition for each mutable runtime configuration value.
// Defaults are unchanged; NVS may overwrite them during setup().
#include "config.h"

float KP_R = 1.426;
float KI_R = 10.009;
float KD_R = 0.0;
float KP_L = 0.951;
float KI_L = 5.475;
float KD_L = 0.0;
float DT = 10;
float FF_DEADZONE_L = 0.2992;
float FF_DEADZONE_R = 0.3101;
float FF_GAIN_L = 0.5151;
float FF_GAIN_R = 0.8424;
float REF_TAU_S = 0.0;
int CTRL_MODE = 1;
float ADRC_WC = 9.0;
float ADRC_WO = 40.0;
float ADRC_B0_L = 9.82;
float ADRC_B0_R = 7.65;
float ADRC_FF = 2.0;
