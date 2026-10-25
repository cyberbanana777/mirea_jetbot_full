#ifndef CONFIG_H
#define CONFIG_H
//Константы физических параметров
const float Pi = 3.14159;
const float WHEEL_BASE = 0.1775;  // длина базы
const float WHEEL_RADIUS = 0.065/2.0; // радиус ведущего колеса
const float max_voltage = 4.2 * 3;
const float min_voltage = 3.3 * 3;

const int ERROR_LED_PIN = 2; // Светодиод D2 на плате: не горит - норма, мигает - ошибка (см. errors.h)

// Подключение энкодеров
const uint8_t PIN_R_A = 34;  // Правое колесо, канал А
const uint8_t PIN_R_B = 35;  // Правое колесо, канал В
const uint8_t PIN_L_A = 32;  // Левое колесо, канал А
const uint8_t PIN_L_B = 33;  // Левое колесо, канал В

// Коэффициенты по умолчанию для регуляторов
float KP_R = 0.8;
float KI_R = 1.5;
float KD_R = 0.04;

float KP_L = 0.85; 
float KI_L = 1.5;
float KD_L = 0.05;

// Частота дискретизации для ПИДов
float DT = 100;

const double MAX_CONSTRUCTIVE_VELOCITY = 0.5; // м/сек

// === Таймеры ===
// Период ПИД-регуляторов (мкс). Коэффициенты ПИД подобраны под этот период.
const unsigned long timer_timeout = DT * 1000; //мкс
// Одометрия считается и отправляется в ODOM_RATE_DIVIDER раз чаще ПИД:
// 100 мс / 5 = 20 мс (50 Гц). ПИД по-прежнему работает раз в 100 мс.
const uint8_t ODOM_RATE_DIVIDER = 5;
const unsigned long odom_timer_timeout = timer_timeout / ODOM_RATE_DIVIDER; //мкс
// Аварийная остановка, если команды не приходят дольше 1 секунды
const unsigned int emergency_timer_timeout = 1000;
const unsigned int update_voltage_timer_timeout = 200;


#endif