// Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>
#include "GyverPID.h"
#include <Wire.h>                     
#include <Adafruit_PWMServoDriver.h> 
#include <stdio.h>

// перед подключением библиотеки можно добавить настройки:
// сделает часть вычислений целочисленными, что чуть (совсем чуть!) ускорит код
// #define PID_INTEGER

// режим, при котором интегральная составляющая суммируется только в пределах указанного количества значений
#define PID_INTEGRAL_WINDOW 40

#include <GyverOLED.h>
#include "INA219.h"


//Константы физических параметров
const float Pi = 3.14159;
const float l = 0.117;  // длина базы
const float r = 0.065/2; // радиус ведущего колеса
const float max_voltage = 4.2 * 3;
const float min_voltage = 3.3 * 3;


// Пины энкодеров
#define PIN_R_A 34  // Пин для сигнала 1
#define PIN_R_B 35  // Пин для сигнала 2
#define PIN_L_A 32  // Пин для сигнала 3
#define PIN_L_B 33  // Пин для сигнала 4
#define LED_PIN 2 // Оставлен для debug


// Коэффициенты по умолчанию для регуляторов
float Kp_R = 0.5;
float Ki_R = 0.0;
float Kd_R = 0.000;

float Kp_L = 0.5; 
float Ki_L = 0.0;
float Kd_L = 0.000;

// Частота дискретизации для ПИДов
float dt = 30; // Было 5


// === Таймеры ===
const unsigned long timer_timeout = dt * 1000; //мкс
// Для остановки после 5 секунд простоя
const unsigned int im_timer_timeout = 5000;
const unsigned long displayInterval = 200; 

// Фильтры для скоростей колёс (экспоненциальное сглаживание)
float filtered_FreqRight = 0.0;
float filtered_FreqLeft  = 0.0;
// const float alpha = 0.05;  // коэффициент фильтра (0...1). Чем меньше, тем сильнее сглаживание

// Простое скользящее среднее для скоростей колёс
const int WINDOW_SIZE = 5;               // размер окна (количество измерений)
float freq_buffer_R[WINDOW_SIZE];        // буфер для правого колеса
float freq_buffer_L[WINDOW_SIZE];        // буфер для левого колеса
int buffer_index = 0;                    // текущий индекс в буфере
bool buffer_filled = false;              // флаг, что буфер полностью заполнен


// =======================================================================================================================
// Глобальные счётчики меток 
long int global_pos_R = 0;
long int global_pos_L = 0;

void init_interraptions(){
  // Объявление прерываний
  attachInterrupt(PIN_R_A, Read_R_A, CHANGE);
  attachInterrupt(PIN_R_B, Read_R_B, CHANGE);
  attachInterrupt(PIN_L_A, Read_L_A, CHANGE);
  attachInterrupt(PIN_L_B, Read_L_B, CHANGE);
  // Пины энкодеров
  pinMode(PIN_R_A, INPUT);
  pinMode(PIN_R_B, INPUT);
  pinMode(PIN_L_A, INPUT);
  pinMode(PIN_L_B, INPUT);
}



// ========================================================================================

// Переменные скорости правые
float RealFrequencyRight = 0.0;
float TargetRight = 0.0;

// Переменные скорости левые
float RealFrequencyLeft = 0.0;
float TargetLeft = 0.0;

double max_contructive_velocity = 1.0;
float max_frequency = 2.5;
// float max_vel = max_frequency * 2 * Pi * r;
float max_vel = 1.0;

// Правый ПИД
GyverPID regulator_R(Kp_R, Ki_R, Kd_R, dt);

// Левый ПИД
GyverPID regulator_L(Kp_L, Ki_L, Kd_L, dt);


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);

#define MOTOR_L 0x00
#define MOTOR_R 0x02

INA219 INA(0x41);


// ========================================================================================================

GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;  // SSD1306 128x32 с буферизацией

bool init_oled(){
  oled.init();            // инициализация дисплея
  oled.clear();           // очистка буфера
  oled.home();            // курсор в (0,0)
  oled.print("ESP32 ready old");
  oled.update();          // вывод на экран
  delay(1000);
  return true;
}


String pwd_short;
int pros;
float bus_voltage, shunt_v, current;


// Настройки для записи в постоянную память
#define RW_MODE false
#define RO_MODE true
Preferences fleshMemory;

// Для таймеров
unsigned long tmr = 0;
unsigned long delta = 0;
unsigned long lastDisplayUpdate = 0;
unsigned int im_timer = 0;

double x_pos_ = 0.0;
double y_pos_ = 0.0;
double heading_ = 0.0;

// Переменные для хранения принятых данных
String wifi_ssid = "";
String wifi_password = "";
String wifi_ip = "";

String feedback_msg_str = "";
String input_m[8] = {"", "", "", "", "", "", "", ""};
String input_j[3] = {"", "", ""};



// ===== Переменные для приёма с Jetson через Serial (USB) - основной канал связи =====
String mainInputString = "";         // a String to hold incoming data
bool mainStringComplete = false;  // whether the string is complete
bool mainInputOpen = false;
// ====================================================================================

// ===== Переменные для приёма с Jetson через Serial2 - дополнительный канал связи=====
String addictionInputString = "";
bool addictionStringComplete = false;
bool addictionInputOpen = false;
// ====================================================================================



//Ответ на входное сообщение по Serial0 (USB)
void serialEvent() {
  im_timer = millis();
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(inChar == '$' or mainInputOpen == true){
      // add it to the mainInputString:
      mainInputString += inChar;
      mainInputOpen = true;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '#') {
      mainStringComplete = true;
      mainInputOpen = false;
    }
  }
}


void parseJetsonMessage(String input){
  String trimmed = addictionInputString.substring(1, addictionInputString.length() - 2);
  String parts[4]; // массив для 4 элементов
  int count = 0;   // сколько уже сохранили

  int start = 0;
  int semicolon = trimmed.indexOf(';');

  while (semicolon != -1 && count < 4) {
    parts[count++] = trimmed.substring(start, semicolon);
    start = semicolon + 1;
    semicolon = trimmed.indexOf(';', start);
  }

  // Последняя часть
  if (count < 4) {
    parts[count++] = trimmed.substring(start);
  }

  // Теперь можно обратиться к элементам массива
  // или, если очень хочется, присвоить отдельным переменным:
  // String msg_type = parts[0];
  wifi_ssid = parts[1];
  wifi_password = parts[2];
  wifi_ip = parts[3];
}

float convert_ticks_to_freq(long ticks, long timeout_micros, char dir){
  /*
  ticks - счётчик тактов из прерываний
  timeout_micros - время замера количества меток
  dir - направеление. Может быть 'r', 'l'. При 'l'  происходит инвертирование значения, 
  необходиомое для нормальной работы логики. Чтобы при движении вперёд или назад
  знаки угловых скоростей колёс были одинаковые
  */
  const float steps_per_tick = 1.0 / 4.0; // 4 импульса на 1 шаг
  const float turn_per_step = 1.0 / 270.0; // 270 шагов в 1 полном обороте
  const double micros_to_sec= 1.0 / 1000000.0; 
  float timeout_sec = timeout_micros * micros_to_sec;
  float result = turn_per_step * steps_per_tick * ticks / timeout_sec;
  
  // для левого колеса инвертируем частоту
  if(dir == 'l'){
    result = result * (-1);
  }
  else if(dir == 'r'){
    result = result * 1;
  }
  else{
    raise_error("Error in 'convert_ticks_to_freq'! Invalid direcrion (dir)");
  }

  return result;
}

void read_serial1(){
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    if (inChar == '$' || addictionInputOpen) {
      addictionInputString += inChar;
      addictionInputOpen = true;
    }
    if (inChar == '#') {
      addictionStringComplete = true;
      addictionInputOpen = false;
    }
  }
}

void print_telemetry(){
  oled.clear();
  oled.home();  // курсор в (0,0)

  oled.print("SSID: ");
  oled.println(wifi_ssid.substring(0, 15));  // обрезаем до 16 символов

  oled.print("IP: ");
  oled.println(wifi_ip);

  oled.print("PWD: ");
  pwd_short = wifi_password.substring(0, 14);
  if (wifi_password.length() > 14) pwd_short += "...";
  oled.println(pwd_short);

  oled.print("Bat: ");
  bus_voltage = INA.getBusVoltage();
  shunt_v = INA.getShuntVoltage_mV();

  pros = (bus_voltage - min_voltage) / (max_voltage - min_voltage) * 100;
  current = shunt_v / 0.1 / 1000;
  oled.print(bus_voltage);
  oled.print("V ");
  oled.print(pros);
  oled.print("% ");
  oled.print(current);
  oled.println("A");
  oled.update();  // отправляем буфер на дисплей
}



void raise_error(String error_text){
  while(true){
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    Serial.println(error_text);
  }
}

void check_exec(bool trigger, String error_text){
  if(!trigger){
    raise_error(error_text);
  }
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  init_interraptions();
  init_motor_control();

  // Светодиод ошибки
  pinMode(LED_PIN, OUTPUT); // Индикатор ошибок

  // ===================== ИНИЦИАЛИЗАЦИЯ ДИСПЛЕЯ (GyverOLED) и ЦИФРОВОГО МУЛЬТИМЕТРА ====================
  check_exec(Wire.begin(), "Error! Failed to init I2C bus.");           // I2C на пинах 21 (SDA), 22 (SCL)
  check_exec(INA.begin(), "Error! Failed to init INA219 module.");  
  init_oled();
  // =========================================================================


  mainInputString.reserve(250);
  addictionInputString.reserve(250);
  
  delay(100);

  // =================== РАБОТА С ПОСТОЯННОЙ ПАМЯТЬЮ ==========================
  fleshMemory.begin("Memory1", RO_MODE);
  bool tpInit = fleshMemory.isKey("nvsInit");

  // Запись в постоянную память коэффициентов ПИД-регуляторов
  if (tpInit == false) {
 
    fleshMemory.end();                             
    fleshMemory.begin("Memory1", RW_MODE);

    fleshMemory.putFloat("Kp_R", Kp_R); 
    fleshMemory.putFloat("Ki_R", Ki_R); 
    fleshMemory.putFloat("Kd_R", Kd_R); 

    fleshMemory.putFloat("Kp_L", Kp_L);
    fleshMemory.putFloat("Ki_L", Ki_L); 
    fleshMemory.putFloat("Kd_L", Kd_L); 

    fleshMemory.putBool("nvsInit", true);
    
    fleshMemory.end();
  } 


  // Получение коэффициентов ПИД-регуляторов из постоянной памяти
  else {

    regulator_R.Kp = fleshMemory.getFloat("Kp_R"); 
    regulator_R.Ki = fleshMemory.getFloat("Ki_R"); 
    regulator_R.Kd = fleshMemory.getFloat("Kd_R"); 

    regulator_L.Kp = fleshMemory.getFloat("Kp_L"); 
    regulator_L.Ki = fleshMemory.getFloat("Ki_L"); 
    regulator_L.Kd = fleshMemory.getFloat("Kd_L"); 

    fleshMemory.end();
  }
  // ==========================================================================
  motorWrite(MOTOR_R, 0.0);
  motorWrite(MOTOR_L, 0.0);

  // Очистка буферов скользящего среднего
  for (int i = 0; i < WINDOW_SIZE; i++) {
      freq_buffer_R[i] = 0.0;
      freq_buffer_L[i] = 0.0;
}
}



void loop() {

  if (millis() - im_timer > im_timer_timeout){
    // Аварийная остановка
    TargetRight = 0.0;
    TargetLeft = 0.0;
  }

  delta = micros() - tmr;
  
  if (delta >= timer_timeout){  
    tmr = micros();

    // Настройка ПИДов: Установка целевых значений
    regulator_R.setpoint = TargetRight;  
    regulator_L.setpoint = TargetLeft; 
    
    //Подсчёт скорости

    RealFrequencyRight = convert_ticks_to_freq(global_pos_R, timer_timeout, 'r'); 
    RealFrequencyLeft = convert_ticks_to_freq(global_pos_L, timer_timeout, 'l'); 
    global_pos_R = 0;
    global_pos_L = 0;

    // Применяем экспоненциальный фильтр к сырой частоте (или сразу к нормированной)
    // filtered_FreqRight = alpha * RealFrequencyRight + (1.0 - alpha) * filtered_FreqRight;
    // filtered_FreqLeft  = alpha * RealFrequencyLeft  + (1.0 - alpha) * filtered_FreqLeft;

    // === ПРОСТОЕ СКОЛЬЗЯЩЕЕ СРЕДНЕЕ ===
    // Сохраняем новое измерение в буфер
    freq_buffer_R[buffer_index] = RealFrequencyRight;
    freq_buffer_L[buffer_index] = RealFrequencyLeft;

    // Увеличиваем индекс и зацикливаем
    buffer_index++;
    if (buffer_index >= WINDOW_SIZE) {
        buffer_index = 0;
        buffer_filled = true;   // после первого полного прохода буфер считается заполненным
    }

    // Вычисляем среднее, если буфер заполнен хотя бы один раз
    if (buffer_filled) {
        float sum_R = 0.0, sum_L = 0.0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            sum_R += freq_buffer_R[i];
            sum_L += freq_buffer_L[i];
        }
        filtered_FreqRight = sum_R / WINDOW_SIZE;
        filtered_FreqLeft  = sum_L / WINDOW_SIZE;
    } else {
        // Пока буфер не заполнен, используем сырые данные (или можно частичное среднее)
        float sum_R = 0.0, sum_L = 0.0;
        for (int i = 0; i <= buffer_index; i++) {
            sum_R += freq_buffer_R[i];
            sum_L += freq_buffer_L[i];
        }
        filtered_FreqRight = sum_R / (buffer_index + 1);
        filtered_FreqLeft  = sum_L / (buffer_index + 1);
    }

    
    double vel_dt = timer_timeout/1000; // ms
    double linear_vel_x = (filtered_FreqRight + filtered_FreqLeft)*2*Pi*r/2;
    double angular_vel_z = (filtered_FreqRight - filtered_FreqLeft)*2*Pi*r/l;
    double left_wheel_angular_velocity_rad = filtered_FreqLeft * 2 * Pi * r;
    double right_wheel_angular_velocity_rad =filtered_FreqRight  * 2 * Pi * r;
    double delta_heading = angular_vel_z * vel_dt/1000; //radians
    double cos_h = cos(heading_);
    double sin_h = sin(heading_);
    double delta_x = (linear_vel_x * cos_h) * vel_dt/1000; //m
    double delta_y = (linear_vel_x * sin_h) * vel_dt/1000; //m

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;
    odomPublish(x_pos_, y_pos_, heading_, linear_vel_x, angular_vel_z, left_wheel_angular_velocity_rad, right_wheel_angular_velocity_rad);
    // Serial.println(left_wheel_angular_velocity_rad);

    // Отправка в ПИДы расчитанного значения скорости
    regulator_R.input = filtered_FreqRight / max_contructive_velocity;
    regulator_L.input = filtered_FreqLeft / max_contructive_velocity;

    // Подача на моторы "исправленного" сигнал
    motorWrite(MOTOR_R, regulator_R.getResultNow());
    motorWrite(MOTOR_L, regulator_L.getResultNow());
  }

    //Обработка входного значения
  if (mainStringComplete) {

    unsigned int i = 0;
    unsigned int j = 0;

    // msg_type
    for (i = 1; i < mainInputString.length()-1; i++){
      if (mainInputString[i] == ';') break;
      input_m[0] += mainInputString[i];
    }
    char str1[input_m[0].length() + 1];
    for(j=0; j < input_m[0].length(); j++) str1[j] = input_m[0][j];
    str1[input_m[0].length()] = '\0';
    input_m[0] = "";

    int msg_type = (int)atof(str1);

    if (msg_type == 1){
      // управление линейной и угловой скоростью платформы

      unsigned short num_flied = 2;

      for (j = 1; j <= num_flied; j++){
        for (i = i+1; i < mainInputString.length()-1; i++){
          if (mainInputString[i] == ';') break;
          input_m[j] += mainInputString[i];
        }
      }

      char str2[input_m[1].length() + 1]; 
      char str3[input_m[2].length() + 1];
      
      for(j=0; j < input_m[1].length(); j++) str2[j] = input_m[1][j];
      for(j=0; j < input_m[2].length(); j++) str3[j] = input_m[2][j];      

      str2[input_m[1].length()] = '\0';
      str3[input_m[2].length()] = '\0';

      for (j = 1; j <= num_flied; j++){
        input_m[j] = "";
      }

      speed_converter(atof(str2), atof(str3));
    }

    else if (msg_type == 2){
      // управление скоростями каждого из колёс

      unsigned short num_flied = 2;

      for (j = 1; j <= num_flied; j++){
        for (i = i+1; i < mainInputString.length()-1; i++){
          if (mainInputString[i] == ';') break;
          input_m[j] += mainInputString[i];
        }
      }
      char str2[input_m[1].length() + 1]; 
      char str3[input_m[2].length() + 1];
      
      for(j=0; j < input_m[1].length(); j++) str2[j] = input_m[1][j];
      for(j=0; j < input_m[2].length(); j++) str3[j] = input_m[2][j]; 

      str2[input_m[1].length()] = '\0';
      str3[input_m[2].length()] = '\0';

      for (j = 1; j <= num_flied; j++){
        input_m[j] = "";
      }

      cut_speeds(atof(str2), atof(str3));
    }

    else if (msg_type == 3){
      // Установка новых PID-коэффициентов во временную память

      unsigned short num_flied = 6;

      for (j = 1; j <= num_flied; j++){
        for (i = i+1; i < mainInputString.length()-1; i++){
          if (mainInputString[i] == ';') break;
          input_m[j] += mainInputString[i];
        }
      }

      char str2[input_m[1].length() + 1]; 
      char str3[input_m[2].length() + 1];
      char str4[input_m[3].length() + 1]; 
      char str5[input_m[4].length() + 1]; 
      char str6[input_m[5].length() + 1]; 
      char str7[input_m[6].length() + 1]; 

      for(j=0; j < input_m[1].length(); j++) str2[j] = input_m[1][j];
      for(j=0; j < input_m[2].length(); j++) str3[j] = input_m[2][j]; 
      for(j=0; j < input_m[3].length(); j++) str4[j] = input_m[3][j]; 
      for(j=0; j < input_m[4].length(); j++) str5[j] = input_m[4][j]; 
      for(j=0; j < input_m[5].length(); j++) str6[j] = input_m[5][j]; 
      for(j=0; j < input_m[6].length(); j++) str7[j] = input_m[6][j]; 

      str2[input_m[1].length()] = '\0';
      str3[input_m[2].length()] = '\0';
      str4[input_m[3].length()] = '\0';
      str5[input_m[4].length()] = '\0';
      str6[input_m[5].length()] = '\0';
      str7[input_m[6].length()] = '\0';

      for (j = 1; j <= num_flied; j++){
        input_m[j] = "";
      }

      regulator_L.Kp = atof(str2);
      regulator_L.Ki = atof(str3);
      regulator_L.Kd = atof(str4);

      regulator_R.Kp = atof(str5);
      regulator_R.Ki = atof(str6);
      regulator_R.Kd = atof(str7);
    }

    else if (msg_type == 4){
      // Установка новых PID-коэффициентов и запись их во Flash-память (постоянная память)

      unsigned short num_flied = 6;

      for (j = 1; j <= num_flied; j++){
        for (i = i+1; i < mainInputString.length()-1; i++){
          if (mainInputString[i] == ';') break;
          input_m[j] += mainInputString[i];
        }
      }

      char str2[input_m[1].length() + 1]; 
      char str3[input_m[2].length() + 1];
      char str4[input_m[3].length() + 1]; 
      char str5[input_m[4].length() + 1]; 
      char str6[input_m[5].length() + 1]; 
      char str7[input_m[6].length() + 1]; 

      for(j=0; j < input_m[1].length(); j++) str2[j] = input_m[1][j];
      for(j=0; j < input_m[2].length(); j++) str3[j] = input_m[2][j]; 
      for(j=0; j < input_m[3].length(); j++) str4[j] = input_m[3][j]; 
      for(j=0; j < input_m[4].length(); j++) str5[j] = input_m[4][j]; 
      for(j=0; j < input_m[5].length(); j++) str6[j] = input_m[5][j]; 
      for(j=0; j < input_m[6].length(); j++) str7[j] = input_m[6][j]; 

      str2[input_m[1].length()] = '\0';
      str3[input_m[2].length()] = '\0';
      str4[input_m[3].length()] = '\0';
      str5[input_m[4].length()] = '\0';
      str6[input_m[5].length()] = '\0';
      str7[input_m[6].length()] = '\0';


      for (j = 1; j <= num_flied; j++){
        input_m[j] = "";
      }

      regulator_L.Kp = atof(str2);
      regulator_L.Ki = atof(str3);
      regulator_L.Kd = atof(str4);

      regulator_R.Kp = atof(str5);
      regulator_R.Ki = atof(str6);
      regulator_R.Kd = atof(str7);

      fleshMemory.begin("Memory1", RW_MODE);

      fleshMemory.putFloat("Kp_R", regulator_R.Kp); 
      fleshMemory.putFloat("Ki_R", regulator_R.Ki); 
      fleshMemory.putFloat("Kd_R", regulator_R.Kd); 

      fleshMemory.putFloat("Kp_L", regulator_L.Kp);
      fleshMemory.putFloat("Ki_L", regulator_L.Ki); 
      fleshMemory.putFloat("Kd_L", regulator_L.Kd); 

      fleshMemory.end();
    }
        
    else if (msg_type == 5){
      regulatorsCoefficientsPublish(
        regulator_L.Kp,
        regulator_L.Ki,
        regulator_L.Kd,
        regulator_R.Kp,
        regulator_R.Ki,
        regulator_R.Kd
      );
    }
    


    // else{
    //   raise_error();
    // }

    mainInputString = "";
    mainStringComplete = false;

  }
  
  // ===================== ЧТЕНИЕ ДАННЫХ С JETSON NANO (Serial1) =====================
  read_serial1();

  if (addictionStringComplete) {
    parseJetsonMessage(addictionInputString);
    addictionInputString = "";
    addictionStringComplete = false;
  }
  // =========================================================================


  // ===================== ОБНОВЛЕНИЕ ДИСПЛЕЯ =====================
  if (millis() - lastDisplayUpdate > displayInterval) {
    lastDisplayUpdate = millis();
    print_telemetry();
  }
  // =========================================================================

}