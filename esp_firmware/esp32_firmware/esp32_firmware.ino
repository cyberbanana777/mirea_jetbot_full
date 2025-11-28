// Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

// Библиотеки регуляции и управления
#include "GyverPID.h"
#include <Wire.h>                     
#include <Adafruit_PWMServoDriver.h> 
#include <stdio.h>

// Настройки для записи в постоянную память
#define RW_MODE false
#define RO_MODE true
Preferences fleshMemory;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool input_open = false;

//Константы физических параметров
const float Pi = 3.14159;
const float l = 0.117;
const float r = 0.065/2;

float max_frequency = 2.5;
float max_vel = max_frequency * 2 * Pi * r;

#define LED_PIN 13 // Оставлен для debug

// Пины энкодеров
#define PIN_R_A 34  // Пин для сигнала 1
#define PIN_R_B 35  // Пин для сигнала 2
#define PIN_L_A 33  // Пин для сигнала 3
#define PIN_L_B 32  // Пин для сигнала 4

// Частота дискретизации для ПИДов
float dt = 30;

// Правый ПИД
float Kp_R = 2.075;//2.075;
float Ki_R = 0.0;
float Kd_R = 0.005;//0.005;
GyverPID regulator_R(Kp_R, Ki_R, Kd_R, dt);

// Левый ПИД
float Kp_L = 2.075;
float Ki_L = 0.0;
float Kd_L = 0.005; 
GyverPID regulator_L(Kp_L, Ki_L, Kd_L, dt);


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);

#define MOTOR_L 0x00
#define MOTOR_R 0x02

// Для энкодеров
long int global_pos_R = 0;
long int global_pos_L = 0;

// Для таймеров
unsigned long tmr = 0;
unsigned long delta = 0;
const unsigned long timer_timeout = dt * 1000; //мкс

// Для остановки после 5 секунд простоя
const unsigned int im_timer_timeout = 5000;
unsigned int im_timer = 0;


// Переменные скорости правые
float RealFrequencyRight = 0.0;
float TargetRight = 0.0;

// Переменные скорости левые
float RealFrequencyLeft = 0.0;
float TargetLeft = 0.0;

double max_contructive_velocity = 3.2;

double x_pos_ = 0.0;
double y_pos_ = 0.0;
double heading_ = 0.0;
String input_m[8] = {"", "", "", "", "", "", "", ""};
String feedback_msg_str = "";


// Для прерываний (машины состояний)
bool ReadPIN_R_A_A;
bool ReadPIN_R_B_A;
bool ReadPIN_R_A_B;
bool ReadPIN_R_B_B;
bool ReadPIN_L_A_A;
bool ReadPIN_L_B_A;
bool ReadPIN_L_A_B;
bool ReadPIN_L_B_B;



// Функции на прерывания для считывания изменения меток с каждого канала
// Функции идентичны, только для разных каналов
IRAM_ATTR void Read_R_A(){
  ReadPIN_R_A_A = digitalRead(PIN_R_A);
  ReadPIN_R_B_A = digitalRead(PIN_R_B);

  switch (ReadPIN_R_A_A) {
    case 0:
      if (ReadPIN_R_B_A == 1) {global_pos_R++; break;}
      if (ReadPIN_R_B_A == 0) {global_pos_R--; break;}
      break;
    
    case 1:
      if (ReadPIN_R_B_A == 1) {global_pos_R--; break;}
      if (ReadPIN_R_B_A == 0) {global_pos_R++; break;}
      break;
  }
}

IRAM_ATTR void Read_R_B(){
  ReadPIN_R_A_B = digitalRead(PIN_R_A);
  ReadPIN_R_B_B = digitalRead(PIN_R_B);

  switch (ReadPIN_R_B_B) {
    case 0:
      if (ReadPIN_R_A_B == 1) {global_pos_R--; break;}
      if (ReadPIN_R_A_B == 0) {global_pos_R++; break;}
      break;
    
    case 1:
      if (ReadPIN_R_A_B == 1) {global_pos_R++; break;}
      if (ReadPIN_R_A_B == 0) {global_pos_R--; break;}
      break;
  }
}

IRAM_ATTR void Read_L_A(){
  ReadPIN_L_A_A = digitalRead(PIN_L_A);
  ReadPIN_L_B_A = digitalRead(PIN_L_B);

  switch (ReadPIN_L_A_A) {
    case 0:
      if (ReadPIN_L_B_A == 1) {global_pos_L++; break;}
      if (ReadPIN_L_B_A == 0) {global_pos_L--; break;}
      break;
    
    case 1:
      if (ReadPIN_L_B_A == 1) {global_pos_L--; break;}
      if (ReadPIN_L_B_A == 0) {global_pos_L++; break;}
      break;
  }
}

IRAM_ATTR void Read_L_B(){
  ReadPIN_L_A_B = digitalRead(PIN_L_A);
  ReadPIN_L_B_B = digitalRead(PIN_L_B);

  switch (ReadPIN_L_B_B) {
    case 0:
      if (ReadPIN_L_A_B == 1) {global_pos_L--; break;}
      if (ReadPIN_L_A_B == 0) {global_pos_L++; break;}
      break;
    
    case 1:
      if (ReadPIN_L_A_B == 1) {global_pos_L++; break;}
      if (ReadPIN_L_A_B == 0) {global_pos_L--; break;}
      break;
  }
}

// Управление моторами с помощью ШИМ
void motorWrite(int CH, float set_speed) {

  int16_t PWM = 0;
  set_speed = set_speed;
  if (CH == 0x00) set_speed *= -1;  // Инвертирование левого двигателя
  if (set_speed > 0)                // Вращение вперед
  {
    PWM = set_speed * 4096;
    pwm.setPin(CH + 1, 0, false);  // Переключение направления вращения (если вращался в эту сторону)
    pwm.setPin(CH + 0, PWM, false);
  } else                            // Вращение назад
  {
    set_speed *= -1;
    PWM = set_speed * 4096;
    pwm.setPin(CH + 0, 0, false);  // Переключенеи направления вращения (если вращался в эту сторону) CH+0 и CH+1 нужны для выбора канала
    pwm.setPin(CH + 1, PWM, false);
  }
}

void odomPublish (double x_pos_, double y_pos_, double heading_, double linear_vel_x, double angular_vel_z, double left_wheel_velocity, double right_wheel_velocity) {
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

void regulatorsCoefficientsPublish (double Kp_L, double Ki_L, double Kd_L, double Kp_R, double Ki_R, double Kd_R) {
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

//int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    float temp_value;
    if (value < fromLow) {temp_value = fromLow;}
    else if (value > fromHigh) {temp_value = fromHigh;}
    else {
      temp_value = value;
    }
    return (temp_value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void speed_converter(double xl, double zw) {
  TargetLeft = xl - zw * l / 2;
  TargetRight = xl + zw * l / 2;
  TargetLeft = mapFloat(TargetLeft, -max_vel*0.8, max_vel*0.8, -0.8, 0.8);
  TargetRight = mapFloat(TargetRight, -max_vel*0.8, max_vel*0.8, -0.8, 0.8);
}

void cut_speeds(float V_l, float V_r){
  TargetLeft = mapFloat(V_l, -max_vel*0.8, max_vel*0.8, -0.8, 0.8);
  TargetRight = mapFloat(V_r, -max_vel*0.8, max_vel*0.8, -0.8, 0.8);
}

//Ответ на входное сообщение
void serialEvent() {
  im_timer = millis();
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if(inChar == '$' or input_open == true){
      // add it to the inputString:
      inputString += inChar;
      input_open = true;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '#') {
      stringComplete = true;
      input_open = false;
    }
  }
}

void raise_error(){
  while(true){
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

void setup() {
  // Объявление прерываний
  attachInterrupt(PIN_R_A, Read_R_A, CHANGE);
  attachInterrupt(PIN_R_B, Read_R_B, CHANGE);
  attachInterrupt(PIN_L_A, Read_L_A, CHANGE);
  attachInterrupt(PIN_L_B, Read_L_B, CHANGE);

  // Светодиод ошибки
  pinMode(LED_PIN, INPUT); // Оставлен для debug

  // Пины энкодеров
  pinMode(PIN_R_A, INPUT);
  pinMode(PIN_R_B, INPUT);
  pinMode(PIN_L_A, INPUT);
  pinMode(PIN_L_B, INPUT);

  // Подключение ШИМ-модуляции
  delay(10);
  pwm.begin();
  pwm.setPWMFreq(1000);
  delay(10);
  
  // Настройка ПИДов: установка пределов
  regulator_R.setLimits(-1.0, 1.0);
  regulator_L.setLimits(-1.0, 1.0);

  Serial.begin(115200);
  inputString.reserve(250);
  
  delay(100);

  fleshMemory.begin("Memory1", RO_MODE);
  bool tpInit = fleshMemory.isKey("nvsInit");

  if (tpInit == false) {

    Serial.println("First time run the program");

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
  
  else {

    regulator_R.Kp = fleshMemory.getFloat("Kp_R"); 
    regulator_R.Ki = fleshMemory.getFloat("Ki_R"); 
    regulator_R.Kd = fleshMemory.getFloat("Kd_R"); 

    regulator_L.Kp = fleshMemory.getFloat("Kp_L"); 
    regulator_L.Ki = fleshMemory.getFloat("Ki_L"); 
    regulator_L.Kd = fleshMemory.getFloat("Kd_L"); 

    fleshMemory.end();
  }
  
  motorWrite(MOTOR_R, 0.0);
  motorWrite(MOTOR_L, 0.0);
}


void loop() {

  if (millis() - im_timer > im_timer_timeout){
    // Аварийная остановка
    TargetRight = 0;
    TargetLeft = 0;
  }

  delta = micros() - tmr;
  
  if (delta >= timer_timeout){  
    tmr = micros();

    // Настройка ПИДов: Установка целевых значений
    regulator_R.setpoint = TargetRight;  
    regulator_L.setpoint = TargetLeft; 
    
    //Подсчёт скорости
    RealFrequencyRight = (((float)global_pos_R*1000000)/(270*4*timer_timeout));
    RealFrequencyLeft = (((float)global_pos_L*1000000)/(270*4*timer_timeout));
    global_pos_R = 0;
    global_pos_L = 0;
    
    double vel_dt = timer_timeout/1000; // ms
    double linear_vel_x = (RealFrequencyRight + RealFrequencyLeft)*2*Pi*r/2;
    double angular_vel_z = (RealFrequencyRight - RealFrequencyLeft)*2*Pi*r/l;
    double left_wheel_velocity = RealFrequencyLeft * 2 * Pi * r;
    double right_wheel_velocity = RealFrequencyRight  * 2 * Pi * r;
    double delta_heading = angular_vel_z * vel_dt/1000; //radians
    double cos_h = cos(heading_);
    double sin_h = sin(heading_);
    double delta_x = (linear_vel_x * cos_h) * vel_dt/1000; //m
    double delta_y = (linear_vel_x * sin_h) * vel_dt/1000; //m

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;
    odomPublish(x_pos_, y_pos_, heading_, linear_vel_x, angular_vel_z, left_wheel_velocity, right_wheel_velocity);

    // Отправка в ПИДы расчитанного значения скорости 
    regulator_R.input = RealFrequencyRight / max_contructive_velocity;
    regulator_L.input = RealFrequencyLeft / max_contructive_velocity;

    // Подача на моторы "исправленного" сигнала  
    motorWrite(MOTOR_R, regulator_R.getResult());
    motorWrite(MOTOR_L, regulator_L.getResult());
  }

    //Обработка входного значения
  if (stringComplete) {

    unsigned int i = 0;
    unsigned int j = 0;

    // msg_type
    for (i = 1; i < inputString.length()-1; i++){
      if (inputString[i] == ';') break;
      input_m[0] += inputString[i];
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
        for (i = i+1; i < inputString.length()-1; i++){
          if (inputString[i] == ';') break;
          input_m[j] += inputString[i];
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
        for (i = i+1; i < inputString.length()-1; i++){
          if (inputString[i] == ';') break;
          input_m[j] += inputString[i];
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
        for (i = i+1; i < inputString.length()-1; i++){
          if (inputString[i] == ';') break;
          input_m[j] += inputString[i];
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
        for (i = i+1; i < inputString.length()-1; i++){
          if (inputString[i] == ';') break;
          input_m[j] += inputString[i];
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

    inputString = "";
    stringComplete = false;

  }

}