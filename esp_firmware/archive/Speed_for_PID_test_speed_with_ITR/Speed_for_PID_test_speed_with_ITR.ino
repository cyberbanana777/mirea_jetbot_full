// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

/*
АННОТАЦИЯ

Скетч для ESP32 реализует полноценный квадратурный декодер для двух независимых
инкрементальных энкодеров (правый — пины 34,35; левый — пины 32,33). Внутри
обработчиков прерываний CHANGE на каждом из четырёх каналов скрыт конечный
автомат, отслеживающий все возможные переходы сигналов A и B. В зависимости от
текущего состояния каналов и их предыдущих значений (хранятся в глобальных
флагах вида ReadPIN_*_A/B) счётчик позиции global_pos_R или global_pos_L
инкрементируется или декрементируется. В отличие от упрощённого детектора
фронтов, этот автомат устойчив к дребезгу и корректно обрабатывает любое
направление вращения энкодера. В цикле loop() значения позиций выводятся в Serial.

Подробнее про обработку значений с двухканального энкодера и машину состояний
можно прочитать здесь: https://alexgyver.ru/lessons/encoder/

*/

// Пины энкодеров
#define PIN_R_A 34  // Пин для сигнала 1
#define PIN_R_B 35  // Пин для сигнала 2
#define PIN_L_A 32  // Пин для сигнала 3
#define PIN_L_B 33  // Пин для сигнала 4

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



// Для прерываний (машины состояний)
bool ReadPIN_R_A_A;
bool ReadPIN_R_B_A;
bool ReadPIN_R_A_B;
bool ReadPIN_R_B_B;
bool ReadPIN_L_A_A;
bool ReadPIN_L_B_A;
bool ReadPIN_L_A_B;
bool ReadPIN_L_B_B;


int_least64_t global_pos_R = 0;
int_least64_t global_pos_L = 0;

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

void setup(){
  init_interraptions();
  Serial.begin(115200);
  Serial.print("hello from debug firmware!");
}

void loop(){
  Serial.print(global_pos_R);
  Serial.print(" ");
  Serial.println(global_pos_L);
}

