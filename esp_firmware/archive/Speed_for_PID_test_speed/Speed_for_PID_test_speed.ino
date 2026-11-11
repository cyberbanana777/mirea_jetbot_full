// Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

/*
АННОТАЦИЯ

Отладочный скетч для ESP32 реализует простейший конечный автомат обработки
сигналов с двухканального энкодера (пины 34, 35). Внутренние флаги A_1 и B_1
хранят предыдущее активное состояние движения. Логика:
  - Если оба канала в 0 — автомат сбрасывает запомненное направление (A_1=0,
  но в коде также обнуляется A_2 — вероятно, опечатка, предполагавшая сброс B_1).
  - Если ровно на одном канале 1 (состояние (1,0) или (0,1)), автомат выводит
  "Forward" или "Back" и фиксирует это направление в A_1 или B_1.
Таким образом, автомат реагирует на каждый импульс энкодера, но не обрабатывает
полную квадратурную таблицу переходов, работая как детектор фронтов с памятью
последнего направления.

ПРИМЕЧАНИЕ
Корректно работает на малых скоростях вращения вала мотора. При "больших"
скоростях логика, заложенная в работу программы нарушается.

Подробнее про обработку значений с двухканального энкодера и машину состояний
можно прочитать здесь: https://alexgyver.ru/lessons/encoder/

*/

#define pinA 34
#define pinB 35
bool A_1;
bool A_2;
bool B_1;
bool B_2;

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  A_1 = digitalRead(pinA);
  B_1 = digitalRead(pinB);
  Serial.begin(115200);
  Serial.print("hello from debug firmware!");
}

void loop() {
  A_2 = digitalRead(pinA);
  B_2 = digitalRead(pinB);
  
  if ((A_2 == 0) and (B_2 == 0)){
    A_1 = 0;
    A_2 = 0;
  }
  
  if (((A_2 == 1 ) || (B_2 == 1)) & (A_2 != B_2)){
    if (A_2 == 1){
      Serial.println("Forward");
      A_1 = A_2;
    }
    if (B_2 == 1){
      Serial.println("Back");
      B_1 = B_2;
    }
  }
}
