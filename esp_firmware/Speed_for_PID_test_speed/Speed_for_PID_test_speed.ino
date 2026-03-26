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
      Serial.println( "Forward ");
      A_1 = A_2;
    }
    if (B_2 == 1){
      Serial.println( "Back");
      B_1 = B_2;
    }
  }
}
