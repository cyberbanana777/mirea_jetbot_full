
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