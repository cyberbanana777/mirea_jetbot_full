

void init_motor_control(){
  // Подключение ШИМ-модуляции
  delay(10);
  pwm.begin();
  pwm.setPWMFreq(1000);
  delay(10);
  
  // Настройка ПИДов: установка пределов
  regulator_R.setLimits(-1.0, 1.0);
  regulator_L.setLimits(-1.0, 1.0);
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