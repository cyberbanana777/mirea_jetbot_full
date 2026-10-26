// СТАРАЯ схема (из присланного архива): M-метод 10 Гц + скользящее среднее на 5 + ПИД, мёртвая зона 0.35 жёстко.
// Собирается из orig/motor_controller.* без изменений.
#include "Arduino.h"
#include "sim_run.h"
#define private public
#include "motor_controller.h"
#undef private

struct OldChain {
  MotorController ctl;
  double periodUs = 100000;
  int32_t prevR = 0, prevL = 0;
  OldChain() : ctl(0.1775f, 1.0f, 100.0f, 0.8f, 1.5f, 0.04f, 0.85f, 1.5f, 0.05f) { ctl.begin(); }
  void command(double ms) { ctl.setWheelSpeeds((float)ms, (float)ms); }
  void tick(uint32_t, const EdgeSim &eL, const EdgeSim &eR) {
    const int32_t tr = eR.edges - prevR, tl = eL.edges - prevL;
    prevR = eR.edges; prevL = eL.edges;
    const double revR = tr / 1080.0 / 0.1;
    const double revL = -(tl / 1080.0 / 0.1);   // как convert_ticks_to_freq(..., 'l')
    ctl.update((float)(revL * REV_TO_MS), (float)(revR * REV_TO_MS));
  }
  double dutyR() { return ((double)ctl._pwm.pins[2] - ctl._pwm.pins[3]) / 4095.0; }
  double dutyL() { return ((double)ctl._pwm.pins[1] - ctl._pwm.pins[0]) / 4095.0; }
  double i2cWrites() { return (double)ctl._pwm.writes; }
};

int main() {
  PlantParams ppR, ppL;
  ppL.Kv *= 0.92; ppL.dz += 0.02; ppL.dBreak += 0.02;
  printf("СТАРАЯ СХЕМА (та же модель двигателя)\n\n");
  printHeader();
  const double targets[] = {0.05, 0.5, 3.5};
  for (double r : targets) {
    const double T = (r < 0.1) ? 5.0 : 3.0;
    OldChain ch;
    RunResult res = runStep(ch, r, T, 0.0, ppR, ppL);
    printRow("M-метод 10 Гц + MA5 + ПИД", r, res.right);
  }
  return 0;
}
