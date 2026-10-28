// Модель по результатам калибровки реального робота (mtcal) и сравнение с steptest на железе.
#include "new_chain.h"

#ifndef KPL
#define KPL 0.362f
#define KIL 2.085f
#define KPR 0.557f
#define KIR 3.907f
#endif
int main() {
  PlantParams L, R;
  L.Tm = 0.174; L.Kv = 8.37; L.dz = 0.107; L.dBreak = 0.18;
  R.Tm = 0.143; R.Kv = 5.36; R.dz = 0.111; R.dBreak = 0.20;
  MTSpeedParams sp = {1.0f / 1080.0f, 60000, 2, 6.0f, 1.0f};
  printf("Модель реального робота (L: Tm 174 мс, R: Tm 143 мс). ПИ: L 0.362/2.085, R 0.557/3.907\n");
  printf("На железе (steptest): 0.5 об/с: перерег. L 31%% R 26%%, t90 440/310 мс; 1.5 об/с: 23%% / 20%%, t90 340/230 мс\n\n");
  printHeader();
  const double targets[] = {0.05, 0.5, 1.5};
  for (double r : targets) {
    NewChain ch(sp, 0.362f, 2.085f, 0.0f, 0.5848f, 0.1073f);
    ch.ctl.setPID(KPL, KIL, 0, KPR, KIR, 0);
    ch.ctl.setFeedforward(0.5848f, 0.1073f, 0.9133f, 0.1114f);
    RunResult res = runStep(ch, r, 2.5, 0.0, R, L);
    printRow("L", r, res.left);
    printRow("R", r, res.right);
  }
  return 0;
}
