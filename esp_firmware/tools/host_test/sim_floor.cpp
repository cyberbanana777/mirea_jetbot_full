// Модель "робот на полу": мёртвая зона и трение покоя заметно выше, чем в воздухе (по step_log с пола),
// а feedforward остаётся калиброванным "в воздухе".
#include "new_chain.h"
int main() {
  PlantParams L, R;
  L.Tm = 0.25; L.Kv = 8.37; L.dz = 0.30; L.dBreak = 0.42;
  R.Tm = 0.20; R.Kv = 5.36; R.dz = 0.31; R.dBreak = 0.49;
  MTSpeedParams sp = {1.0f / 1080.0f, 60000, 2, 6.0f, 1.0f};
  printf("Модель пола по логам: L dz 0.30 трогание 0.42, R dz 0.31 трогание 0.49; FF откалиброван на полу\n");
  printHeader();
  const double targets[] = {0.05, 0.1, 0.15, 0.2, 0.5, 1.0};
  for (double r : targets) {
    NewChain ch(sp, 0.951f, 5.475f, 0.0f, 0.5151f, 0.2992f);
    ch.ctl.setPID(0.951f, 5.475f, 0, 1.426f, 10.009f, 0);
    ch.ctl.setFeedforward(0.5151f, 0.2992f, 0.8424f, 0.3101f);
    RunResult res = runStep(ch, r, 4.0, 0.0, R, L);
    printRow("L", r, res.left);
    printRow("R", r, res.right);
  }
  return 0;
}
