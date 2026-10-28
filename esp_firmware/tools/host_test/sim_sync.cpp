// Синхронность колёс: ПИ с разной / одинаковой желаемой динамикой и фильтр уставки.
// Модели: "воздух" - по калибровке mtcal; "пол" - мёртвая зона и трение покоя выше (по логам steptest),
// feedforward и ПИ те же (калибровка в воздухе). Tm на полу - допущение.
#include "new_chain.h"

struct Cfg { const char *name; float kpL, kiL, kpR, kiR; float tau; };

static void runScenario(const char *title, PlantParams L, PlantParams R, float ffGL, float ffDL, float ffGR, float ffDR) {
  printf("\n=== %s ===\n", title);
  const Cfg cfgs[] = {
    {"A: ПИ lam 1.0 (как сейчас), без фильтра", 0.524f, 3.02f, 0.801f, 5.62f, 0.0f},
    {"B: ПИ общий lambda 0.2 с, без фильтра  ", 0.463f, 2.66f, 0.594f, 4.153f, 0.0f},
    {"C: B + фильтр уставки tau 0.15 с        ", 0.463f, 2.66f, 0.594f, 4.153f, 0.15f},
    {"D: B + фильтр уставки tau 0.25 с        ", 0.463f, 2.66f, 0.594f, 4.153f, 0.25f},
    {"E: A + фильтр уставки tau 0.15 с        ", 0.524f, 3.02f, 0.801f, 5.62f, 0.15f},
  };
  const double targets[] = {0.5, 1.2};
  MTSpeedParams sp = {1.0f / 1080.0f, 60000, 2, 6.0f, 1.0f};
  for (double r : targets) {
    printf("\nуставка %.1f об/с\n%-42s %8s %8s %8s %8s %9s %9s\n", r, "вариант", "t90 L", "t90 R", "пер.L,%", "пер.R,%", "max|dW|", "курс,гр");
    for (const Cfg &c : cfgs) {
      NewChain ch(sp, c.kpL, c.kiL, 0.0f, ffGL, ffDL);
      ch.ctl.setPID(c.kpL, c.kiL, 0, c.kpR, c.kiR, 0);
      ch.ctl.setFeedforward(ffGL, ffDL, ffGR, ffDR);
      ch.ctl.setRefFilter(c.tau);
      RunResult res = runStep(ch, r, 3.0, 0.0, R, L);
      printf("%-42s %8.0f %8.0f %8.1f %8.1f %9.3f %9.2f\n", c.name, res.left.rise90 * 1000, res.right.rise90 * 1000,
             res.left.overshootPct, res.right.overshootPct, res.maxDiffRev, res.peakYawDeg);
    }
  }
}

int main() {
  PlantParams L, R;
  L.Tm = 0.174; L.Kv = 8.37; L.dz = 0.107; L.dBreak = 0.18;
  R.Tm = 0.143; R.Kv = 5.36; R.dz = 0.111; R.dBreak = 0.20;
  runScenario("ВОЗДУХ (feedforward калиброван в воздухе)", L, R, 0.5848f, 0.1073f, 0.9133f, 0.1114f);
  PlantParams L2 = L, R2 = R;
  L2.Tm = 0.25; L2.dz = 0.30; L2.dBreak = 0.42;
  R2.Tm = 0.20; R2.dz = 0.31; R2.dBreak = 0.49;
  runScenario("ПОЛ (feedforward по полу: 0.5151/0.2992/0.8424/0.3101)", L2, R2, 0.5151f, 0.2992f, 0.8424f, 0.3101f);
  return 0;
}
