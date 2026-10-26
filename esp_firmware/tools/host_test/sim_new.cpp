// Новая схема: M/T-оценка + контур 100 Гц + feedforward + ПИ (код прошивки без изменений).
#include "new_chain.h"

int main(int argc, char **argv) {
  const bool csv = argc > 1;  // ./sim_new csv - выгрузка кривой для графика
  PlantParams ppR, ppL;
  ppL.Kv *= 0.92; ppL.dz += 0.02; ppL.dBreak += 0.02;   // левое колесо чуть другое
  const double kvRev = 1.0 / ppR.Kv;                    // скважность на 1 об/с
  const float ffGainTrue = (float)(kvRev / REV_TO_MS);  // скважность на 1 м/с (идеальная калибровка)
  const float ffGainDef = 0.6f;                         // значение по умолчанию из config.h

  MTSpeedParams sp = {1.0f / 1080.0f, 60000, 2, 6.0f, 1.0f};  // как в config.h: SPEED_MIN_EDGES = 2
  MTSpeedParams sp1 = sp; sp1.minEdges = 1;

  printf("НОВАЯ СХЕМА (модель: Tm=%.0f мс, мёртвая зона %.2f, трогание %.2f; PI Kp=1.0 Ki=6.0 как в config.h)\n\n",
         ppR.Tm * 1000, ppR.dz, ppR.dBreak);
  printHeader();
  const double targets[] = {0.05, 0.5, 3.5};
  for (double r : targets) {
    const double T = (r < 0.1) ? 5.0 : 3.0;
    struct V { const char *name; MTSpeedParams p; float gain, dz; double asym; } vs[] = {
      {"FF по умолчанию",          sp,  ffGainDef,  0.35f, 0.0},
      {"FF откалиброван",          sp,  ffGainTrue, 0.33f, 0.0},
      {"+квадр. 10%, мин. 1 фронт", sp1, ffGainTrue, 0.33f, 0.10},
      {"+квадр. 10%, мин. 2 фронта", sp, ffGainTrue, 0.33f, 0.10},
    };
    for (auto &v : vs) {
      NewChain ch(v.p, 1.0f, 6.0f, 0.0f, v.gain, v.dz);
      RunResult res = runStep(ch, r, T, v.asym, ppR, ppL);
      printRow(v.name, r, res.right);
      if (fabs(res.left.finalRel - 1.0) > 0.05 && res.left.finalRel > 0)
        printf("   (левое колесо: в конце %.0f%% от цели)\n", res.left.finalRel * 100);
      if (res.left.finalRel <= 0) printf("   !!! левое колесо не едет (знак?)\n");
    }
    printf("\n");
  }
  {
    NewChain ch(sp, 1.0f, 6.0f, 0.0f, ffGainTrue, 0.33f);
    RunResult res = runStep(ch, 0.5, 3.0, 0.0, ppR, ppL);
    printf("Записей в PCA9685 за секунду: %.0f (на 2 мотора, 100 тактов/с)\n", res.i2cWritesPerSec);
  }
  if (csv) {
    NewChain ch(sp, 1.0f, 6.0f, 0.0f, ffGainTrue, 0.33f);
    std::vector<double> t, w;
    runStep(ch, 0.5, 3.0, 0.0, ppR, ppL, &t, &w);
    FILE *f = fopen("trace_new.csv", "w");
    for (size_t i = 0; i < t.size(); ++i) fprintf(f, "%.4f,%.5f\n", t[i], w[i]);
    fclose(f);
  }
  return 0;
}
