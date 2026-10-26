// Подбор коэффициентов ПИ по умолчанию: перебор по моделям двигателя (Tm, усиление) и
// по FF "не калиброван" / "откалиброван". Печатает худший случай по всем моделям.
#include "new_chain.h"
#include <map>

int main(int argc, char **argv) {
  MTSpeedParams sp = {1.0f / 1080.0f, 60000, 2, 6.0f, 1.0f};
  struct Gains { float kp, ki, kd; } gs[] = {{0.8f,1.5f,0.04f},{0.8f,4,0},{1.0f,6,0},{1.0f,10,0},{1.5f,10,0},{1.0f,15,0},{2.0f,20,0}};
  const double Tms[] = {0.04, 0.08, 0.16};
  const double Kvs[] = {0.7, 1.0, 1.4};
  const double targets[] = {0.05, 0.5, 3.5};
  const bool calibrated = argc > 1;
  printf("FF: %s\n", calibrated ? "откалиброван (точно по модели)" : "по умолчанию dz=0.35 gain=0.5 (истина dz=0.33, gain~0.78/Kv)");
  printf("Худший случай по 9 моделям двигателя (Tm 40/80/160 мс x усиление 0.7/1/1.4): перегул.%%, установл.±10%% (мс), рябь%%, не установилась\n");
  for (auto &g : gs) {
    printf("Kp=%.1f Ki=%4.1f Kd=%.2f\n", g.kp, g.ki, g.kd);
    for (double r : targets) {
      double worstOs = 0, worstSettle = 0, worstRip = 0; int notReached = 0, n = 0;
      for (double Tm : Tms) for (double kvs : Kvs) {
        PlantParams pp; pp.Tm = Tm; pp.Kv *= kvs;
        float gain = calibrated ? (float)((1.0 / pp.Kv) / REV_TO_MS) : 0.5f;
        float dz = calibrated ? (float)pp.dz : 0.35f;
        NewChain ch(sp, g.kp, g.ki, g.kd, gain, dz);
        RunResult res = runStep(ch, r, (r < 0.1) ? 6.0 : 4.0, 0.0, pp, pp);
        const StepMetrics &m = res.right;
        ++n;
        if (m.settle10 < 0) ++notReached; else worstSettle = std::max(worstSettle, m.settle10);
        worstOs = std::max(worstOs, m.overshootPct);
        worstRip = std::max(worstRip, m.rippleRelPct);
      }
      printf("   %5.2f об/с: %6.1f%% %6.0f мс %5.1f%%  %d/%d\n", r, worstOs, worstSettle * 1000, worstRip, notReached, n);
    }
  }
  return 0;
}
