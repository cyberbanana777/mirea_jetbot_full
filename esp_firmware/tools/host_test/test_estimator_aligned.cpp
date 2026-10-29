// Окно из целого числа меток (MTSpeedEstimator::updateAligned) против прежней оценки на энкодере
// с неравномерной квадратурой. asym = 0.5: один из фронтов сдвинут на ~0.5 интервала, как
// показал опыт noisetest на роботе (иглы 0.4-0.55 фронта на окно).
#include "sim_common.h"
#include "speed_estimator.h"
#include <stdio.h>
#include <vector>

static int failures = 0;
static void check(bool ok, const char *what) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what); if (!ok) ++failures; }

struct Res { double rms, mx, lagMs; std::vector<double> t, est, truth; };

template <class F>
static Res run(F wOf, double T, bool aligned, double asym, int minEdges = 2, uint32_t tOff = 0, uint32_t posOff = 0,
               double t0 = 0.5, double t1 = 1e9) {
  MTSpeedParams p = {1.0f / 1080.0f, 60000, minEdges, 6.0f, 1.0f, aligned};
  MTSpeedEstimator est(p);
  EdgeSim e; e.asym = asym;
  double th = 0; Res r{}; double s2 = 0; int n = 0;
  const double dt = 10e-6; double next = 10000;
  for (double tt = 0; tt < T; tt += dt) {
    const double w = wOf(tt), th1 = th + w * dt;
    e.advance(th, th1, tt * 1e6, dt * 1e6); th = th1;
    if (tt * 1e6 >= next) {
      next += 10000;
      uint32_t ph[4]; for (int i = 0; i < 4; ++i) ph[i] = e.ph[i] + tOff;
      const uint32_t now = (uint32_t)((uint64_t)(tt * 1e6) + tOff);
      const float v = est.updateAligned((int32_t)((uint32_t)e.edges + posOff), ph, e.dir, now);
      r.t.push_back(tt); r.est.push_back(v); r.truth.push_back(w);
      if (tt >= t0 && tt <= t1 && fabs(w) > 1e-6) { const double rel = (v - w) / w; s2 += rel * rel; r.mx = std::max(r.mx, fabs(rel)); ++n; }
    }
  }
  r.rms = n ? sqrt(s2 / n) : 0; return r;
}

int main() {
  printf("1) Постоянная скорость, квадратура с перекосом 0.5 интервала: прежняя оценка против целых меток\n");
  printf("   %8s %14s %14s %14s %14s\n", "об/с", "прежняя СКО,%", "прежняя max,%", "метки СКО,%", "метки max,%");
  bool allGood = true, allBetter = true;
  for (double w : {0.2, 0.37, 0.7, 0.91, 1.57, 2.6, 3.5}) {
    Res a = run([&](double) { return w; }, 3.0, false, 0.5);
    Res b = run([&](double) { return w; }, 3.0, true, 0.5);
    printf("   %8.2f %14.2f %14.2f %14.3f %14.3f\n", w, a.rms * 100, a.mx * 100, b.rms * 100, b.mx * 100);
    if (b.mx > 0.01) allGood = false;
    if (w >= 0.37 && b.rms > 0.25 * a.rms) allBetter = false;
  }
  check(allGood, "с целыми метками ошибка < 1 % на всех скоростях");
  check(allBetter, "с 0.37 об/с и выше СКО меньше прежней хотя бы вчетверо");

  printf("2) Идеальная квадратура: целые метки не хуже прежнего\n");
  {
    Res b = run([](double) { return 1.0; }, 2.0, true, 0.0);
    check(b.mx < 0.002, "ошибка < 0.2 % на 1 об/с");
  }
  printf("3) Малая скорость 0.05 об/с и запаздывание\n");
  {
    Res b = run([](double) { return 0.05; }, 6.0, true, 0.5, 2, 0, 0, 2.0);
    printf("   СКО %.2f %%, max %.2f %%\n", b.rms * 100, b.mx * 100);
    check(b.mx < 0.02, "0.05 об/с: ошибка < 2 %");
  }
  printf("4) Разгон 0 -> 2 об/с за 0.5 с, торможение до нуля и реверс\n");
  {
    auto prof = [](double t) { return t < 0.2 ? 0.0 : t < 0.7 ? 4.0 * (t - 0.2) : t < 1.5 ? 2.0 : t < 2.0 ? 2.0 - 4.0 * (t - 1.5) : t < 2.5 ? -4.0 * (t - 2.0) : -2.0; };
    Res b = run(prof, 3.5, true, 0.5, 2, 0, 0, 0.9, 1.4);
    check(b.mx < 0.01, "на постоянных 2 об/с после разгона ошибка < 1 %");
    double atStop = 1, atRev = 0;
    for (size_t i = 0; i < b.t.size(); ++i) {
      if (fabs(b.t[i] - 2.0) < 0.006) atStop = b.est[i];
      if (b.t[i] > 3.2 && b.t[i] < 3.5) atRev = std::min(atRev, b.est[i]);
    }
    printf("   оценка около остановки (истина 0): %.3f об/с, после реверса: %.3f об/с\n", atStop, atRev);
    check(fabs(atStop) < 0.15, "у нуля скорость не залипает");
    Res r = run(prof, 3.5, true, 0.5, 2, 0, 0, 3.0, 3.5);
    check(r.mx < 0.01, "после реверса на -2 об/с ошибка < 1 %");
  }
  printf("5) Переполнение счётчиков времени и позиции\n");
  {
    Res b = run([](double) { return 1.3; }, 2.0, true, 0.5, 2, 0xFFFFFFFFu - 600000u, 0x7FFFFFF0u);
    check(b.mx < 0.01, "ошибка < 1 % при переполнении micros() и позиции");
  }
  printf("6) Торможение до остановки: оценка спадает к нулю\n");
  {
    auto prof = [](double t) { return t < 1.0 ? 1.0 : std::max(0.0, 1.0 - 2.0 * (t - 1.0)); };
    Res b = run(prof, 2.5, true, 0.5);
    check(fabs(b.est.back()) < 1e-6, "через 1 с после остановки оценка 0");
  }
  printf("\n%s (%d не прошло)\n", failures ? "ЕСТЬ ОШИБКИ" : "ВСЕ ПРОВЕРКИ ПРОШЛИ", failures);
  return failures ? 1 : 0;
}
