// Тест оценки скорости (speed_estimator.h) на смоделированном энкодере.
// Печатает таблицу и PASS/FAIL; код возврата != 0, если что-то не прошло.
#include "sim_common.h"
#include "speed_estimator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

static int failures = 0;
static void check(bool ok, const char *what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) ++failures;
}

static MTSpeedParams P(int minEdges = 1, float k = 1.0f) { return {1.0f / 1080.0f, 60000, minEdges, 6.0f, k}; }

// Движение по закону w(t) (об/с); тики контура каждые 10 мс; результат - оценка в моменты тиков.
// tOff, posOff - смещения, чтобы проверить переполнение 32-битных счётчиков.
struct Trace { std::vector<double> t, est, truth; };

template <class F>
Trace run(F wOf, double T, const MTSpeedParams &p, double asym, uint32_t tOff = 0, uint32_t posOff = 0, double theta0 = 0.0) {
  MTSpeedEstimator est(p);
  EdgeSim e; e.asym = asym;
  double theta = theta0;
  e.edges = (int32_t)floor(theta0 * 1080.0);
  Trace tr;
  const double dt = 20e-6;
  double nextTick = 10000.0;
  for (double tt = 0; tt < T; tt += dt) {
    const double w = wOf(tt);
    const double th1 = theta + w * dt;
    e.advance(theta, th1, tt * 1e6, dt * 1e6);
    theta = th1;
    if (tt * 1e6 >= nextTick) {
      const uint32_t now = (uint32_t)((uint64_t)(tt * 1e6) + tOff);
      const uint32_t last = e.tLast + tOff;
      const int32_t pos = (int32_t)((uint32_t)e.edges + posOff);
      tr.t.push_back(tt);
      tr.est.push_back(est.update(pos, last, now));
      tr.truth.push_back(w);
      nextTick += 10000.0;
    }
  }
  return tr;
}

struct Stat { double meanErr, maxErr, rms; };
static Stat stat(const Trace &tr, double t0, double t1) {
  double s = 0, mx = 0, s2 = 0; int n = 0;
  for (size_t i = 0; i < tr.t.size(); ++i) {
    if (tr.t[i] < t0 || tr.t[i] > t1) continue;
    const double rel = (tr.est[i] - tr.truth[i]) / tr.truth[i];
    s += rel; s2 += rel * rel; mx = std::max(mx, fabs(rel)); ++n;
  }
  return {s / n, mx, sqrt(s2 / n)};
}

int main() {
  printf("1) Постоянная скорость, идеальная квадратура (ошибка оценки в %% от истины)\n");
  printf("   %-10s %10s %10s\n", "об/с", "среднее", "максимум");
  const double speeds[] = {0.05, 0.1, 0.5, 3.5, -0.5};
  for (double w : speeds) {
    Trace tr = run([w](double) { return w; }, 2.0, P(), 0.0);
    Stat s = stat(tr, 0.3, 2.0);
    printf("   %-10.2f %9.3f%% %9.3f%%\n", w, s.meanErr * 100, s.maxErr * 100);
    char msg[80]; snprintf(msg, sizeof msg, "%.2f об/с: максимум ошибки < 1%%", w);
    check(s.maxErr < 0.01, msg);
  }

  printf("\n2) Неидеальная квадратура (позиции фронтов сдвинуты на 15%% интервала): рябь на низких скоростях\n");
  printf("   %-10s %-14s %10s %10s\n", "об/с", "мин. фронтов", "СКО", "максимум");
  for (double w : {0.05, 0.5, 3.5}) {
    for (int me : {1, 4}) {
      Trace tr = run([w](double) { return w; }, 2.0, P(me), 0.15);
      Stat s = stat(tr, 0.3, 2.0);
      printf("   %-10.2f %-14d %9.2f%% %9.2f%%\n", w, me, s.rms * 100, s.maxErr * 100);
      if (me == 4 && w == 0.05) check(s.maxErr < 0.03, "0.05 об/с, окно 4 фронта: ошибка < 3% несмотря на неидеальную квадратуру");
    }
  }

  printf("\n3) Остановка с 0.5 об/с: оценка должна уйти в 0 не позже таймаута\n");
  {
    Trace tr = run([](double t) { return t < 1.0 ? 0.5 : 0.0; }, 1.5, P(), 0.0);
    double tZero = -1;
    for (size_t i = 0; i < tr.t.size(); ++i) if (tr.t[i] > 1.0 && tr.est[i] == 0.0) { tZero = tr.t[i] - 1.0; break; }
    printf("   ноль через %.0f мс после остановки\n", tZero * 1000);
    check(tZero > 0 && tZero <= 0.075, "ноль не позже 75 мс (таймаут 60 мс + такт 10 мс)");
  }

  printf("\n4) Торможение 0.5 -> 0 за 1 с: оценка не должна 'залипать' выше истины\n");
  {
    Trace tr = run([](double t) { return t < 1.0 ? 0.5 * (1.0 - t) : 0.0; }, 1.3, P(), 0.0);
    double worst = 0; double tw = 0;
    for (size_t i = 0; i < tr.t.size(); ++i) {
      if (tr.t[i] < 0.2 || tr.t[i] > 0.9) continue;
      const double ex = tr.est[i] - tr.truth[i];
      if (ex > worst) { worst = ex; tw = tr.t[i]; }
    }
    printf("   наибольшее превышение оценки над истиной: %.3f об/с (t=%.2f с, истина %.3f)\n", worst, tw, 0.5 * (1 - tw));
    check(worst < 0.03, "превышение < 0.03 об/с");
  }

  printf("\n5) Старт с места: задержка первой оценки\n");
  {
    Trace tr = run([](double t) { return t < 0.5 ? 0.0 : 0.05; }, 1.5, P(), 0.0, 0, 0, 0.0001);
    double tFirst = -1;
    for (size_t i = 0; i < tr.t.size(); ++i) if (tr.t[i] > 0.5 && tr.est[i] > 0.0) { tFirst = tr.t[i] - 0.5; break; }
    printf("   0.05 об/с: первая ненулевая оценка через %.0f мс (интервал между фронтами 18.5 мс)\n", tFirst * 1000);
    check(tFirst > 0 && tFirst < 0.06, "первая оценка < 60 мс");
  }

  printf("\n6) Реверс 0.5 -> -0.5 об/с через остановку\n");
  {
    Trace tr = run([](double t) { return t < 0.8 ? 0.5 : (t < 1.0 ? 0.5 - (t - 0.8) * 5.0 : -0.5); }, 2.0, P(), 0.0);
    bool okPos = false, okNeg = false;
    for (size_t i = 0; i < tr.t.size(); ++i) {
      if (tr.t[i] > 0.5 && tr.t[i] < 0.7 && fabs(tr.est[i] - 0.5) < 0.01) okPos = true;
      if (tr.t[i] > 1.5 && fabs(tr.est[i] + 0.5) < 0.01) okNeg = true;
    }
    check(okPos && okNeg, "знак и величина верны до и после реверса");
  }

  printf("\n7) Переполнение 32-битных счётчиков (время - через 0.5 с, позиция - через 500 фронтов)\n");
  {
    Trace tr = run([](double) { return 0.5; }, 2.0, P(), 0.0, (uint32_t)(4294967296.0 - 500000.0), (uint32_t)(2147483647u - 500u));
    Stat s = stat(tr, 0.3, 2.0);
    printf("   максимум ошибки: %.3f%%\n", s.maxErr * 100);
    check(s.maxErr < 0.01, "оценка не ломается при переполнении");
  }

  printf("\n8) Фильтр на такте контура (k = 0.5): ступенька 0.5 -> 1.0 об/с\n");
  {
    Trace a = run([](double t) { return t < 0.5 ? 0.5 : 1.0; }, 1.2, P(1, 1.0f), 0.0);
    Trace b = run([](double t) { return t < 0.5 ? 0.5 : 1.0; }, 1.2, P(1, 0.5f), 0.0);
    auto t63 = [](const Trace &x) { for (size_t i = 0; i < x.t.size(); ++i) if (x.t[i] > 0.5 && x.est[i] >= 0.5 + 0.63 * 0.5) return x.t[i] - 0.5; return -1.0; };
    printf("   63%% ступеньки: без фильтра %.0f мс, с k=0.5 %.0f мс\n", t63(a) * 1000, t63(b) * 1000);
    check(t63(a) >= 0 && t63(b) >= t63(a), "фильтр добавляет задержку, а не ломает оценку");
  }

  printf("\n%s (%d не прошло)\n", failures ? "ЕСТЬ ОШИБКИ" : "ВСЕ ПРОВЕРКИ ПРОШЛИ", failures);
  return failures ? 1 : 0;
}
