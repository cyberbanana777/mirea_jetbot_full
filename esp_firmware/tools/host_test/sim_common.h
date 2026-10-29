// Общее для симуляций: модель колеса с мёртвой зоной и генератор фронтов квадратурного энкодера.
// ПАРАМЕТРЫ МОДЕЛИ - ДОПУЩЕНИЯ, а не измерения вашего мотора. Тест показывает, как ведёт себя
// ЛОГИКА при таком двигателе; реальные цифры даст только лог с железа.
#pragma once
#include <stdint.h>
#include <math.h>
#include <vector>
#include <algorithm>

struct PlantParams {
  double Tm = 0.08;      // механическая постоянная времени, с
  double Kv = 6.27;      // об/с на единицу скважности выше мёртвой зоны
  double dz = 0.33;      // скважность, ниже которой колесо не вращается (трение скольжения)
  double dBreak = 0.36;  // скважность, с которой колесо трогается с места (трение покоя)
};

// Одно колесо: скорость в об/с, вперёд > 0
struct Wheel {
  PlantParams p;
  double w = 0.0;      // об/с
  double theta = 0.0;  // обороты
  void step(double duty, double dt) {
    const double a = fabs(duty);
    if (fabs(w) < 1e-4 && a < p.dBreak) { w = 0.0; return; }   // трение покоя держит колесо
    const double sgn = duty >= 0 ? 1.0 : -1.0;
    const double wss = (a > p.dz) ? sgn * p.Kv * (a - p.dz) : 0.0;
    const double tm = (a > p.dz) ? p.Tm : p.Tm / 3.0;           // без тяги колесо тормозится быстрее
    w += (wss - w) / tm * dt;
    theta += w * dt;
  }
};

// Квадратурный энкодер 1080 фронтов/об. asym - неидеальность квадратуры в долях интервала между
// фронтами (сдвиг фаз не ровно 90 градусов / скважность не 50%): фронт k приходит на позиции k + e[k%4]
struct EdgeSim {
  int32_t edges = 0;     // накопленная позиция, фронтов
  uint32_t tLast = 0;    // время последнего фронта, мкс
  uint32_t ph[4] = {0, 0, 0, 0};  // время последнего фронта каждого типа (id & 3), как в encoders.cpp
  int8_t dir = 1;        // направление последнего фронта
  double asym = 0.0;
  static double pat(int k) { static const double e[4] = {0.0, 1.0, -0.5, 0.5}; return e[k & 3]; }
  double pos(int k) const { return k + asym * pat(k); }
  // theta0 -> theta1 (обороты) за интервал, начало t0_us длительностью dt_us
  void advance(double theta0, double theta1, double t0_us, double dt_us) {
    const double x0 = theta0 * 1080.0, x1 = theta1 * 1080.0;
    if (x1 == x0) return;
    while (x1 >= pos(edges + 1)) {
      const double f = (pos(edges + 1) - x0) / (x1 - x0);
      ++edges; tLast = (uint32_t)(t0_us + std::max(0.0, std::min(1.0, f)) * dt_us);
      ph[(uint32_t)edges & 3u] = tLast; dir = 1;
    }
    while (x1 < pos(edges)) {
      const double f = (pos(edges) - x0) / (x1 - x0);
      --edges; tLast = (uint32_t)(t0_us + std::max(0.0, std::min(1.0, f)) * dt_us);
      ph[(uint32_t)(edges + 1) & 3u] = tLast; dir = -1;
    }
  }
};

struct StepMetrics {
  double rise10 = -1, rise90 = -1, settle5 = -1, settle10 = -1, overshootPct = 0, rippleRelPct = 0, finalRel = 0;
};

// w(t) - истинная скорость; r - целевая (об/с); tStep - момент ступеньки;
// последняя секунда записи считается "установившимся" режимом.
// -1 в поле означает "не достигнуто за время записи".
inline StepMetrics metrics(const std::vector<double> &t, const std::vector<double> &w, double r, double tStep) {
  StepMetrics m;
  const double tEnd = t.back();
  double wmax = 0.0;
  for (size_t i = 0; i < t.size(); ++i) {
    if (t[i] < tStep) continue;
    if (m.rise10 < 0 && w[i] >= 0.1 * r) m.rise10 = t[i] - tStep;
    if (m.rise90 < 0 && w[i] >= 0.9 * r) m.rise90 = t[i] - tStep;
    wmax = std::max(wmax, w[i]);
  }
  m.overshootPct = std::max(0.0, (wmax - r) / r * 100.0);
  double last5 = tStep, last10 = tStep;
  for (size_t i = 0; i < t.size(); ++i) {
    if (t[i] < tStep) continue;
    if (fabs(w[i] - r) > 0.05 * r) last5 = t[i];
    if (fabs(w[i] - r) > 0.10 * r) last10 = t[i];
  }
  m.settle5 = (last5 >= tEnd - 1e-9) ? -1 : last5 - tStep;
  m.settle10 = (last10 >= tEnd - 1e-9) ? -1 : last10 - tStep;
  double s = 0, s2 = 0; int n = 0;
  for (size_t i = 0; i < t.size(); ++i) if (t[i] >= tEnd - 1.0) { s += w[i]; s2 += w[i] * w[i]; ++n; }
  const double mean = s / n;
  m.rippleRelPct = sqrt(std::max(0.0, s2 / n - mean * mean)) / r * 100.0;
  m.finalRel = mean / r;
  return m;
}
