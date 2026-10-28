// Прогон ступеньки скорости: колёса (модель) + энкодеры + цепочка "оценка скорости + регулятор".
// Цепочка (Chain) задаёт период своего такта и умеет: command(), tick(), dutyL(), dutyR().
#pragma once
#include "sim_common.h"
#include <stdio.h>

static const double WHEEL_R = 0.065 / 2.0;
static const double REV_TO_MS = 2.0 * 3.14159265358979 * WHEEL_R;  // об/с -> м/с

extern unsigned long g_micros;

struct RunResult {
  StepMetrics right;
  StepMetrics left;
  double i2cWritesPerSec = 0;
  double maxDiffRev = 0;   // наибольшая разность скоростей колёс |wL - wR|, об/с
  double peakYawDeg = 0;   // наибольший набег курса из-за разности скоростей, градусы
};

template <class Chain>
RunResult runStep(Chain &ch, double rRev, double tTotal, double asym,
                  const PlantParams &ppR, const PlantParams &ppL,
                  std::vector<double> *traceT = nullptr, std::vector<double> *traceW = nullptr) {
  Wheel wr, wl;
  wr.p = ppR;
  wl.p = ppL;
  EdgeSim er, el;
  er.asym = el.asym = asym;
  const double dt = 50e-6;      // шаг модели, с
  const double tStep = 0.2;     // момент ступеньки задания
  double nextTickUs = ch.periodUs;
  bool commanded = false;
  std::vector<double> t, wR, wL;
  double nextRec = 0.0;
  for (double tt = 0.0; tt < tTotal; tt += dt) {
    const double tUs = tt * 1e6;
    if (!commanded && tt >= tStep) { ch.command(rRev * REV_TO_MS); commanded = true; }
    if (tUs >= nextTickUs) {
      g_micros = (unsigned long)tUs;
      ch.tick((uint32_t)tUs, el, er);
      nextTickUs += ch.periodUs;
    }
    const double thR0 = wr.theta, thL0 = wl.theta;
    wr.step(ch.dutyR(), dt);
    wl.step(ch.dutyL(), dt);
    er.advance(thR0, wr.theta, tUs, dt * 1e6);
    el.advance(-thL0, -wl.theta, tUs, dt * 1e6);   // левый энкодер считает "назад" при движении вперёд
    if (tt >= nextRec) { t.push_back(tt); wR.push_back(wr.w); wL.push_back(wl.w); nextRec += 1e-3; }
  }
  RunResult res;
  {
    double yaw = 0;
    for (size_t i = 0; i < t.size(); ++i) {
      const double d = wR[i] - wL[i];                    // об/с; правое быстрее -> поворот влево
      res.maxDiffRev = std::max(res.maxDiffRev, fabs(d));
      yaw += d * REV_TO_MS / 0.1775 * 1e-3;              // рад, шаг записи 1 мс, база 0.1775 м
      res.peakYawDeg = std::max(res.peakYawDeg, fabs(yaw) * 180.0 / 3.14159265358979);
    }
  }
  res.right = metrics(t, wR, rRev, tStep);
  res.left = metrics(t, wL, rRev, tStep);
  res.i2cWritesPerSec = ch.i2cWrites() / tTotal;
  if (traceT) *traceT = t;
  if (traceW) *traceW = wR;
  return res;
}

inline void printHeader() {
  printf("%-26s %8s %9s %9s %10s %10s %9s %9s\n", "вариант", "цель,об/с", "t10%,мс", "t90%,мс",
         "уст.±5%,мс", "уст.±10%,мс", "перег.,%", "рябь,%");
}
static inline void fmtMs(char *b, double s) { if (s < 0) snprintf(b, 16, "   -"); else snprintf(b, 16, "%.0f", s * 1000.0); }
inline void printRow(const char *name, double r, const StepMetrics &m) {
  char a[16], b[16], c[16], d[16];
  fmtMs(a, m.rise10); fmtMs(b, m.rise90); fmtMs(c, m.settle5); fmtMs(d, m.settle10);
  printf("%-26s %8.2f %9s %9s %10s %10s %9.1f %9.1f\n", name, r, a, b, c, d, m.overshootPct, m.rippleRelPct);
}
