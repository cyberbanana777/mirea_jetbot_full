// ПИ против LADRC на настоящем коде прошивки (motor_controller.cpp + оценка скорости M/T + модель энкодеров).
// Оба колеса, ступенька с места; синхронность: max|wL - wR| и набег курса. Плюс проверка безударного
// переключения режима на ходу. Параметры моделей - допущения (как в sim_sync.cpp), а не измерения.
#include "new_chain.h"

struct Cfg { const char *name; bool adrc; float wc, wo; int ff; float tau; };

static void scenario(const char *title, PlantParams L, PlantParams R) {
  printf("\n=== %s ===\n", title);
  // ПИ: общая lambda 0.2 с по воздушной калибровке, FF по полу (то, что сейчас в плате);
  // LADRC: b0 по воздуху, прямая подача (если включена) - тот же FF по полу
  const Cfg cfgs[] = {
    {"ПИ, общая lambda 0.2, FF пол         ", false, 0, 0, 0, 0.0f},
    {"ПИ + фильтр уставки 0.15             ", false, 0, 0, 0, 0.0f + 0.15f},
    {"LADRC wc 5, wo 25, без FF            ", true, 5, 25, 0, 0.0f},
    {"LADRC wc 5, wo 25, FF мёртвая зона   ", true, 5, 25, 1, 0.0f},
    {"LADRC wc 5, wo 25, полный FF         ", true, 5, 25, 2, 0.0f},
    {"LADRC wc 5, wo 15, полный FF         ", true, 5, 15, 2, 0.0f},
    {"LADRC wc 5, wo 40, полный FF         ", true, 5, 40, 2, 0.0f},
    {"LADRC wc 9, wo 40, полный FF         ", true, 9, 40, 2, 0.0f},
    {"LADRC wc 5, wo 25, полн.FF + фильтр  ", true, 5, 25, 2, 0.15f},
  };
  MTSpeedParams sp = {1.0f / 1080.0f, 60000, 2, 6.0f, 1.0f};
  for (double r : {0.2, 0.5, 1.2}) {
    printf("\nуставка %.1f об/с\n%-38s %7s %7s %8s %8s %9s %9s\n", r, "вариант", "t90 L", "t90 R", "пер.L,%", "пер.R,%", "max|dW|", "курс,гр");
    for (const Cfg &c : cfgs) {
      NewChain ch(sp, 0.463f, 2.66f, 0.0f, 0.5151f, 0.2992f);
      ch.ctl.setPID(0.463f, 2.66f, 0, 0.594f, 4.153f, 0);
      ch.ctl.setFeedforward(0.5151f, 0.2992f, 0.8424f, 0.3101f);
      ch.ctl.setRefFilter(c.tau);
      ch.ctl.setAdrc(c.wc, c.wo, 9.82f, 7.65f, (uint8_t)c.ff);
      ch.ctl.setControlMode(c.adrc ? ControlMode::kLadrc : ControlMode::kPi);
      RunResult res = runStep(ch, r, 3.0, 0.0, R, L);
      printf("%-38s %7.0f %7.0f %8.1f %8.1f %9.3f %9.2f\n", c.name, res.left.rise90 * 1000, res.right.rise90 * 1000,
             res.left.overshootPct, res.right.overshootPct, res.maxDiffRev, res.peakYawDeg);
    }
  }
}

// Переключение режима на ходу: наибольший скачок скважности за такт в окне переключения
// против такого же скачка в обычной работе (до переключения)
static void bumpless(PlantParams L, PlantParams R) {
  printf("\n=== Безударное переключение на ходу (пол, 0.8 об/с, переключение в 1.5 с и обратно в 2.5 с) ===\n");
  MTSpeedParams sp = {1.0f / 1080.0f, 60000, 2, 6.0f, 1.0f};
  for (int startAdrc = 0; startAdrc < 2; ++startAdrc) {
    NewChain ch(sp, 0.463f, 2.66f, 0.0f, 0.5151f, 0.2992f);
    ch.ctl.setPID(0.463f, 2.66f, 0, 0.594f, 4.153f, 0);
    ch.ctl.setFeedforward(0.5151f, 0.2992f, 0.8424f, 0.3101f);
    ch.ctl.setRefFilter(0.0f);
    ch.ctl.setAdrc(5, 25, 9.82f, 7.65f, 2);
    ch.ctl.setControlMode(startAdrc ? ControlMode::kLadrc : ControlMode::kPi);
    Wheel wr, wl; wr.p = R; wl.p = L;
    EdgeSim er, el;
    const double dt = 50e-6; double next = ch.periodUs; float prevU = 0; double jumpNormal = 0, jumpSwitch = 0;
    double wMin = 1e9, wMax = 0; bool sw1 = false, sw2 = false;
    const double r = 0.8 * REV_TO_MS;
    for (double t = 0; t < 3.5; t += dt) {
      if (t >= 0.0 && t < dt) ch.command(r);
      if (!sw1 && t >= 1.5) { ch.ctl.setControlMode(startAdrc ? ControlMode::kPi : ControlMode::kLadrc); sw1 = true; }
      if (!sw2 && t >= 2.5) { ch.ctl.setControlMode(startAdrc ? ControlMode::kLadrc : ControlMode::kPi); sw2 = true; }
      if (t * 1e6 >= next) {
        g_micros = (unsigned long)(t * 1e6);
        if (fmod(t, 0.5) < 0.0101) ch.command(r);   // не дать сработать аварийной остановке не нужно: это не .ino
        ch.tick((uint32_t)(t * 1e6), el, er);
        next += ch.periodUs;
        const float u = (float)ch.dutyL();
        const double d = fabs(u - prevU);
        if (t > 1.0 && t < 1.45) jumpNormal = std::max(jumpNormal, d);
        if ((t > 1.5 && t < 1.53) || (t > 2.5 && t < 2.53)) jumpSwitch = std::max(jumpSwitch, d);
        prevU = u;
      }
      const double thR0 = wr.theta, thL0 = wl.theta;
      wr.step(ch.dutyR(), dt); wl.step(ch.dutyL(), dt);
      er.advance(thR0, wr.theta, t * 1e6, dt * 1e6);
      el.advance(-thL0, -wl.theta, t * 1e6, dt * 1e6);
      if (t > 1.5 && t < 3.0) { wMin = std::min(wMin, wl.w); wMax = std::max(wMax, wl.w); }
    }
    printf("%s: скачок скважности за такт в обычной работе %.4f, в моменты переключения %.4f; скорость L в 1.5-3 с: %.3f..%.3f об/с\n",
           startAdrc ? "LADRC -> ПИ -> LADRC" : "ПИ -> LADRC -> ПИ", jumpNormal, jumpSwitch, wMin, wMax);
  }
}

int main() {
  PlantParams L, R;
  L.Tm = 0.174; L.Kv = 8.37; L.dz = 0.107; L.dBreak = 0.18;
  R.Tm = 0.143; R.Kv = 5.36; R.dz = 0.111; R.dBreak = 0.20;
  scenario("ВОЗДУХ", L, R);
  PlantParams L2 = L, R2 = R;
  L2.Tm = 0.25; L2.dz = 0.30; L2.dBreak = 0.42;
  R2.Tm = 0.20; R2.dz = 0.31; R2.dBreak = 0.49;
  scenario("ПОЛ (Tm больше, мёртвая зона и трение покоя выше; b0 LADRC по воздуху)", L2, R2);
  bumpless(L2, R2);
  return 0;
}
