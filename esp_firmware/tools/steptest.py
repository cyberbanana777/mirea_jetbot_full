#!/usr/bin/env python3
# Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
"""
Проверка замкнутого контура скорости: ступеньки уставки и метрики отклика.

    python3 steptest.py run --port /dev/ttyUSB0            # колёса в воздухе
    python3 steptest.py run --port /dev/ttyUSB0 --levels 0.05,0.5,1.5,2.5
    python3 steptest.py run --sim                          # проверка скрипта на модели, без железа
    python3 steptest.py fit step_log.csv                   # пересчитать метрики по сохранённому логу

Для каждой уставки (об/с, оба колеса вперёд): пауза 1.5 с -> ступенька на 2.5 с -> пауза 1.5 с.
Уставка отправляется командой $2 (м/с), скорости берутся из телеметрии $3.
Печатаются: время выхода на 90%, перерегулирование, время установления +-10%, ошибка в
установившемся режиме и рябь (СКО / уставка) за последнюю секунду ступеньки.

Нужен numpy (и pyserial для реального порта). Использует классы из mtcal.py (лежит рядом).
"""
import argparse
import math
import sys

import numpy as np

from mtcal import SerialLink, SimLink, Telemetry, TWO_PI

PRE, HOLD, POST = 1.5, 2.5, 1.5

# Пороги "хорошо" для вердикта (ориентиры, не стандарт)
LIM_T90 = 0.40      # с
LIM_OVER = 15.0     # %
LIM_ERR = 5.0       # % установившейся ошибки
LIM_RIPPLE = 8.0    # %


class ClosedSim(SimLink):
    """Модель платы с замкнутым контуром (feedforward + ПИ, 100 Гц) для проверки самого скрипта."""

    def __init__(self, wheel_r, seed=1):
        super().__init__(wheel_r, seed)
        self.ref = [0.0, 0.0]          # м/с
        self.integ = [0.0, 0.0]
        self.kp, self.ki = 1.0, 6.0
        self.dz, self.gain = 0.35, 0.6
        self.next_ctl = 0.0

    def send(self, text):
        for m in __import__("re").finditer(r"\$(\d+);([^#]*)#", text):
            typ, args = int(m.group(1)), [float(x) for x in m.group(2).split(";") if x]
            self.last_cmd = self.t
            if typ == 2:
                self.ref = [args[0], args[1]]
            elif typ == 12:
                self.telemetry = args[0] >= 0.5
                self.next_tel = self.t

    def _step(self, dt):
        super()._step(dt)
        if self.t < self.next_ctl:
            return
        self.next_ctl += 0.01
        if self.t - self.last_cmd > 1.0:
            self.ref = [0.0, 0.0]
        for i in range(2):
            r = self.ref[i]
            v = self.meas[i] * TWO_PI * self.wheel_r
            if abs(r) < 0.002:
                self.integ[i] = 0.0
                self.duty[i] = 0.0
                continue
            ff = math.copysign(self.dz + self.gain * abs(r), r)
            e = r - v
            self.integ[i] = max(-1 - ff, min(1 - ff, self.integ[i] + self.ki * e * 0.01))
            self.duty[i] = max(-1.0, min(1.0, ff + self.kp * e + self.integ[i]))


def run_steps(link, wheel_r, levels):
    tel = Telemetry(wheel_r)
    log = []   # (уровень об/с, фаза, t, wL, wR, uL, uR)
    state = {"level": 0.0, "phase": "idle", "last": -1e9}

    other = []
    params = []

    def pump():
        for ln in link.poll():
            s = tel.parse(ln)
            if not s and ln:
                if ln.startswith("$4;") or ln.startswith("$2;"):
                    params.append(ln)        # ответы на $13 / $5: их не вытесняют строки одометрии $1
                else:
                    other[:] = (other + [ln])[-5:]
            if s:
                log.append((state["level"], state["phase"]) + s)

    def hold(v_ms, dur):
        end = link.now() + dur
        state["last"] = -1e9                     # новую уставку отправить сразу, а не через остаток 0.1 с
        while link.now() < end:
            if link.now() - state["last"] >= 0.1:
                link.send("$2;%.5f;%.5f;#\r\n" % (v_ms, v_ms))
                state["last"] = link.now()
            link.sleep(0.005)
            pump()

    def idle(dur):
        """Ждать dur секунд, ничего не отправляя (команды подряд встык плата может терять)"""
        end = link.now() + dur
        while link.now() < end:
            link.sleep(0.005)
            pump()

    try:
        # Порядок и паузы такие же, как в ручной проверке, которая работала: команды по одной, с паузами
        other.clear()
        params.clear()
        link.send("$13;#\r\n"); idle(0.4)
        link.send("$5;#\r\n"); idle(0.4)
        for _ in range(10):                      # включаем телеметрию, при неудаче повторяем
            link.send("$12;1;#\r\n"); idle(0.4)
            if log:
                break
        if not log:
            msg = "Нет телеметрии $3: плата не ответила на $12;1;#.\n"
            if other:
                msg += "Плата присылает другое (старая прошивка или не та скорость порта):\n  " + "\n  ".join(other) + "\n"
            else:
                msg += "От платы ничего не пришло: проверьте порт (не занят ли ROS-узлом) и прошивку 2.3.0.\n"
            raise SystemExit(msg)
        print("Параметры в плате (ответы на $13 и $5):")
        for ln in params:
            print("  " + ln)
        if not params:
            print("  (плата не ответила на $13 и $5)")
        for lev in levels:
            state["level"] = lev
            state["phase"] = "pre"
            hold(0.0, PRE)
            state["phase"] = "step"
            hold(lev * TWO_PI * wheel_r, HOLD)
            state["phase"] = "post"
            hold(0.0, POST)
            state["phase"] = "idle"
    finally:
        for _ in range(3):
            link.send("$2;0;0;#\r\n")
            link.sleep(0.05)
        link.send("$12;0;#\r\n")
    return log


def analyze(log, levels):
    """Метрики по каждому уровню и колесу. -> список словарей"""
    rows = []
    for lev in levels:
        seg = np.array([[r[2], r[3], r[4], r[5], r[6]] for r in log if r[0] == lev and r[1] == "step"])
        if len(seg) < 50:
            continue
        t = seg[:, 0] - seg[0, 0]
        for name, col in (("L", 1), ("R", 2)):
            w = seg[:, col]
            u = seg[:, col + 2]
            t10 = next((t[i] for i in range(len(t)) if w[i] >= 0.1 * lev), None)
            t90 = next((t[i] for i in range(len(t)) if w[i] >= 0.9 * lev), None)
            over = max(0.0, (w.max() - lev) / lev * 100.0)
            out = np.nonzero(np.abs(w - lev) > 0.10 * lev)[0]
            if len(out) == 0:
                settle = 0.0
            elif out[-1] >= len(t) - 1:
                settle = None
            else:
                settle = t[out[-1] + 1]
            tail = w[t >= t[-1] - 1.0]
            err = (tail.mean() - lev) / lev * 100.0
            ripple = tail.std() / lev * 100.0
            rows.append(dict(level=lev, wheel=name, t10=t10, t90=t90, over=over, settle=settle,
                             err=err, ripple=ripple, u_end=float(u[t >= t[-1] - 1.0].mean())))
    return rows


def _ms(x):
    return "   --  " if x is None else "%6.0f " % (x * 1000)


def report(rows):
    print("\n=== Отклик замкнутого контура (уставка в об/с, время в мс) ===")
    print("уставка  колесо  t10%   t90%   перерег.%  уст.+-10%  ошибка%  рябь%  ШИМ")
    bad = []
    for r in rows:
        print("%6.2f   %s     %s %s  %8.1f  %s %8.1f %6.1f  %.2f" % (
            r["level"], r["wheel"], _ms(r["t10"]), _ms(r["t90"]), r["over"], _ms(r["settle"]),
            r["err"], r["ripple"], r["u_end"]))
        tag = "%.2f об/с, колесо %s" % (r["level"], r["wheel"])
        if r["t90"] is None:
            bad.append("%s: уставка не достигнута (не хватает ШИМ или мощности? ШИМ в конце %.2f)" % (tag, r["u_end"]))
            continue
        if r["t90"] > LIM_T90:
            bad.append("%s: медленный выход (t90 %.0f мс): ПИ слабый или feedforward занижен" % (tag, r["t90"] * 1000))
        if r["over"] > LIM_OVER:
            if r["level"] <= 0.1:
                bad.append("%s: заброс %.0f%% при трогании: трение покоя (колесо трогается при бо́льшей скважности, чем dz); для малых скоростей это ожидаемо, можно поднять dz в $11 на 0.02-0.04" % (tag, r["over"]))
            else:
                bad.append("%s: перерегулирование %.0f%%: ПИ слишком агрессивный или feedforward завышен (пересчитайте `mtcal.py fit --lam 1.0`)" % (tag, r["over"]))
        if abs(r["err"]) > LIM_ERR:
            bad.append("%s: ошибка в установившемся режиме %.1f%%: интеграл не успел или упёрся в ограничение" % (tag, r["err"]))
        if r["ripple"] > LIM_RIPPLE:
            bad.append("%s: рябь %.1f%%: смотрите SPEED_MIN_EDGES в config.h (2 или 4)" % (tag, r["ripple"]))
        if r["settle"] is None:
            bad.append("%s: за %.1f с скорость так и не вошла в +-10%%" % (tag, HOLD))
    print("\nПороги вердикта: t90 <= %d мс, перерегулирование <= %d%%, ошибка <= %d%%, рябь <= %d%%." %
          (LIM_T90 * 1000, LIM_OVER, LIM_ERR, LIM_RIPPLE))
    if bad:
        print("\nЗамечания:")
        for b in bad:
            print("  ! " + b)
    else:
        print("\nВсе уставки укладываются в пороги.")


def save_csv(log, path):
    with open(path, "w") as f:
        f.write("level_rev_s,phase,t_s,wL_rev_s,wR_rev_s,uL,uR\n")
        for r in log:
            f.write("%.3f,%s,%.6f,%.5f,%.5f,%.4f,%.4f\n" % r)


def load_csv(path):
    log = []
    with open(path) as f:
        next(f)
        for ln in f:
            a = ln.strip().split(",")
            log.append((float(a[0]), a[1]) + tuple(float(x) for x in a[2:7]))
    return log


def make_plot(log, levels, rows, a):
    if getattr(a, "no_plot", False):
        return
    from plots import plot_steps
    plot_steps(log, levels, rows, a.plot, a.show)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--port", default="/dev/ttyUSB0")
    r.add_argument("--baud", type=int, default=115200)
    r.add_argument("--sim", action="store_true")
    r.add_argument("--yes", action="store_true")
    r.add_argument("--levels", default="0.05,0.5,1.5", help="уставки, об/с, через запятую")
    r.add_argument("--boot-wait", type=float, default=5.0, help="пауза после открытия порта на загрузку платы, с (по умолчанию 5)")
    r.add_argument("--radius", type=float, default=0.065 / 2)
    r.add_argument("--out", default="step_log.csv")
    f = sub.add_parser("fit")
    f.add_argument("csv")
    for p in (r, f):
        p.add_argument("--plot", default="step_plot.png", help="куда сохранить график (PNG)")
        p.add_argument("--show", action="store_true", help="открыть график в окне")
        p.add_argument("--no-plot", action="store_true", help="не строить график")
    a = ap.parse_args()

    if a.cmd == "fit":
        log = load_csv(a.csv)
        levels = sorted({r[0] for r in log if r[0] > 0})   # уровень 0 - стартовая пауза до первой ступеньки
        rows = analyze(log, levels)
        report(rows)
        make_plot(log, levels, rows, a)
        return

    levels = [float(x) for x in a.levels.split(",")]
    if a.sim:
        link = ClosedSim(a.radius)
    else:
        link = SerialLink(a.port, a.baud, a.boot_wait)
        if not a.yes:
            print("ВНИМАНИЕ: колёса будут вращаться до %.2f об/с. Поднимите робота, колёса в воздухе." % max(levels))
            if input("Колёса подняты? Введите 'да': ").strip().lower() not in ("да", "yes", "y"):
                sys.exit("Отменено.")
    log = run_steps(link, a.radius, levels)
    save_csv(log, a.out)
    print("Лог сохранён: %s (%d отсчётов)" % (a.out, len(log)))
    rows = analyze(log, levels)
    report(rows)
    make_plot(log, levels, rows, a)


if __name__ == "__main__":
    main()
