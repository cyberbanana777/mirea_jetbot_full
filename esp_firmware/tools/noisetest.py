#!/usr/bin/env python3
# Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
"""
Откуда шум скорости: квадратура энкодера, механика или метки времени.

    python3 noisetest.py run --port /dev/ttyUSB0          # КОЛЁСА В ВОЗДУХЕ, ~1.5 мин
    python3 noisetest.py run --sim                        # проверка скрипта на модели, без железа
    python3 noisetest.py fit noise_log.csv                # пересчитать анализ по сохранённому логу

Опыт (разомкнутый контур, регулятор не маскирует шум): для каждого режима оценки скорости
(--modes, команда $18;n;align#: n - минимум фронтов в окне, align 1 - окно из целых меток,
0 - прежняя оценка) колёса крутятся на нескольких постоянных скважностях (--duties, команда $10),
скорость пишется по телеметрии $3.

Главная метрика - "иглы": 95-й процентиль отклонения от скользящей медианы, выраженный в фронтах
энкодера за такт. Ошибка положения фронтов (неравномерная квадратура) даёт иглы постоянной
величины в фронтах на любой скорости; дрожание меток времени - постоянной в микросекундах.
  * иглы ~0.4-0.6 фронта при align 0 и в разы меньше при align 1 -> квадратура, исправлено;
  * иглы не уменьшились -> не квадратура: смотрите, постоянны ли они в мкс (метки времени)
    или в спектре есть пик на частоте оборота колеса (механика).

Нужна прошивка 2.4.0 с командой $18;n;align# (без неё опыт идёт только с оценкой из config.h).
Нужны numpy, pyserial (для порта), matplotlib (для графика).
"""
import argparse
import math
import re
import sys

import numpy as np

import ui
from mtcal import SerialLink, SimLink, Telemetry, TWO_PI

SETTLE = 1.0     # с после смены скважности до начала анализа (разгон)
EDGES_PER_REV = 1080
FS = 100.0       # частота телеметрии, Гц


# ----------------------------------------------------------------------------------------------
# Модель платы для проверки скрипта: шум квадратуры ~ 1/окно и исчезает при окне 4,
# плюс механическое биение на частоте оборота колеса
# ----------------------------------------------------------------------------------------------
class NoiseSim(SimLink):
    """Модель платы для проверки скрипта: без выравнивания (align 0) - иглы ~0.5 фронта за такт, как на
    роботе; с выравниванием (align 1) - только мелкий шум и механическое биение на частоте оборота."""

    def __init__(self, wheel_r, seed=1, quad=0.5, mech=0.003):
        super().__init__(wheel_r, seed)
        self.window, self.align = 2, 1
        self.quad, self.mech = quad, mech
        self.phase = [0.0, 0.0]

    def send(self, text):
        super().send(text)
        for m in re.finditer(r"\$18;([^#]*)#", text):
            a = [x for x in m.group(1).split(";") if x]
            if a:
                self.window = max(1, min(8, int(round(float(a[0])))))
            if len(a) > 1:
                self.align = 1 if float(a[1]) >= 0.5 else 0
            self.out.append("$8;%d;%d;#" % (self.window, self.align))

    def sleep(self, dt):
        end = self.t + dt
        while self.t < end - 1e-12:
            self._step(0.001)
            for i in range(2):
                self.phase[i] += self.w[i] * 0.001 * TWO_PI
            if self.telemetry and self.t >= self.next_tel:
                self.next_tel += 0.01
                v = []
                for i in range(2):
                    w = self.meas[i]
                    ept = max(abs(w) * EDGES_PER_REV * 0.01, 1.0)
                    q = 0.0 if self.align else self.quad / ept * self.rng.choice([-1, 0, 0, 0, 1])
                    w = w * (1 + q + self.mech * math.sin(self.phase[i])) + self.rng.normal(0, 0.001)
                    v.append(w * TWO_PI * self.wheel_r)
                t_us = int(self.t0_us + self.t * 1e6) % 2 ** 32
                self.out.append("$3;%d;0.0000;%.4f;%.3f;0.0000;%.4f;%.3f;#" % (t_us, v[0], self.duty[0], v[1], self.duty[1]))


# ----------------------------------------------------------------------------------------------
# Опыт
# ----------------------------------------------------------------------------------------------
def run(link, wheel_r, duties, windows, hold):
    tel = Telemetry(wheel_r)
    log = []                     # (режим "n:align", скважность, t, wL, wR, uL, uR)
    st = {"win": 0, "duty": 0.0, "rec": False}
    replies = []

    def pump():
        for ln in link.poll():
            s = tel.parse(ln)
            if s and st["rec"]:
                log.append((st["win"], st["duty"]) + s)
            elif ln.startswith("$8;"):
                replies.append(ln)

    def idle(dur):
        end = link.now() + dur
        while link.now() < end:
            link.sleep(0.005)
            pump()

    def spin(d, dur, rec):
        st["rec"] = rec
        end = link.now() + dur
        last = -1e9
        while link.now() < end:
            if link.now() - last >= 0.1:            # чаще раза в секунду: иначе аварийная остановка
                link.send("$10;%.4f;%.4f;#\r\n" % (d, d))
                last = link.now()
            link.sleep(0.005)
            pump()
        st["rec"] = False

    try:
        got = False
        for _ in range(10):
            link.send("$12;1;#\r\n")
            st["rec"] = True
            idle(0.4)
            st["rec"] = False
            if log:
                got = True
                break
        if not got:
            raise SystemExit("Нет телеметрии $3: проверьте порт и прошивку (python3 console.py --port ...)")
        log.clear()
        for w in windows:
            n, al = w
            replies.clear()
            link.send("$18;%d;%d;#\r\n" % (n, al))
            idle(0.4)
            label = "%d:%d" % (n, al)
            if not replies:
                ui.warn("Плата не ответила на $18 (прошивка без отладочной команды): режим не меняется, опыт с оценкой из config.h")
                windows = [None]
                w = None
                label = "конф."
            elif replies[-1] != "$8;%d;%d;#" % (n, al):
                ui.warn("Плата ответила %s, ожидалось $8;%d;%d;# (старая прошивка без выравнивания?)" % (replies[-1], n, al))
            st["win"] = label
            ui.say("Режим %s (минимум фронтов : целые метки): скважности %s" % (label, ", ".join("%.2f" % d for d in duties)), "bold")
            for d in duties:
                st["duty"] = d
                spin(d, SETTLE, False)
                spin(d, hold, True)
            spin(0.0, 1.0, False)                  # остановка между режимами
            if w is None:
                break
    finally:
        for _ in range(3):
            link.send("$10;0;0;#\r\n")
            link.sleep(0.05)
        link.send("$2;0;0;#\r\n")
        link.sleep(0.05)
        link.send("$18;2;1;#\r\n")
        link.sleep(0.05)
        link.send("$12;0;#\r\n")
    return log


# ----------------------------------------------------------------------------------------------
# Анализ
# ----------------------------------------------------------------------------------------------
def _segment(log, w, d, col):
    a = np.array([[r[2], r[col]] for r in log if r[0] == w and abs(r[1] - d) < 1e-6])
    return (a[:, 0], a[:, 1]) if len(a) else (np.array([]), np.array([]))


def spectrum(t, x):
    """Равномерная сетка 100 Гц, линейный тренд убран. -> (f, амплитуда)"""
    if len(t) < 64:
        return np.array([]), np.array([])
    tu = np.arange(t[0], t[-1], 1.0 / FS)
    xu = np.interp(tu, t, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    win = np.hanning(len(xu))
    amp = np.abs(np.fft.rfft(xu * win)) * 2 / win.sum()
    f = np.fft.rfftfreq(len(xu), 1.0 / FS)
    return f, amp


def _medfilt(x, k=9):
    p = k // 2
    xp = np.pad(x, p, mode="edge")
    return np.array([np.median(xp[i:i + k]) for i in range(len(x))])


def analyze(log):
    windows = sorted({r[0] for r in log})
    duties = sorted({r[1] for r in log})
    rows = []
    for w in windows:
        for d in duties:
            for name, col in (("L", 3), ("R", 4)):
                t, x = _segment(log, w, d, col)
                if len(x) < 50:
                    continue
                mean = float(np.mean(x))
                if abs(mean) < 0.02:
                    rows.append(dict(win=w, duty=d, wheel=name, mean=mean, std=None))
                    continue
                det = x - np.polyval(np.polyfit(t - t[0], x, 1), t - t[0])
                f, amp = spectrum(t, x)
                band = f > 0.5
                fp = float(f[band][np.argmax(amp[band])]) if band.any() else float("nan")
                ap = float(amp[band].max()) if band.any() else float("nan")
                dev = np.abs(x - _medfilt(x)) / abs(mean)
                p95 = float(np.percentile(dev, 95))
                ept = abs(mean) * EDGES_PER_REV * 0.01          # фронтов за такт 10 мс
                rows.append(dict(win=w, duty=d, wheel=name, mean=mean, std=float(np.std(det)),
                                 rel=float(np.std(det) / abs(mean) * 100), p2p=float(np.ptp(det)),
                                 fpeak=fp, apeak=ap, fwheel=abs(mean), dt_jit=float(np.std(np.diff(t)) * 1e3),
                                 spk_pct=p95 * 100, spk_edges=p95 * ept, spk_us=p95 * 10000))
    return windows, duties, rows


def _find(rows, w, d, wh):
    return next((r for r in rows if r["win"] == w and r["duty"] == d and r["wheel"] == wh and r.get("std") is not None), None)


def report(windows, duties, rows):
    tab = []
    for r in rows:
        if r.get("std") is None:
            tab.append([str(r["win"]), "%.2f" % r["duty"], r["wheel"], "%.3f" % r["mean"], "колесо стоит", "", "", "", "", ""])
            continue
        tab.append([str(r["win"]), "%.2f" % r["duty"], r["wheel"], "%.3f" % r["mean"], "%.4f" % r["std"],
                    ui.good_bad(r["rel"], 2.0, "%.2f"), "%.4f" % r["p2p"], "%.2f" % r["spk_pct"],
                    ui.good_bad(r["spk_edges"], 0.2, "%.2f"), "%.0f" % r["spk_us"]])
    ui.table("Рябь скорости при постоянной скважности (разомкнутый контур)",
             [("режим", "center"), ("скважн.", "right"), ("колесо", "center"), ("скорость, об/с", "right"),
              ("СКО, об/с", "right"), ("СКО, %", "right"), ("размах, об/с", "right"),
              ("иглы, %", "right"), ("иглы, фронтов", "right"), ("иглы, мкс", "right")],
             tab, caption="Режим n:a - минимум фронтов : целые метки (1 - да). Иглы - 95-й процентиль отклонения от "
                          "скользящей медианы. Постоянны в фронтах - квадратура, в мкс - метки времени")

    if len(windows) < 2:
        ui.warn("Режим был один: сравнить нельзя. Нужна прошивка с $18;n;align#.")
        return
    ref = next((w for w in windows if str(w).endswith(":0")), windows[0])
    best = next((w for w in windows if str(w).endswith(":1")), windows[-1])
    verdict = []
    vrows = []
    for d in duties:
        for wh in ("L", "R"):
            a, b = _find(rows, ref, d, wh), _find(rows, best, d, wh)
            if not a or not b or a["std"] <= 0:
                continue
            ratio = b["spk_edges"] / max(a["spk_edges"], 1e-9)
            near_rev = abs(b["fpeak"] / b["fwheel"] - round(b["fpeak"] / b["fwheel"])) < 0.15 and b["fpeak"] > 0.5 \
                and round(b["fpeak"] / b["fwheel"]) in (1, 2, 3, 4)
            if ratio < 0.5:
                why = ("квадратура: исправлено", "bold green")
            elif near_rev:
                why = ("механика (оборот колеса)", "bold yellow")
            else:
                why = ("не окно: метки времени / вращение", "bold yellow")
            verdict.append(why[0])
            vrows.append(["%.2f" % d, wh, "%.2f" % a["spk_edges"], "%.2f" % b["spk_edges"],
                          "%.4f / %.4f" % (a["std"], b["std"]), "%.2f" % ratio, why])
    ui.table("Вывод: режим %s против %s" % (best, ref),
             [("скважн.", "right"), ("колесо", "center"), ("иглы %s, фронтов" % ref, "right"),
              ("иглы %s, фронтов" % best, "right"), ("СКО, об/с", "right"), ("отношение игл", "right"),
              ("главная причина", "left")], vrows,
             caption="Отношение < 0.5: иглы ушли с окном из целых меток - это была квадратура")
    if not verdict:
        return
    q = sum(v.startswith("квадратура") for v in verdict) / len(verdict)
    if q >= 0.5:
        ui.ok("Иглы были от неравномерности фронтов квадратуры, и окно из целых меток их убирает.")
    else:
        ui.warn("Окно рябь почти не убирает: она механическая или от меток времени. Фильтр её только задержит; "
                "для LADRC держать wo ниже, механику проверить (биение колеса, люфт, крепление диска энкодера).")


def save_csv(log, path):
    with open(path, "w") as f:
        f.write("window,duty,t_s,wL_rev_s,wR_rev_s,uL,uR\n")
        for r in log:
            f.write("%s,%.4f,%.6f,%.5f,%.5f,%.4f,%.4f\n" % r)


def load_csv(path):
    log = []
    with open(path) as f:
        next(f)
        for ln in f:
            p = ln.strip().split(",")
            log.append((p[0], float(p[1])) + tuple(float(x) for x in p[2:7]))
    return log


def plot(log, windows, duties, path, show):
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        ui.warn("Графики не построены: pip install matplotlib")
        return
    fig, axs = plt.subplots(len(duties), 4, figsize=(18, 3.2 * len(duties)), squeeze=False)
    colors = ["#1f77b4", "#e07b00", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"]
    for i, d in enumerate(duties):
        for j, (wh, col) in enumerate((("L", 3), ("R", 4))):
            at, af = axs[i, j], axs[i, j + 2]
            for k, w in enumerate(windows):
                t, x = _segment(log, w, d, col)
                if len(x) < 50:
                    continue
                c = colors[k % len(colors)]
                at.plot(t - t[0], x, color=c, lw=0.9, label="режим %s" % w)
                f, amp = spectrum(t, x)
                if len(f):
                    af.semilogy(f[1:], amp[1:] + 1e-6, color=c, lw=0.9, label="режим %s" % w)
            at.set_title("%s, скважность %.2f: скорость" % (wh, d), fontsize=9)
            at.set_xlabel("с")
            at.set_ylabel("об/с")
            at.grid(alpha=0.3)
            af.set_title("%s, скважность %.2f: спектр ряби" % (wh, d), fontsize=9)
            af.set_xlabel("Гц")
            af.set_ylabel("амплитуда, об/с")
            af.grid(alpha=0.3, which="both")
    axs[0, 0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    ui.ok("График сохранён: %s" % path)
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--port", default="/dev/ttyUSB0")
    r.add_argument("--baud", type=int, default=115200)
    r.add_argument("--boot-wait", type=float, default=5.0)
    r.add_argument("--sim", action="store_true", help="модель платы вместо реальной")
    r.add_argument("--yes", action="store_true", help="не спрашивать подтверждение")
    r.add_argument("--duties", default="0.14,0.2,0.3,0.5",
                   help="скважности через запятую (в воздухе по калибровке: левое ~0.3/0.8/1.6/3.3 об/с, правое ~0.15/0.5/1.0/2.1)")
    r.add_argument("--modes", default="2:0,2:1",
                   help="режимы оценки скорости 'n:align' через запятую (команда $18): 2:0 - прежняя, 2:1 - целые метки")
    r.add_argument("--hold", type=float, default=6.0,
                   help="запись на каждой скважности, с (для спектра нужно хотя бы 2-3 оборота колеса)")
    r.add_argument("--out", default="noise_log.csv")
    f = sub.add_parser("fit")
    f.add_argument("csv")
    for p in (r, f):
        p.add_argument("--radius", type=float, default=0.065 / 2)
        p.add_argument("--plot", default="noise_plot.png")
        p.add_argument("--show", action="store_true")
        p.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    if a.cmd == "fit":
        log = load_csv(a.csv)
    else:
        duties = [float(x) for x in a.duties.split(",")]
        windows = [tuple(int(v) for v in m.split(":")) for m in a.modes.split(",")]
        if any(d > 1.0 or d < 0 for d in duties):
            sys.exit("Скважности от 0 до 1")
        total = len(windows) * (len(duties) * (SETTLE + a.hold) + 1.0)
        if a.sim:
            link = NoiseSim(a.radius)
        else:
            link = SerialLink(a.port, a.baud, a.boot_wait)
            if not a.yes:
                ui.warn("ВНИМАНИЕ: моторы будут крутиться до скважности %.2f около %.0f с. КОЛЁСА ДОЛЖНЫ БЫТЬ В ВОЗДУХЕ."
                        % (max(duties), total))
                if input("Колёса подняты? Введите 'да': ").strip().lower() not in ("да", "yes", "y"):
                    sys.exit("Отменено.")
        try:
            log = run(link, a.radius, duties, windows, a.hold)
        except KeyboardInterrupt:
            for _ in range(3):
                link.send("$10;0;0;#\r\n")
            raise
        save_csv(log, a.out)
        ui.ok("Лог сохранён: %s (%d отсчётов)" % (a.out, len(log)))
    windows, duties, rows = analyze(log)
    report(windows, duties, rows)
    if not a.no_plot:
        plot(log, windows, duties, a.plot, a.show)


if __name__ == "__main__":
    main()
