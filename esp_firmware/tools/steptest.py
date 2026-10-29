#!/usr/bin/env python3
# Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
"""
Проверка замкнутого контура скорости: ступеньки уставки и метрики отклика.

    python3 steptest.py run --port /dev/ttyUSB0            # колёса в воздухе
    python3 steptest.py run --port /dev/ttyUSB0 --levels 0.05,0.5,1.5,2.5
    python3 steptest.py run --sim                          # проверка скрипта на модели, без железа
    python3 steptest.py fit step_log.csv                   # пересчитать метрики по сохранённому логу

Сравнение ПИ и LADRC (настройки только на время опыта, во flash не пишутся):
    python3 steptest.py run --port /dev/ttyUSB0 --levels 0.2,0.5,1.2 --mode pi        # -> step_log_pi.csv
    python3 steptest.py run --port /dev/ttyUSB0 --levels 0.2,0.5,1.2 --mode ladrc --adrc 5,25,9.82,7.65,2
    python3 steptest.py compare step_log_pi.csv step_log_ladrc.csv                     # таблица и график

Для каждой уставки (об/с, оба колеса вперёд): пауза 1.5 с -> ступенька на 2.5 с -> пауза 1.5 с.
Уставка отправляется командой $2 (м/с), скорости берутся из телеметрии $3.
Печатаются: время выхода на 90%, перерегулирование, время установления +-10%, ошибка в
установившемся режиме и рябь (СКО / уставка) за последнюю секунду ступеньки, а также синхронность
колёс: наибольшая разность скоростей |wL - wR| и набег курса из-за неё (база --base).

Нужен numpy (и pyserial для реального порта). Использует классы из mtcal.py (лежит рядом).
"""
import argparse
import math
import sys

import numpy as np

import adrc
import ui
from mtcal import SerialLink, SimLink, Telemetry, TWO_PI

PRE, HOLD, POST = 1.5, 2.5, 1.5

# Пороги "хорошо" для вердикта (ориентиры, не стандарт)
LIM_T90 = 0.40      # с
LIM_OVER = 15.0     # %
LIM_ERR = 5.0       # % установившейся ошибки
LIM_RIPPLE = 8.0    # %


class ClosedSim(SimLink):
    """Модель платы с замкнутым контуром (feedforward + ПИ или LADRC, 100 Гц) для проверки самого скрипта."""

    def __init__(self, wheel_r, seed=1):
        super().__init__(wheel_r, seed)
        self.ref = [0.0, 0.0]          # м/с
        self.integ = [0.0, 0.0]
        self.kp, self.ki = 1.0, 6.0
        self.dz, self.gain = 0.35, 0.6
        self.next_ctl = 0.0
        # LADRC: b0 модели SimLink = Kv*2*pi*R/Tm ~ 16 (м/с^2) на 1.0
        self.mode = 0
        self.adrc = dict(wc=5.0, wo=25.0, b0L=16.0, b0R=16.0, ff=2)
        self.eso = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]   # z1, z2, u без прямой подачи
        self.prev_sign = [0, 0]

    def send(self, text):
        for m in __import__("re").finditer(r"\$(\d+);([^#]*)#", text):
            typ, args = int(m.group(1)), [float(x) for x in m.group(2).split(";") if x]
            self.last_cmd = self.t
            if typ == 2:
                self.ref = [args[0], args[1]]
            elif typ == 12:
                self.telemetry = args[0] >= 0.5
                self.next_tel = self.t
            elif typ == 15 and args:
                self.mode = 1 if args[0] >= 0.5 else 0
            elif typ == 16 and len(args) >= 4:
                self.adrc = dict(wc=args[0], wo=args[1], b0L=args[2], b0R=args[3], ff=int(args[4]) if len(args) > 4 else 0)
            elif typ == 13:
                self.out += ["$4;%.5f;%.5f;%.5f;%.5f;#" % (self.gain, self.dz, self.gain, self.dz), "$6;0.00000;#",
                             "$7;%d;%.3f;%.3f;%.4f;%.4f;%d;#" % (self.mode, self.adrc["wc"], self.adrc["wo"],
                                                                 self.adrc["b0L"], self.adrc["b0R"], self.adrc["ff"])]
            elif typ == 5:
                self.out.append("$2;%.5f;%.5f;0.00000;%.5f;%.5f;0.00000;#" % (self.kp, self.ki, self.kp, self.ki))

    def _ladrc(self, i, r, v, h=0.01):
        p = self.adrc
        sign = (r > 0.002) - (r < -0.002)
        z = self.eso[i]
        if sign != self.prev_sign[i]:
            z[:] = [v, 0.0, 0.0]
        self.prev_sign[i] = sign
        if not sign:
            z[:] = [v, 0.0, 0.0]
            return 0.0
        b0 = p["b0L"] if i == 0 else p["b0R"]
        wc, wo = p["wc"], p["wo"]
        ff = 0.0 if p["ff"] == 0 else sign * self.dz if p["ff"] == 1 else sign * (self.dz + self.gain * abs(r))
        e = v - z[0]
        z[0] += h * (z[1] + b0 * z[2] + 2 * wo * e)
        z[1] += h * wo * wo * e
        u = max(-1.0, min(1.0, ff + (wc * (r - z[0]) - z[1]) / b0))
        z[2] = u - ff
        return u

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
            if self.mode == 1:
                self.duty[i] = self._ladrc(i, r, v)
                continue
            if abs(r) < 0.002:
                self.integ[i] = 0.0
                self.duty[i] = 0.0
                continue
            ff = math.copysign(self.dz + self.gain * abs(r), r)
            e = r - v
            self.integ[i] = max(-1 - ff, min(1 - ff, self.integ[i] + self.ki * e * 0.01))
            self.duty[i] = max(-1.0, min(1.0, ff + self.kp * e + self.integ[i]))


def run_steps(link, wheel_r, levels, setup=()):
    """setup - команды настройки перед опытом (по одной, с паузами), например $16 и $15 без сохранения"""
    tel = Telemetry(wheel_r)
    log = []   # (уровень об/с, фаза, t, wL, wR, uL, uR)
    state = {"level": 0.0, "phase": "idle", "last": -1e9}

    other = []
    params = []

    def pump():
        for ln in link.poll():
            s = tel.parse(ln)
            if not s and ln:
                if ln.startswith(("$4;", "$2;", "$6;", "$7;")):
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
        for c in setup:
            link.send(c + "\r\n"); idle(0.4)
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
                msg += "От платы ничего не пришло: проверьте порт (не занят ли ROS-узлом) и прошивку 2.4.0.\n"
            raise SystemExit(msg)
        ui.panel("\n".join(params) if params else "(плата не ответила на $13 и $5)", "Параметры в плате (ответы на $13 и $5)", "blue")
        p7 = next((adrc.parse7(ln) for ln in params if adrc.parse7(ln)), None)
        if p7:
            ui.say(adrc.describe(p7), "bold")
        elif setup:
            ui.warn("Плата не прислала $7: прошивка без LADRC, команды --mode/--adrc не сработают")
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


def analyze_sync(log, levels, wheel_r, base):
    """Синхронность колёс на ступеньке: max|wL - wR| (об/с) и набег курса (градусы) за фазу step"""
    out = []
    for lev in levels:
        seg = np.array([[r[2], r[3], r[4]] for r in log if r[0] == lev and r[1] == "step"])
        if len(seg) < 50:
            continue
        t = seg[:, 0]
        d = seg[:, 2] - seg[:, 1]                                 # wR - wL, об/с: > 0 - поворот влево
        dt = np.diff(t, prepend=t[0])
        yaw = np.cumsum(d * TWO_PI * wheel_r / base * dt) * 180.0 / math.pi
        first = t <= t[0] + 1.0                                   # первая секунда - переходный процесс
        out.append(dict(level=lev, max_dw=float(np.abs(d).max()), max_dw_tr=float(np.abs(d[first]).max()),
                        yaw_peak=float(yaw[np.argmax(np.abs(yaw))]), yaw_1s=float(yaw[first][-1]), yaw_end=float(yaw[-1])))
    return out


def report_sync(sync, title="Синхронность колёс (ступенька, обе уставки одинаковые)"):
    if not sync:
        return
    rows = [["%.2f" % r["level"], "%.3f" % r["max_dw_tr"], "%.3f" % r["max_dw"], "%+.2f" % r["yaw_1s"], "%+.2f" % r["yaw_end"]]
            for r in sync]
    ui.table(title, [("уставка", "right"), ("max|wL-wR| 1 с", "right"), ("max|wL-wR|", "right"),
                     ("курс за 1 с, °", "right"), ("курс за ступеньку, °", "right")], rows,
             caption="Курс > 0 - робот уводит влево (правое колесо быстрее). Переходный процесс - первая секунда")


def _ms(x, limit=None):
    if x is None:
        return ("--", "bold red")
    txt = "%.0f" % (x * 1000)
    return (txt, ("green" if x <= limit else "bold red")) if limit else txt


def report(rows, mode=None):
    """mode: 'pi', 'ladrc' или None - от него зависят подсказки"""
    hint_slow = {"pi": "ПИ слабый или feedforward занижен",
                 "ladrc": "увеличьте wc (и wo следом), проверьте прямую подачу ff и b0 (сильно завышенный b0 - медленно)"}
    hint_over = {"pi": "ПИ слишком агрессивный или feedforward завышен (пересчитайте `mtcal.py fit --lam 1.0` или --lambda-s 0.2)",
                 "ladrc": "уменьшите wo или wc; заниженный b0 тоже раскачивает (лучше завысить)"}
    bad = []
    table_rows = []
    for r in rows:
        table_rows.append([
            "%.2f" % r["level"], r["wheel"], _ms(r["t10"]), _ms(r["t90"], LIM_T90),
            ui.good_bad(r["over"], LIM_OVER, "%.1f"),
            _ms(r["settle"]),
            ui.good_bad(abs(r["err"]), LIM_ERR, "%.1f"),
            ui.good_bad(r["ripple"], LIM_RIPPLE, "%.1f"),
            "%.2f" % r["u_end"],
        ])
        tag = "%.2f об/с, колесо %s" % (r["level"], r["wheel"])
        if r["t90"] is None:
            bad.append("%s: уставка не достигнута (не хватает ШИМ или мощности? ШИМ в конце %.2f)" % (tag, r["u_end"]))
            continue
        if r["t90"] > LIM_T90:
            bad.append("%s: медленный выход (t90 %.0f мс): %s" % (tag, r["t90"] * 1000,
                       hint_slow.get(mode, hint_slow["pi"] + "; у LADRC - " + hint_slow["ladrc"])))
        if r["over"] > LIM_OVER:
            if r["level"] <= 0.1:
                bad.append("%s: заброс %.0f%% при трогании: трение покоя (колесо трогается при бо́льшей скважности, чем dz); для малых скоростей это ожидаемо, можно поднять dz в $11 на 0.02-0.04" % (tag, r["over"]))
            else:
                bad.append("%s: перерегулирование %.0f%%: %s" % (tag, r["over"],
                           hint_over.get(mode, hint_over["pi"] + "; у LADRC - " + hint_over["ladrc"])))
        if abs(r["err"]) > LIM_ERR:
            bad.append("%s: ошибка в установившемся режиме %.1f%%: интеграл не успел или упёрся в ограничение" % (tag, r["err"]))
        if r["ripple"] > LIM_RIPPLE:
            bad.append("%s: рябь %.1f%%: смотрите SPEED_MIN_EDGES в config.h (2 или 4)" % (tag, r["ripple"]))
        if r["settle"] is None:
            bad.append("%s: за %.1f с скорость так и не вошла в +-10%%" % (tag, HOLD))
    ui.table("Отклик замкнутого контура (уставка в об/с, время в мс)",
             [("уставка", "right"), ("колесо", "center"), ("t10, мс", "right"), ("t90, мс", "right"),
              ("перерег., %", "right"), ("уст. ±10%, мс", "right"), ("ошибка, %", "right"), ("рябь, %", "right"), ("ШИМ", "right")],
             table_rows,
             caption="Пороги: t90 <= %d мс, перерегулирование <= %d%%, ошибка <= %d%%, рябь <= %d%%" %
             (LIM_T90 * 1000, LIM_OVER, LIM_ERR, LIM_RIPPLE))
    if bad:
        ui.say("Замечания:", "bold")
        for b in bad:
            ui.warn(b)
    else:
        ui.ok("Все уставки укладываются в пороги.")


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


def compare(paths, names, wheel_r, base):
    """Сравнение нескольких логов (например, ПИ и LADRC): метрики рядом и синхронность"""
    logs = [load_csv(pth) for pth in paths]
    levels = sorted(set.intersection(*[{r[0] for r in lg if r[0] > 0} for lg in logs]))
    if not levels:
        sys.exit("В логах нет общих уставок")
    res = [(analyze(lg, levels), analyze_sync(lg, levels, wheel_r, base)) for lg in logs]
    cols = [("уставка", "right"), ("колесо", "center")]
    for n in names:
        cols += [("%s t90" % n, "right"), ("%s перерег.%%" % n, "right"), ("%s уст.±10%%" % n, "right"), ("%s рябь%%" % n, "right")]
    rows = []
    for lev in levels:
        for wh in ("L", "R"):
            row = ["%.2f" % lev, wh]
            for rr, _ in res:
                r = next((x for x in rr if x["level"] == lev and x["wheel"] == wh), None)
                if r is None:
                    row += ["--"] * 4
                    continue
                row += [_ms(r["t90"], LIM_T90), ui.good_bad(r["over"], LIM_OVER, "%.1f"), _ms(r["settle"]),
                        ui.good_bad(r["ripple"], LIM_RIPPLE, "%.1f")]
            rows.append(row)
    ui.table("Сравнение отклика (время в мс)", cols, rows)
    cols = [("уставка", "right")]
    for n in names:
        cols += [("%s max|dW| 1 с" % n, "right"), ("%s курс 1 с, °" % n, "right"), ("%s курс итог, °" % n, "right")]
    rows = []
    for lev in levels:
        row = ["%.2f" % lev]
        for _, ss in res:
            r = next((x for x in ss if x["level"] == lev), None)
            row += ["--"] * 3 if r is None else ["%.3f" % r["max_dw_tr"], "%+.2f" % r["yaw_1s"], "%+.2f" % r["yaw_end"]]
        rows.append(row)
    ui.table("Сравнение синхронности колёс", cols, rows, caption="max|dW| - об/с; курс > 0 - уводит влево")
    return logs, levels, res


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
    r.add_argument("--out", default=None, help="CSV лога (по умолчанию step_log.csv или step_log_<mode>.csv)")
    r.add_argument("--mode", choices=("pi", "ladrc"), help="регулятор на время опыта ($15 без сохранения во flash)")
    r.add_argument("--adrc", help="параметры LADRC на время опыта: wc,wo,b0L,b0R[,ff] ($16 без сохранения)")
    f = sub.add_parser("fit")
    f.add_argument("csv")
    f.add_argument("--mode", choices=("pi", "ladrc"), help="каким регулятором снят лог (для подсказок)")
    c = sub.add_parser("compare", help="сравнить несколько логов, например ПИ и LADRC")
    c.add_argument("csv", nargs="+")
    c.add_argument("--names", help="подписи через запятую (по умолчанию - имена файлов)")
    for p in (r, f, c):
        p.add_argument("--radius", type=float, default=0.065 / 2)
        p.add_argument("--base", type=float, default=0.1775, help="база робота, м (WHEEL_BASE) - для набега курса")
        p.add_argument("--plot", default=None, help="куда сохранить график (PNG)")
        p.add_argument("--show", action="store_true", help="открыть график в окне")
        p.add_argument("--no-plot", action="store_true", help="не строить график")
    a = ap.parse_args()

    if a.cmd == "compare":
        names = a.names.split(",") if a.names else [pth.rsplit("/", 1)[-1].replace(".csv", "") for pth in a.csv]
        if len(names) != len(a.csv):
            sys.exit("--names: подписей должно быть столько же, сколько файлов")
        logs, levels, res = compare(a.csv, names, a.radius, a.base)
        if not a.no_plot:
            from plots import plot_compare
            plot_compare(logs, names, levels, a.radius, a.base, a.plot or "step_compare.png", a.show)
        return

    if a.cmd == "fit":
        log = load_csv(a.csv)
        levels = sorted({r[0] for r in log if r[0] > 0})   # уровень 0 - стартовая пауза до первой ступеньки
        rows = analyze(log, levels)
        report(rows, a.mode)
        report_sync(analyze_sync(log, levels, a.radius, a.base))
        a.plot = a.plot or "step_plot.png"
        make_plot(log, levels, rows, a)
        return

    levels = [float(x) for x in a.levels.split(",")]
    setup = []
    if a.adrc:
        v = [float(x) for x in a.adrc.split(",")]
        if len(v) not in (4, 5):
            sys.exit("--adrc: нужно wc,wo,b0L,b0R[,ff]")
        p = dict(wc=v[0], wo=v[1], b0L=v[2], b0R=v[3], ff=int(v[4]) if len(v) > 4 else 0)
        for n in adrc.check(p):
            ui.warn(n)
        setup.append(adrc.command(p, save=False))
    if a.mode:
        setup.append("$15;%d;0;#" % (1 if a.mode == "ladrc" else 0))
    suffix = ("_" + a.mode) if a.mode else ""
    a.out = a.out or "step_log%s.csv" % suffix
    a.plot = a.plot or "step_plot%s.png" % suffix
    if a.sim:
        link = ClosedSim(a.radius)
    else:
        link = SerialLink(a.port, a.baud, a.boot_wait)
        if not a.yes:
            ui.warn("ВНИМАНИЕ: колёса будут вращаться до %.2f об/с. Поднимите робота, колёса в воздухе." % max(levels))
            if input("Колёса подняты? Введите 'да': ").strip().lower() not in ("да", "yes", "y"):
                sys.exit("Отменено.")
    log = run_steps(link, a.radius, levels, setup)
    save_csv(log, a.out)
    ui.ok("Лог сохранён: %s (%d отсчётов)" % (a.out, len(log)))
    rows = analyze(log, levels)
    report(rows, a.mode)
    report_sync(analyze_sync(log, levels, a.radius, a.base))
    make_plot(log, levels, rows, a)
    if a.mode or a.adrc:
        ui.say("Настройки --mode/--adrc действовали только в этом опыте: при следующем открытии порта плата "
               "перезагрузится с тем, что сохранено во flash (adrc.py mode/set - сохранить).", "dim")


if __name__ == "__main__":
    main()
