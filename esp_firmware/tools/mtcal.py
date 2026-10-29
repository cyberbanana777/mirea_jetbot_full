#!/usr/bin/env python3
# Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
"""
Калибровка контура скорости (feedforward + ПИ) по трём коротким опытам на разомкнутом приводе.

    python3 mtcal.py run --port /dev/ttyUSB0          # опыты на роботе (колёса ПОДНЯТЬ над полом!)
    python3 mtcal.py run --sim                        # то же на встроенной модели, без железа
    python3 mtcal.py fit cal_log.csv                  # пересчитать результат по сохранённому логу

Что делает `run` (оба колеса одновременно, обороты считаются по телеметрии $3):
  A. Плавный подъём скважности ШИМ с шагом 0.01 до страгивания каждого колеса  -> скважность трогания.
  B. Ступеньки скважности 1.0 -> 0.35 вниз: установившиеся скорости, прямая w = Kv*(d - dz)
     даёт мёртвую зону dz и усиление Kv (у feedforward это dz и gain = 1/(Kv*2*pi*R)).
  C. Ступенька скважности внутри рабочей области: постоянная времени Tm и запаздывание L
     (метод двух точек 28.3% / 63.2%) -> коэффициенты ПИ.
В конце печатаются команды $11 (feedforward) и $4 (ПИ), их можно сразу отправить флагом --apply.

Нужен только numpy (и pyserial для реального порта).
"""
import argparse
import math
import re
import sys
import time

import numpy as np

import ui

TWO_PI = 2.0 * math.pi
LINE_RE = re.compile(r"\$3;([^#]*)#")


# ----------------------------------------------------------------------------------------------
# Связь с робототехнической платой: реальный порт и модель для проверки без железа
# ----------------------------------------------------------------------------------------------
class SerialLink:
    def __init__(self, port, baud, boot_wait=5.0):
        import serial  # pyserial
        self.s = serial.Serial(port, baud, timeout=0)
        self.buf = b""
        # Открытие порта по DTR/RTS перезагружает ESP32: ждём конца загрузки прошивки
        ui.say("Порт открыт, жду загрузку платы %.1f с..." % boot_wait, "dim")
        time.sleep(boot_wait)
        self.s.reset_input_buffer()

    def now(self):
        return time.monotonic()

    def sleep(self, dt):
        time.sleep(dt)

    def send(self, text):
        self.s.write(text.encode("ascii"))

    def poll(self):
        self.buf += self.s.read(4096)
        *lines, self.buf = self.buf.split(b"\n")
        return [ln.decode("ascii", "ignore").strip() for ln in lines]


class SimLink:
    """Модель платы: разомкнутый режим, телеметрия $3 на 100 Гц, аварийная остановка без команд.
    Параметры двигателя - допущения; нужна, чтобы проверить сам скрипт."""

    def __init__(self, wheel_r, seed=1):
        self.t = 0.0
        self.wheel_r = wheel_r
        self.rng = np.random.default_rng(seed)
        # (Tm, Kv об/с на скважность, мёртвая зона, трогание) для левого и правого колеса
        self.p = [(0.08, 6.27 * 0.95, 0.35, 0.38), (0.08, 6.27, 0.33, 0.36)]
        self.w = [0.0, 0.0]
        self.meas = [0.0, 0.0]
        self.duty = [0.0, 0.0]
        self.last_cmd = -1e9
        self.telemetry = False
        self.next_tel = 0.0
        self.out = []
        self.t0_us = 2 ** 32 - 3_000_000  # переполнение micros() посреди опыта

    def now(self):
        return self.t

    def send(self, text):
        for m in re.finditer(r"\$(\d+);([^#]*)#", text):
            typ, args = int(m.group(1)), [float(x) for x in m.group(2).split(";") if x]
            self.last_cmd = self.t   # как emergency_timer в прошивке: сбрасывается любой командой
            if typ == 10:
                self.duty = [max(-1.0, min(1.0, args[0])), max(-1.0, min(1.0, args[1]))]
            elif typ == 12:
                self.telemetry = args[0] >= 0.5
                self.next_tel = self.t

    def _step(self, dt):
        if self.t - self.last_cmd > 1.0:
            self.duty = [0.0, 0.0]  # аварийная остановка
            self.telemetry = False
        for i in range(2):
            tm, kv, dz, db = self.p[i]
            d = self.duty[i]
            a = abs(d)
            if abs(self.w[i]) < 1e-4 and a < db:
                self.w[i] = 0.0
            else:
                wss = math.copysign(kv * (a - dz), d) if a > dz else 0.0
                self.w[i] += (wss - self.w[i]) / (tm if a > dz else tm / 3) * dt
            # измерение: запаздывание ~12 мс, шум, ноль ниже порога таймаута
            self.meas[i] += (self.w[i] - self.meas[i]) * min(1.0, dt / 0.012)
        self.t += dt

    def sleep(self, dt):
        end = self.t + dt
        while self.t < end - 1e-12:
            self._step(0.001)
            if self.telemetry and self.t >= self.next_tel:
                self.next_tel += 0.01
                v = [(m + self.rng.normal(0, 0.002)) * TWO_PI * self.wheel_r for m in self.meas]
                t_us = int(self.t0_us + self.t * 1e6) % 2 ** 32
                self.out.append("$3;%d;0.0000;%.4f;%.3f;0.0000;%.4f;%.3f;#" % (t_us, v[0], self.duty[0], v[1], self.duty[1]))

    def poll(self):
        out, self.out = self.out, []
        return out


# ----------------------------------------------------------------------------------------------
# Разбор телеметрии
# ----------------------------------------------------------------------------------------------
class Telemetry:
    """Разбирает строки $3;t_us;refL;vL;uL;refR;vR;uR;# в обороты в секунду, разворачивая переполнение micros()."""

    def __init__(self, wheel_r):
        self.k = 1.0 / (TWO_PI * wheel_r)  # м/с -> об/с
        self.prev = None
        self.wrap = 0

    def parse(self, line):
        m = LINE_RE.search(line)
        if not m:
            return None
        f = m.group(1).split(";")
        if len(f) < 7:
            return None
        try:
            t_us = int(f[0])
            vL, uL, vR, uR = float(f[2]), float(f[3]), float(f[5]), float(f[6])
        except ValueError:
            return None
        if self.prev is not None and t_us < self.prev - 2 ** 31:
            self.wrap += 1
        self.prev = t_us
        return ((t_us + self.wrap * 2 ** 32) * 1e-6, vL * self.k, vR * self.k, uL, uR)


# ----------------------------------------------------------------------------------------------
# Опыты
# ----------------------------------------------------------------------------------------------
class Experiment:
    def __init__(self, link, wheel_r):
        self.link = link
        self.tel = Telemetry(wheel_r)
        self.log = []          # (фаза, t, wL, wR, uL, uR)
        self.phase = "idle"
        self._last_send = -1e9

    def _pump(self):
        new = []
        for ln in self.link.poll():
            s = self.tel.parse(ln)
            if not s and ln:
                self.other = (getattr(self, "other", []) + [ln])[-5:]
            if s:
                self.log.append((self.phase,) + s)
                new.append(s)
        return new

    def hold(self, d, dur, stop_when=None):
        """Держать скважность d на обоих колёсах dur секунд (команда повторяется: иначе сработает аварийная остановка)."""
        end = self.link.now() + dur
        while self.link.now() < end:
            if self.link.now() - self._last_send >= 0.1:
                self.link.send("$10;%.4f;%.4f;#\r\n" % (d, d))
                self._last_send = self.link.now()
            self.link.sleep(0.005)
            new = self._pump()
            if stop_when and new and stop_when(new[-1]):
                return True
        return False

    def run(self):
        L = self.link
        # Команду повторяем до 4 с: первая могла уйти, пока плата ещё загружалась
        for _ in range(8):
            L.send("$12;1;#\r\n")
            self.hold(0.0, 0.5)
            if self.log:
                break
        if not self.log:
            other = getattr(self, "other", [])
            msg = "Нет телеметрии $3: плата не ответила на $12;1;#.\n"
            if other:
                msg += "Плата присылает другое (скорее всего, старая прошивка или не та скорость порта):\n  " + "\n  ".join(other) + "\n"
            else:
                msg += "От платы вообще ничего не пришло: проверьте порт, не занят ли он другой программой (ROS-узлом), и что прошита версия 2.3.0.\n"
            raise SystemExit(msg)

        # A. страгивание
        self.phase = "ramp"
        thr = 0.03  # об/с
        found = {}
        d = 0.0
        while d < 0.80 and len(found) < 2:
            d = round(d + 0.01, 4)

            def moved(s, found=found):
                if "L" not in found and s[1] > thr:
                    found["L"] = s[3]
                if "R" not in found and s[2] > thr:
                    found["R"] = s[4]
                return len(found) == 2

            if self.hold(d, 0.25, stop_when=moved):
                break
        self.phase = "idle"
        self.hold(0.0, 1.2)
        if len(found) < 2:
            self._stop()
            raise SystemExit("Колёса не тронулись до 80%% ШИМ (сдвинулось: %s). Проверьте питание, драйвер и знак энкодеров." % (list(found) or "ни одно"))

        # B. статика, сверху вниз (без гистерезиса трогания)
        self.phase = "static"
        self.hold(1.0, 1.2)
        for lev in (1.0, 0.8, 0.6, 0.5, 0.4, 0.35, 0.3, 0.25, 0.2, 0.17, 0.14):
            self.hold(lev, 1.3)
        self.phase = "idle"
        self.hold(0.0, 1.0)

        # C. ступенька в рабочей области (уровни по результату статики)
        st = fit_static(self.log)
        dz = np.mean([st[w][1] for w in ("L", "R")])
        d_lo, d_hi = dz + 0.25 * (1 - dz), dz + 0.60 * (1 - dz)
        self.phase = "step"
        self.hold(d_lo, 1.5)
        self.hold(d_hi, 1.5)
        self.phase = "idle"
        self._stop()

    def _stop(self):
        for _ in range(3):
            self.link.send("$10;0;0;#\r\n")
            self.link.sleep(0.05)
        self.link.send("$12;0;#\r\n")


# ----------------------------------------------------------------------------------------------
# Анализ (чистые функции, работают и по сохранённому логу)
# ----------------------------------------------------------------------------------------------
def _cols(log, phase):
    rows = [r for r in log if r[0] == phase]
    return np.array([r[1:] for r in rows]) if rows else np.zeros((0, 5))


def fit_breakaway(log, thr=0.03):
    a = _cols(log, "ramp")
    res = {}
    for name, wi, ui in (("L", 1, 3), ("R", 2, 4)):
        idx = np.where(a[:, wi] > thr)[0] if len(a) else []
        res[name] = float(a[idx[0], ui]) if len(idx) else None
    return res


FIT_MAX_W = 2.2   # об/с; верх рабочего диапазона для подгонки статики (MAX_CONSTRUCTIVE_VELOCITY 0.5 м/с ~ 1.96 об/с)


def fit_static(log):
    """{'L': (Kv об/с на скважность, dz, R^2, [(d, w), ...]), 'R': ...}"""
    a = _cols(log, "static")
    out = {}
    if not len(a):
        raise ValueError("в логе нет фазы static")
    for name, wi, ui in (("L", 1, 3), ("R", 2, 4)):
        levels = {}
        for d in sorted(set(np.round(a[:, ui], 3)), reverse=True):
            if d <= 0.0:
                continue
            seg = a[np.isclose(a[:, ui], d, atol=1e-3)]
            if len(seg) < 20:
                continue
            tail = seg[seg[:, 0] >= seg[-1, 0] - 0.5]       # последние 0.5 с уровня: установившийся режим
            levels[float(d)] = float(np.mean(tail[:, wi]))
        # точки у самой мёртвой зоны нелинейны (трение): в подгонку берём только уверенное вращение
        # Подгонка только в рабочем диапазоне скоростей (до FIT_MAX_W): feedforward нужен там, а у мотора
        # с редуктором характеристика у максимума обычно загибается, и линия по всем точкам даёт
        # нефизичную мёртвую зону (даже отрицательную).
        pts = [(d, w) for d, w in levels.items() if 0.2 < w <= FIT_MAX_W]
        if len(pts) < 3:
            pts = [(d, w) for d, w in levels.items() if w > 0.2]
        if len(pts) < 3:
            raise ValueError("колесо %s: слишком мало точек статики с ненулевой скоростью (%d)" % (name, len(pts)))
        x = np.array([p[0] for p in pts]); y = np.array([p[1] for p in pts])
        kv, b = np.polyfit(x, y, 1)
        r2 = 1 - np.sum((y - (kv * x + b)) ** 2) / max(1e-12, np.sum((y - y.mean()) ** 2))
        dz = float(-b / kv)
        out[name] = (float(kv), max(dz, 0.0), float(r2), sorted(pts), max(levels.values()))
    return out


def fit_step(log):
    """Двухточечная идентификация (28.3% / 63.2%): {'L': (Tm, L), 'R': (Tm, L)}"""
    a = _cols(log, "step")
    if not len(a):
        raise ValueError("в логе нет фазы step")
    jump = np.where(np.abs(np.diff(a[:, 3])) > 0.01)[0]
    if not len(jump):
        raise ValueError("не найден скачок скважности в фазе step")
    j = jump[-1] + 1          # последний скачок = переход d_lo -> d_hi (первый, 0 -> d_lo, нам не нужен)
    tj = a[j, 0]
    out = {}
    for name, wi in (("L", 1), ("R", 2)):
        pre = a[(a[:, 0] < tj) & (a[:, 0] > tj - 0.3), wi]
        post = a[a[:, 0] > a[-1, 0] - 0.3, wi]
        w0, w1 = float(np.mean(pre)), float(np.mean(post))
        if abs(w1 - w0) < 0.05:
            raise ValueError("колесо %s: ступенька скорости слишком мала (%.3f -> %.3f об/с)" % (name, w0, w1))
        seg = a[a[:, 0] >= tj]
        t = seg[:, 0] - tj
        y = (seg[:, wi] - w0) / (w1 - w0)

        def cross(level):
            i = np.where(y >= level)[0]
            if not len(i) or i[0] == 0:
                return None
            k = i[0]
            return float(t[k - 1] + (level - y[k - 1]) * (t[k] - t[k - 1]) / (y[k] - y[k - 1]))

        t1, t2 = cross(0.283), cross(0.632)
        if t1 is None or t2 is None:
            raise ValueError("колесо %s: ступенька не дошла до 63%%" % name)
        tm = 1.5 * (t2 - t1)
        lag = max(0.02, t2 - tm)   # не меньше задержки самого контура: такт 10 мс + половина интервала фронтов
        out[name] = (max(tm, 0.01), lag)
    return out


def recommend(log, wheel_r, lam_factor=0.5, lambda_s=None):
    ff, pi = {}, {}
    br = fit_breakaway(log)
    st = fit_static(log)
    sp = fit_step(log)
    for name in ("L", "R"):
        kv, dz, r2, _, _ = st[name]
        tm, lag = sp[name]
        k_ms = kv * TWO_PI * wheel_r                       # м/с на единицу скважности
        ff[name] = (1.0 / k_ms, dz)                        # (gain, dz)
        lam = max(lambda_s if lambda_s else lam_factor * tm, 2.0 * lag)              # желаемая постоянная замкнутого контура (не быстрее 2 задержек: иначе колебания)
        kp = tm / (k_ms * (lam + lag))                     # IMC/лямбда-настройка ПИ для звена 1-го порядка с запаздыванием
        ki = kp / tm
        pi[name] = (min(kp, 3.0), min(ki, 30.0))
    return br, st, sp, ff, pi


def report(log, wheel_r, lam_factor, lambda_s=None):
    br, st, sp, ff, pi = recommend(log, wheel_r, lam_factor, lambda_s)
    rows, notes = [], []
    for n in ("L", "R"):
        kv, dz, r2, pts, top = st[n]
        brk = br[n]
        rows.append([
            n,
            ("%.2f" % brk) if brk is not None else "?",
            "%.3f" % dz,
            "%.2f" % kv,
            ("%.4f" % r2, "green" if r2 >= 0.99 else "bold red"),
            "%d" % len(pts),
            "%.0f" % (sp[n][0] * 1000),
            "%.0f" % (sp[n][1] * 1000),
            ("%.2f" % top, "green" if top >= 3.5 else "bold red"),
        ])
        if brk is not None and brk - dz > 0.05:
            notes.append("Колесо %s: страгивание (%.2f) заметно выше dz (%.2f): на скоростях около 0.05 об/с возможен рывок при трогании" % (n, brk, dz))
        if r2 < 0.99:
            notes.append("Колесо %s: R^2 < 0.99: статика нелинейна (проверьте, что колёса свободно вращаются и опыт шёл с поднятыми колёсами)" % n)
        if top < 3.5:
            notes.append("Колесо %s: при ШИМ 1.0 даёт %.2f об/с: 3.5 об/с недостижимы (проверьте питание)" % (n, top))
    ui.table("Результат калибровки",
             [("колесо", "center"), ("страгивание, ШИМ", "right"), ("dz", "right"), ("Kv, об/с на 1.0", "right"),
              ("R^2", "right"), ("точек", "right"), ("Tm, мс", "right"), ("L, мс", "right"), ("макс., об/с", "right")],
             rows)
    pi_rows = [[n, "%.4f" % ff[n][0], "%.4f" % ff[n][1], "%.3f" % pi[n][0], "%.3f" % pi[n][1]] for n in ("L", "R")]
    lam_txt = ("общая lambda = %.2f с" % lambda_s) if lambda_s else ("lambda = %.2f*Tm у каждого колеса" % lam_factor)
    ui.table("Рекомендуемые коэффициенты (%s)" % lam_txt,
             [("колесо", "center"), ("FF gain", "right"), ("FF dz", "right"), ("Kp", "right"), ("Ki", "right")], pi_rows)
    for n in notes:
        ui.warn(n)
    cmd_ff = "$11;%.4f;%.4f;%.4f;%.4f;1;#" % (ff["L"][0], ff["L"][1], ff["R"][0], ff["R"][1])
    cmd_pi = "$4;%.3f;%.3f;0;%.3f;%.3f;0;#" % (pi["L"][0], pi["L"][1], pi["R"][0], pi["R"][1])
    ui.panel(cmd_ff + "\n" + cmd_pi, "Команды для платы (по порядку: feedforward, затем ПИ; оба с сохранением во flash)", "green")
    ui.say("Если ПИ окажется колебательным - пересчитайте с --lam 1.0 (или 1.5) либо с общей --lambda-s 0.3: python3 mtcal.py fit cal_log.csv --lam 1.0", "dim")
    # LADRC по той же калибровке (b0 = K/Tm); подробнее и запись в плату - adrc.py
    import adrc
    p, notes = adrc.recommend(st, sp, wheel_r, lambda_s=lambda_s or 0.2)
    ui.say("LADRC по этой калибровке (wc = 1/lambda, wo = 5*wc, полный FF): %s" % adrc.command(p, True), "dim")
    ui.say("Подробно и с проверками: python3 adrc.py fit <лог> [--lambda-s 0.2]", "dim")
    return cmd_ff, cmd_pi


def save_csv(log, path):
    with open(path, "w") as f:
        f.write("phase,t_s,wL_rev_s,wR_rev_s,uL,uR\n")
        for r in log:
            f.write("%s,%.6f,%.5f,%.5f,%.4f,%.4f\n" % r)


def load_csv(path):
    log = []
    with open(path) as f:
        next(f)
        for ln in f:
            p = ln.strip().split(",")
            log.append((p[0],) + tuple(float(x) for x in p[1:]))
    return log


def make_plot(log, a):
    if a.no_plot:
        return
    from plots import plot_cal
    plot_cal(log, fit_static(log), fit_breakaway(log), fit_step(log), a.plot, a.show)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--port")
    r.add_argument("--baud", type=int, default=115200)
    r.add_argument("--sim", action="store_true", help="модель платы вместо реальной")
    r.add_argument("--out", default="cal_log.csv")
    r.add_argument("--yes", action="store_true", help="не спрашивать подтверждение")
    r.add_argument("--apply", action="store_true", help="отправить полученные команды на плату")
    for p in (r, sub.add_parser("fit")):
        p.add_argument("--radius", type=float, default=0.065 / 2, help="радиус колеса, м (WHEEL_RADIUS из config.h)")
        p.add_argument("--lambda-s", type=float, default=None, help="общая для обоих колёс желаемая постоянная замкнутого контура, с (например 0.2); заменяет --lam")
        p.add_argument("--lam", type=float, default=0.5, help="lambda = LAM*Tm: больше - спокойнее ПИ (по умолчанию 0.5)")
    f = sub.choices["fit"]
    f.add_argument("csv")
    for p in (r, f):
        p.add_argument("--plot", default="cal_plot.png", help="куда сохранить график (PNG)")
        p.add_argument("--show", action="store_true", help="открыть график в окне")
        p.add_argument("--no-plot", action="store_true", help="не строить график")
    a = ap.parse_args()

    if a.cmd == "fit":
        log = load_csv(a.csv)
        report(log, a.radius, a.lam, a.lambda_s)
        make_plot(log, a)
        return

    if a.sim:
        link = SimLink(a.radius)
    else:
        if not a.port:
            sys.exit("Укажите --port (или --sim для проверки без робота)")
        link = SerialLink(a.port, a.baud)
        if not a.yes:
            ui.warn("ВНИМАНИЕ: опыт прогонит моторы до 100% ШИМ. Поднимите робота так, чтобы колёса вращались в воздухе.")
            if input("Колёса подняты? Введите 'да': ").strip().lower() not in ("да", "yes", "y"):
                sys.exit("Отменено.")
    exp = Experiment(link, a.radius)
    try:
        exp.run()
    except KeyboardInterrupt:
        exp._stop()
        raise
    save_csv(exp.log, a.out)
    ui.ok("Лог сохранён: %s (%d отсчётов)" % (a.out, len(exp.log)))
    cmd_ff, cmd_pi = report(exp.log, a.radius, a.lam, a.lambda_s)
    make_plot(exp.log, a)
    if a.apply:
        link.send(cmd_ff + "\r\n")
        link.sleep(0.1)
        link.send(cmd_pi + "\r\n")
        link.sleep(0.1)
        link.send("$5;#\r\n$13;#\r\n")
        link.sleep(0.3)
        ui.panel("\n".join(ln for ln in link.poll() if ln.startswith(("$2", "$4", "$6"))) or "(плата не ответила)",
                 "Команды отправлены. Ответ платы", "green")


if __name__ == "__main__":
    main()
