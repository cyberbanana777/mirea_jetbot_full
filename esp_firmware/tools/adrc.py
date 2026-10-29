#!/usr/bin/env python3
# Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
"""
LADRC в прошивке: расчёт параметров по калибровке, запись в плату и выбор регулятора (ПИ / LADRC).

    python3 adrc.py fit cal_log.csv                       # b0 по калибровке mtcal, wc/wo, команда $16
    python3 adrc.py fit cal_log.csv --lambda-s 0.2 --apply --port /dev/ttyUSB0
    python3 adrc.py show --port /dev/ttyUSB0              # что сейчас в плате: регулятор, LADRC, FF, ПИ
    python3 adrc.py mode ladrc --port /dev/ttyUSB0        # переключить на LADRC (и сохранить во flash)
    python3 adrc.py mode pi --port /dev/ttyUSB0
    python3 adrc.py set --port /dev/ttyUSB0 --wc 9 --wo 40              # поменять часть параметров
    python3 adrc.py set --port /dev/ttyUSB0 --b0 9.82,7.65 --ff 2

Параметры LADRC (одинаковые wc и wo для обоих колёс - одинаковая желаемая динамика):
  wc  полоса контура, рад/с. Отклик на уставку ~ 1/(s/wc + 1): lambda = 1/wc (5 рад/с ~ lambda 0.2 с).
  wo  полоса наблюдателя ESO, рад/с. Обычно 3-5 * wc. Ограничения: wo*dt <= 0.5 (такт 10 мс -> wo <= 50)
      и wo*L <= ~1.5 (L - запаздывание контура из калибровки), иначе шум и раскачка.
  b0  ускорение колеса на единицу скважности, (м/с^2)/1.0 = K/Tm, где K = Kv*2*pi*R (м/с на 1.0).
      Ошибиться лучше в большую сторону: завышенный b0 - медленнее, заниженный - риск раскачки.
  ff  прямая подача по калибровке feedforward ($11): 0 - нет (чистый LADRC), 1 - мёртвая зона,
      2 - полный статический FF. Наблюдатель оценивает только то, что калибровка не объяснила.

Важно: открытие порта перезагружает плату (DTR). Значения без сохранения во flash после этого
пропадают, поэтому `mode` и `set` по умолчанию сохраняют (--no-save - только до перезагрузки).
Для разовых опытов без записи во flash есть `steptest.py run --mode ... --adrc ...`.
"""
import argparse
import math
import re
import sys

import ui

TWO_PI = 2.0 * math.pi
DT = 0.01                 # такт контура скорости, с (CONTROL_PERIOD_US)
WO_DT_MAX = 0.5           # wo*dt: выше - дискретный наблюдатель "звенит"
WO_L_MAX = 1.5            # wo*L: выше - запаздывание измерения раскачивает контур
FF_NAMES = {0: "нет (чистый LADRC)", 1: "мёртвая зона", 2: "полный FF"}
MODE_NAMES = {0: "ПИ", 1: "LADRC"}


# ----------------------------------------------------------------------------------------------
# Расчёт
# ----------------------------------------------------------------------------------------------
def b0_from_model(kv_rev, tm, wheel_r):
    """Kv (об/с на единицу скважности), Tm (с) -> b0, (м/с^2) на единицу скважности"""
    return kv_rev * TWO_PI * wheel_r / tm


def recommend(st, sp, wheel_r, wc=None, lambda_s=None, wo=None, wo_mult=5.0, ff=2):
    """st, sp - результаты mtcal.fit_static / fit_step. -> (параметры, замечания)"""
    notes = []
    b0 = {n: b0_from_model(st[n][0], sp[n][0], wheel_r) for n in ("L", "R")}
    lag = max(sp["L"][1], sp["R"][1])
    if wc is None:
        wc = 1.0 / lambda_s if lambda_s else 5.0
    if 1.0 / wc < 2.0 * lag:
        wc_new = 1.0 / (2.0 * lag)
        notes.append("wc = %.1f рад/с быстрее, чем позволяет запаздывание L = %.0f мс (lambda >= 2L): снижено до %.1f"
                     % (wc, lag * 1000, wc_new))
        wc = wc_new
    if wo is None:
        wo = wo_mult * wc
    wo_cap = min(WO_DT_MAX / DT, WO_L_MAX / lag)
    if wo > wo_cap:
        notes.append("wo = %.0f рад/с больше предела %.0f (wo*dt <= %.1f, wo*L <= %.1f): снижено"
                     % (wo, wo_cap, WO_DT_MAX, WO_L_MAX))
        wo = wo_cap
    if wo < 3.0 * wc:
        notes.append("wo = %.0f меньше 3*wc = %.0f: наблюдатель медленный, возмущения подавляются хуже" % (wo, 3 * wc))
    p = dict(wc=wc, wo=wo, b0L=b0["L"], b0R=b0["R"], ff=int(ff))
    return p, notes


def check(p, lag=None):
    """Замечания к набору параметров (для `set` и steptest)"""
    notes = []
    if p["wo"] * DT > WO_DT_MAX:
        notes.append("wo*dt = %.2f > %.1f: дискретный наблюдатель на 100 Гц близок к раскачке" % (p["wo"] * DT, WO_DT_MAX))
    if lag and p["wo"] * lag > WO_L_MAX:
        notes.append("wo*L = %.2f > %.1f: запаздывание измерения может раскачать контур" % (p["wo"] * lag, WO_L_MAX))
    if p["wo"] < 3.0 * p["wc"]:
        notes.append("wo < 3*wc: наблюдатель медленнее, чем нужно контуру")
    return notes


def equivalent_pi(p, b0):
    """Эквивалентный ПИ + фильтр для обратной связи LADRC 1-го порядка (вывод - в разборе LADRC):
    Kp = (wo^2 + 2 wo wc) / (b0 (wc + 2 wo)), Ti = (wo + 2 wc) / (wo wc), Tf = 1/(wc + 2 wo)"""
    wc, wo = p["wc"], p["wo"]
    kp = (wo * wo + 2 * wo * wc) / (b0 * (wc + 2 * wo))
    ti = (wo + 2 * wc) / (wo * wc)
    return kp, kp / ti, 1.0 / (wc + 2 * wo)


def command(p, save=True):
    return "$16;%.3f;%.2f;%.4f;%.4f;%d;%d;#" % (p["wc"], p["wo"], p["b0L"], p["b0R"], p["ff"], 1 if save else 0)


def parse7(line):
    """$7;mode;wc;wo;b0L;b0R;ff;# -> dict или None"""
    m = re.search(r"\$7;([^#]*)#", line)
    if not m:
        return None
    f = [x for x in m.group(1).split(";") if x]
    if len(f) < 6:
        return None
    try:
        return dict(mode=int(float(f[0])), wc=float(f[1]), wo=float(f[2]), b0L=float(f[3]), b0R=float(f[4]),
                    ff=int(float(f[5])))
    except ValueError:
        return None


def describe(p):
    return ("регулятор %s; LADRC: wc %.2f рад/с (lambda %.3f с), wo %.1f рад/с, b0 L/R %.3f / %.3f, прямая подача: %s"
            % (MODE_NAMES.get(p.get("mode", -1), "?"), p["wc"], 1.0 / p["wc"], p["wo"], p["b0L"], p["b0R"],
               FF_NAMES.get(p["ff"], "?")))


def print_params(p, title="Параметры LADRC"):
    rows = []
    for n in ("L", "R"):
        b0 = p["b0" + n]
        kp, ki, tf = equivalent_pi(p, b0)
        rows.append([n, "%.2f" % p["wc"], "%.1f" % p["wo"], "%.3f" % b0, "%.3f" % kp, "%.2f" % ki, "%.1f" % (tf * 1000)])
    ui.table(title, [("колесо", "center"), ("wc", "right"), ("wo", "right"), ("b0", "right"),
                     ("Kp экв", "right"), ("Ki экв", "right"), ("Tf, мс", "right")], rows,
             caption="wc, wo - рад/с; b0 - м/с² на 1.0 скважности. lambda = 1/wc = %.3f с; wo*dt = %.2f; прямая подача: %s. "
                     "Kp/Ki экв - эквивалентный ПИ обратной связи (на м/с, как у $4), Tf - его фильтр"
             % (1.0 / p["wc"], p["wo"] * DT, FF_NAMES.get(p["ff"], "?")))


# ----------------------------------------------------------------------------------------------
# Связь с платой
# ----------------------------------------------------------------------------------------------
def query(link, cmd, prefixes, wait=0.5, tries=3):
    """Отправить cmd и собрать строки с заданными префиксами (по одной команде, с паузой)"""
    got = []
    for _ in range(tries):
        link.send(cmd + "\r\n")
        end = link.now() + wait
        while link.now() < end:
            link.sleep(0.01)
            got += [ln for ln in link.poll() if ln.startswith(prefixes)]
        if got:
            return got
    return got


def read_params(link):
    for ln in query(link, "$17;#", ("$7;",)):
        p = parse7(ln)
        if p:
            return p
    return None


def open_link(a):
    from mtcal import SerialLink
    if not a.port:
        sys.exit("Укажите --port")
    return SerialLink(a.port, a.baud, a.boot_wait)


def cmd_show(a):
    link = open_link(a)
    lines = query(link, "$13;#", ("$4;", "$6;", "$7;"))
    lines += query(link, "$5;#", ("$2;",))
    p = next((parse7(ln) for ln in lines if parse7(ln)), None)
    ui.panel("\n".join(lines) if lines else "(плата не ответила)", "Ответ платы на $13 и $5", "blue")
    if p is None:
        ui.warn("Нет ответа $7: в плате прошивка без LADRC (нужна 2.4.0)")
        return
    ui.say(describe(p), "bold")
    print_params(p)


def cmd_mode(a):
    link = open_link(a)
    m = 1 if a.which == "ladrc" else 0
    link.send("$15;%d;%d;#\r\n" % (m, 0 if a.no_save else 1))
    link.sleep(0.3)
    p = read_params(link)
    if p is None:
        sys.exit("Плата не ответила на $17: прошивка без LADRC?")
    if p["mode"] != m:
        sys.exit("Режим не переключился: плата сообщает %s" % MODE_NAMES.get(p["mode"], "?"))
    ui.ok("Регулятор: %s%s" % (MODE_NAMES[m], "" if a.no_save else " (сохранено во flash)"))
    ui.say(describe(p), "dim")


def cmd_set(a):
    link = open_link(a)
    p = read_params(link)
    if p is None:
        sys.exit("Плата не ответила на $17: прошивка без LADRC?")
    if a.lambda_s:
        p["wc"] = 1.0 / a.lambda_s
    if a.wc:
        p["wc"] = a.wc
    if a.wo_mult:
        p["wo"] = a.wo_mult * p["wc"]
    if a.wo:
        p["wo"] = a.wo
    if a.b0:
        p["b0L"], p["b0R"] = (float(x) for x in a.b0.split(","))
    if a.ff is not None:
        p["ff"] = a.ff
    for n in check(p):
        ui.warn(n)
    c = command(p, not a.no_save)
    link.send(c + "\r\n")
    link.sleep(0.3)
    q = read_params(link)
    ui.panel(c, "Отправлено", "green")
    if q is None:
        sys.exit("Плата не ответила на $17")
    ui.say(describe(q), "bold")
    print_params(q, "Параметры LADRC в плате")


def cmd_fit(a):
    from mtcal import fit_static, fit_step, load_csv
    log = load_csv(a.csv)
    st, sp = fit_static(log), fit_step(log)
    p, notes = recommend(st, sp, a.radius, a.wc, a.lambda_s, a.wo, a.wo_mult, a.ff)
    model = [[n, "%.2f" % st[n][0], "%.0f" % (sp[n][0] * 1000), "%.0f" % (sp[n][1] * 1000),
              "%.3f" % (st[n][0] * TWO_PI * a.radius)] for n in ("L", "R")]
    ui.table("Модель колёс по калибровке", [("колесо", "center"), ("Kv, об/с на 1.0", "right"), ("Tm, мс", "right"),
                                            ("L, мс", "right"), ("K, м/с на 1.0", "right")], model,
             caption="b0 = K/Tm. Калибровка в воздухе: на полу Tm больше, значит истинный b0 меньше - "
                     "воздушный b0 завышен, это безопасная сторона")
    print_params(p, "Рекомендуемые параметры LADRC")
    for n in notes:
        ui.warn(n)
    c = command(p, True)
    ui.panel(c + "\n$15;1;1;#", "Команды для платы: параметры LADRC, затем переключение на LADRC (оба с сохранением)", "green")
    ui.say("Вернуть ПИ: $15;0;1;#  (или python3 adrc.py mode pi --port ...)", "dim")
    if a.apply:
        link = open_link(a)
        link.send(c + "\r\n")
        link.sleep(0.3)
        q = read_params(link)
        ui.say(describe(q) if q else "(плата не ответила на $17)", "bold")
        ui.say("Регулятор не переключён: python3 adrc.py mode ladrc --port %s" % a.port, "dim")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit", help="параметры LADRC по логу калибровки mtcal")
    f.add_argument("csv")
    f.add_argument("--radius", type=float, default=0.065 / 2)
    f.add_argument("--apply", action="store_true", help="записать параметры в плату (с сохранением)")
    s = sub.add_parser("set", help="поменять параметры LADRC в плате")
    s.add_argument("--b0", help="b0 левого и правого колеса через запятую, (м/с^2) на 1.0")
    s.add_argument("--no-save", action="store_true", help="не сохранять во flash (пропадёт при следующем открытии порта)")
    for p in (f, s):
        p.add_argument("--wc", type=float, help="полоса контура, рад/с")
        p.add_argument("--lambda-s", type=float, help="желаемая постоянная времени, с (wc = 1/lambda)")
        p.add_argument("--wo", type=float, help="полоса наблюдателя, рад/с")
        p.add_argument("--wo-mult", type=float, default=None, help="wo = K*wc (по умолчанию в fit 5)")
        p.add_argument("--ff", type=int, choices=(0, 1, 2), default=None,
                       help="прямая подача: 0 нет, 1 мёртвая зона, 2 полный FF (в fit по умолчанию 2)")
    m = sub.add_parser("mode", help="выбрать регулятор")
    m.add_argument("which", choices=("pi", "ladrc"))
    m.add_argument("--no-save", action="store_true", help="не сохранять во flash")
    sh = sub.add_parser("show", help="показать параметры в плате")
    for p in (f, s, m, sh):
        p.add_argument("--port")
        p.add_argument("--baud", type=int, default=115200)
        p.add_argument("--boot-wait", type=float, default=5.0, help="пауза на загрузку платы после открытия порта, с")
    a = ap.parse_args()
    if a.cmd == "fit":
        a.wo_mult = a.wo_mult or 5.0
        a.ff = 2 if a.ff is None else a.ff
        cmd_fit(a)
    elif a.cmd == "set":
        cmd_set(a)
    elif a.cmd == "mode":
        cmd_mode(a)
    else:
        cmd_show(a)


if __name__ == "__main__":
    main()
