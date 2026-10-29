#!/usr/bin/env python3
# Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
"""
Консоль для платы: отправить команду и увидеть ответ, не утопая в потоке одометрии.

    python3 console.py --port /dev/ttyUSB0          (Windows: --port COM5)
    python3 console.py --port /dev/ttyUSB0 --show-odom      # показывать и одометрию $1
    python3 console.py --port /dev/ttyUSB0 --log session.txt

По умолчанию скрыты одометрия $1 (50 Гц) и телеметрия $3 (100 Гц, после $12;1). Всё остальное
(ответы $2/$4/$6/$7, сообщения прошивки при загрузке и ошибки) печатается с отметкой времени,
а известные ответы ещё и расшифровываются по полям.

Команды платы вводятся как есть: $13;#   $5;#   $15;1;0#
'#' в конце можно не писать - консоль допишет. Свои команды консоли:
  hold <команда>   повторять команду раз в 0.3 с (иначе через 1 с без команд плата делает
                   аварийную остановку), например: hold $2;0.1;0.1;#
  stop             прекратить hold и отправить $2;0;0;#
  odom / tel       включить или выключить показ $1 / $3
  last             показать последнюю строку одометрии (даже если она скрыта)
  q, quit, exit    выход (перед выходом отправляется $2;0;0;#)
  help             эта справка
Нужен pyserial: pip install pyserial
"""
import argparse
import sys
import threading
import time

try:
    import readline  # noqa: F401  история команд стрелками (Linux/macOS)
    HAVE_READLINE = True
except ImportError:
    HAVE_READLINE = False

PROMPT = "> "
HOLD_PERIOD = 0.3

# Расшифровка ответов платы: тип -> (название, поля)
FIELDS = {
    "1": ("одометрия", ["x", "y", "курс", "v", "w", "vL", "vR"]),
    "2": ("ПИ", ["Kp_L", "Ki_L", "Kd_L", "Kp_R", "Ki_R", "Kd_R"]),
    "3": ("телеметрия", ["t_us", "refL", "vL", "uL", "refR", "vR", "uR"]),
    "4": ("feedforward", ["gain_L", "dz_L", "gain_R", "dz_R"]),
    "6": ("фильтр уставки", ["tau, с"]),
    "7": ("регулятор", ["режим", "wc", "wo", "b0_L", "b0_R", "ff"]),
}
MODE = {"0": "ПИ", "1": "LADRC"}
FF = {"0": "нет", "1": "мёртвая зона", "2": "полный FF"}

USE_COLOR = sys.stdout.isatty()
if USE_COLOR and sys.platform == "win32":
    import os
    os.system("")   # включает ANSI-цвета в консоли Windows 10+


def color(text, code):
    return "\033[%sm%s\033[0m" % (code, text) if USE_COLOR else text


def decode(line):
    """'$4;0.5;0.3;0.8;0.31;#' -> 'feedforward: gain_L=0.5 ...' или None"""
    if not line.startswith("$") or ";" not in line:
        return None
    body = line[1:].split("#", 1)[0]
    parts = [p for p in body.split(";") if p != ""]
    if not parts or parts[0] not in FIELDS:
        return None
    name, names = FIELDS[parts[0]]
    vals = parts[1:]
    pairs = []
    for i, v in enumerate(vals):
        key = names[i] if i < len(names) else "?%d" % i
        if parts[0] == "7" and key == "режим":
            v = MODE.get(v, v)
        if parts[0] == "7" and key == "ff":
            v = FF.get(v, v)
        pairs.append("%s=%s" % (key, v))
    return "%s: %s" % (name, "  ".join(pairs))


class Console:
    def __init__(self, ser, show_odom=False, show_tel=False, log=None):
        self.ser = ser
        self.show = {"1": show_odom, "3": show_tel}
        self.log = log
        self.lock = threading.Lock()
        self.alive = True
        self.hold_cmd = None
        self.hidden = {"1": 0, "3": 0}
        self.last_odom = ""
        self.t0 = time.monotonic()

    # ---------- вывод, не ломающий строку ввода
    def out(self, text):
        with self.lock:
            buf = ""
            if HAVE_READLINE and sys.stdin.isatty():
                import readline as rl
                buf = rl.get_line_buffer()
                sys.stdout.write("\r\033[K")
            sys.stdout.write(text + "\n")
            if HAVE_READLINE and sys.stdin.isatty():
                sys.stdout.write(PROMPT + buf)
            sys.stdout.flush()
            if self.log:
                self.log.write(text + "\n")
                self.log.flush()

    def stamp(self):
        return "%7.2f" % (time.monotonic() - self.t0)

    # ---------- приём
    def reader(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:  # порт пропал (кабель, перезагрузка)
                self.out(color("порт: %s" % e, "31"))
                self.alive = False
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line:
                    self.handle(line)

    def handle(self, line):
        typ = line[1:].split(";", 1)[0] if line.startswith("$") else None
        if typ == "1":
            self.last_odom = line
        if typ in self.show and not self.show[typ]:
            self.hidden[typ] += 1
            return
        dec = decode(line)
        text = "%s  <  %s" % (self.stamp(), line)
        if dec:
            text += "\n" + " " * 12 + color(dec, "36")
        elif not line.startswith("$"):
            text = "%s  <  %s" % (self.stamp(), color(line, "33"))   # текст прошивки: загрузка, ошибки
        self.out(text)

    # ---------- отправка
    def send(self, cmd, echo=True):
        if not cmd.endswith("#"):
            cmd += "#"
        try:
            self.ser.write((cmd + "\r\n").encode("ascii"))
        except Exception as e:
            self.out(color("не отправлено: %s" % e, "31"))
            return
        if echo:
            self.out("%s  >  %s" % (self.stamp(), color(cmd, "1")))

    def holder(self):
        while self.alive:
            time.sleep(HOLD_PERIOD)
            cmd = self.hold_cmd
            if cmd:
                self.send(cmd, echo=False)

    # ---------- ввод
    def command(self, line):
        s = line.strip()
        if not s:
            return True
        low = s.lower()
        if low in ("q", "quit", "exit"):
            return False
        if low == "help":
            self.out(__doc__)
        elif low in ("odom", "tel"):
            k = "1" if low == "odom" else "3"
            self.show[k] = not self.show[k]
            self.out("показ %s: %s" % ("$1" if k == "1" else "$3", "вкл" if self.show[k] else "выкл"))
        elif low == "last":
            self.out(self.last_odom or "(одометрии ещё не было)")
            if self.last_odom:
                self.out(" " * 12 + color(decode(self.last_odom) or "", "36"))
        elif low == "stop":
            self.hold_cmd = None
            self.send("$2;0;0;#")
        elif low.startswith("hold"):
            cmd = s[4:].strip()
            if not cmd.startswith("$"):
                self.out("пример: hold $2;0.1;0.1;#")
            else:
                self.hold_cmd = cmd if cmd.endswith("#") else cmd + "#"
                self.send(self.hold_cmd)
                self.out("повторяю раз в %.1f с; stop - остановить" % HOLD_PERIOD)
        elif s.startswith("$"):
            self.send(s)
        else:
            self.out("не понял: команды платы начинаются с '$' (например $13;#), help - справка")
        return True

    def status_hidden(self):
        n1, n3 = self.hidden["1"], self.hidden["3"]
        self.hidden = {"1": 0, "3": 0}
        return "скрыто: одометрия $1 - %d, телеметрия $3 - %d" % (n1, n3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="например /dev/ttyUSB0 или COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--show-odom", action="store_true", help="показывать одометрию $1")
    ap.add_argument("--show-tel", action="store_true", help="показывать телеметрию $3")
    ap.add_argument("--log", help="дописывать весь показанный вывод в файл")
    a = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("Нужен pyserial: pip install pyserial")
    try:
        ser = serial.serial_for_url(a.port, a.baud, timeout=0.1)
    except Exception as e:
        sys.exit("Не открыть %s: %s (порт занят Arduino IDE / ROS?)" % (a.port, e))

    log = open(a.log, "a", encoding="utf-8") if a.log else None
    con = Console(ser, a.show_odom, a.show_tel, log)
    threading.Thread(target=con.reader, daemon=True).start()
    threading.Thread(target=con.holder, daemon=True).start()
    con.out(color("Порт %s открыт. Плата перезагружается (3-5 с), сообщения загрузки будут видны. "
                  "help - справка, q - выход." % a.port, "2"))
    try:
        while con.alive:
            try:
                line = input(PROMPT if sys.stdin.isatty() else "")
            except EOFError:
                break
            if not con.command(line):
                break
            if not sys.stdin.isatty():
                time.sleep(0.4)   # команды из файла/канала: пауза, чтобы плата не теряла их встык
    except KeyboardInterrupt:
        pass
    finally:
        con.hold_cmd = None
        if con.alive:
            con.send("$2;0;0;#", echo=False)
            time.sleep(0.2)
        con.alive = False
        con.out(con.status_hidden())
        ser.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()
