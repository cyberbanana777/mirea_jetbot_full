# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
Потокобезопасное хранилище телеметрии и логика «мёртвой руки».

Без зависимостей от ROS. Колбэки ROS пишут сюда из потока rclpy,
HTTP-сервер читает из своих потоков.
"""

from collections import deque
import itertools
import threading

SIGNALS = ('v', 'w', 'vl', 'vr')   # скорость базы, угловая, левое и правое колесо


class Telemetry:
    """
    Кольцевые буферы измерений и лога с порядковыми номерами.

    Каждый клиент SSE помнит номер последней полученной записи и забирает
    только новые (since()), поэтому графики на странице получают все
    точки с частотой ESP32, а не прореженные.
    """

    def __init__(self, history_size=1500, log_size=500):
        self._lock = threading.Lock()
        self._seq = itertools.count(1)
        self._samples = deque(maxlen=history_size)   # (seq, t, signal, value)
        self._log = deque(maxlen=log_size)           # (seq, t, direction, text)
        self._pid = None
        self._pid_seq = 0
        self._frame_times = deque(maxlen=100)        # моменты кадров одометрии

    # ---- запись (поток ROS) ----
    def add_sample(self, t, signal, value):
        with self._lock:
            self._samples.append((next(self._seq), t, signal, value))

    def add_log(self, t, direction, text):
        """direction: 'rx' - от ESP32, 'tx' - отправлено, 'info' - событие."""
        with self._lock:
            self._log.append((next(self._seq), t, direction, text))

    def set_pid(self, pid):
        with self._lock:
            self._pid = dict(pid)
            self._pid_seq = next(self._seq)

    def mark_frame(self, t):
        with self._lock:
            self._frame_times.append(t)

    # ---- чтение (потоки HTTP) ----
    def since(self, cursor):
        """Новые записи после cursor: (новый cursor, samples, log, pid или None)."""
        with self._lock:
            samples = self._newer(self._samples, cursor)
            log = self._newer(self._log, cursor)
            pid = dict(self._pid) if self._pid is not None and self._pid_seq > cursor else None
            last = max(self._samples[-1][0] if self._samples else 0,
                       self._log[-1][0] if self._log else 0, self._pid_seq, cursor)
            return last, samples, log, pid

    @staticmethod
    def _newer(buffer, cursor):
        # Номера растут, поэтому идём с конца и останавливаемся на старых:
        # работа пропорциональна числу новых записей, а не размеру буфера.
        out = []
        for entry in reversed(buffer):
            if entry[0] <= cursor:
                break
            out.append(entry[1:])
        out.reverse()
        return out

    def current_pid(self):
        with self._lock:
            return dict(self._pid) if self._pid is not None else None

    def frame_rate(self, now, window=2.0):
        """Частота кадров одометрии от ESP32 за последние window секунд, Гц."""
        with self._lock:
            recent = [t for t in self._frame_times if now - t <= window]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    def last_frame_age(self, now):
        with self._lock:
            return now - self._frame_times[-1] if self._frame_times else None


class DeadMan:
    """
    Активная команда движения и сторож связи со страницей.

    Пока команда активна, нода повторяет её (прошивка останавливает робота,
    если команд нет дольше 1 с). Команда снимается, если страница перестала
    присылать heartbeat дольше timeout или нажат СТОП.
    """

    def __init__(self, timeout):
        self._lock = threading.Lock()
        self._timeout = timeout
        self._command = None      # (mode, a, b) или None
        self._last_beat = None

    def heartbeat(self, now):
        with self._lock:
            self._last_beat = now

    def set_command(self, mode, a, b, now):
        with self._lock:
            self._command = (mode, a, b)
            self._last_beat = now

    def clear(self):
        """Снять команду. Возвращает True, если она была активна."""
        with self._lock:
            was_active = self._command is not None
            self._command = None
            return was_active

    def command(self):
        with self._lock:
            return self._command

    def check(self, now):
        """
        Проверить сторожа; вызывать по таймеру.

        Возвращает (команда для повтора или None, True если команда только
        что снята по таймауту).
        """
        with self._lock:
            if self._command is None:
                return None, False
            if self._last_beat is None or now - self._last_beat > self._timeout:
                self._command = None
                return None, True
            return self._command, False
