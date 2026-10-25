# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
Ступенчатый тест ПИД-регуляторов.

Подать постоянную уставку на заданное время, записать отклик и посчитать
показатели качества. Без зависимостей от ROS.

Показатели для каждого сигнала с ненулевой уставкой (по отрезку ступеньки
[0, duration]):
- rise_time    - время нарастания от 10 % до 90 % уставки, с;
- overshoot    - перерегулирование, % от уставки (0, если не было);
- settling_time- время, после которого отклик остаётся в пределах ±5 %
                 уставки до конца ступеньки, с (None - не установился);
- steady_value - среднее значение на последних 25 % ступеньки;
- steady_error - установившаяся ошибка: уставка - steady_value, в единицах
                 сигнала и в % от уставки.

Скорости приходят из прошивки уже сглаженными (скользящее среднее по 5
периодам ПИД = 0.5 с) - это те же значения, по которым работает ПИД.
Поэтому время нарастания включает задержку этого фильтра.
"""

import csv
import io
import threading

from .telemetry import SIGNALS

SETTLING_BAND = 0.05      # ±5 %
STEADY_FRACTION = 0.25    # последние 25 % ступеньки
MIN_TARGET = 1e-6


def compute_metrics(samples, target, duration):
    """
    Посчитать показатели качества отклика на ступеньку.

    samples: список (t, value), t - от начала ступеньки, с.
    Возвращает dict показателей или None, если уставка нулевая.
    """
    if abs(target) < MIN_TARGET:
        return None
    step = sorted((t, v) for t, v in samples if 0.0 <= t <= duration)
    if len(step) < 3:
        return {'target': target, 'samples': len(step), 'error': 'мало данных'}

    norm = [(t, v / target) for t, v in step]   # 1.0 = уставка (и для отрицательной)

    def first_time(level):
        for t, y in norm:
            if y >= level:
                return t
        return None

    t10, t90 = first_time(0.1), first_time(0.9)
    rise_time = t90 - t10 if t10 is not None and t90 is not None else None

    peak = max(y for _, y in norm)
    overshoot = max(0.0, (peak - 1.0) * 100.0)

    # Идём с конца: время установления - момент первой точки после последнего
    # выхода из полосы ±5 %. Если и последняя точка вне полосы - не установился.
    settling_time = None
    for i in range(len(norm) - 1, -1, -1):
        if abs(norm[i][1] - 1.0) > SETTLING_BAND:
            settling_time = norm[i + 1][0] if i + 1 < len(norm) else None
            break
    else:
        settling_time = norm[0][0]

    steady_from = duration * (1.0 - STEADY_FRACTION)
    tail = [v for t, v in step if t >= steady_from]
    if len(tail) < 3:
        tail = [v for _, v in step[-3:]]
    steady_value = sum(tail) / len(tail)
    steady_error = target - steady_value

    return {
        'target': target,
        'samples': len(step),
        'rise_time': rise_time,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'steady_value': steady_value,
        'steady_error': steady_error,
        'steady_error_pct': steady_error / target * 100.0,
    }


class StepTest:
    """
    Конечный автомат ступенчатого теста.

    idle -> running (ступенька) -> tail (запись торможения после остановки)
    -> done. Прервать можно в любой момент (aborted).
    """

    def __init__(self, tail_duration=1.0):
        self._lock = threading.Lock()
        self._tail = tail_duration
        self._reset()

    def _reset(self):
        self.state = 'idle'
        self.mode = None
        self.a = self.b = 0.0
        self.duration = 0.0
        self.t0 = None
        self.setpoints = {}
        self.samples = {s: [] for s in SIGNALS}
        self.result = None
        self.reason = None

    def start(self, mode, a, b, duration, setpoints, now):
        """setpoints: уставки сигналов SIGNALS во время ступеньки."""
        with self._lock:
            if self.state in ('running', 'tail'):
                raise RuntimeError('тест уже идёт')
            self._reset()
            self.state = 'running'
            self.mode, self.a, self.b = mode, a, b
            self.duration = duration
            self.setpoints = dict(setpoints)
            self.t0 = now

    def active(self):
        with self._lock:
            return self.state in ('running', 'tail')

    def running(self):
        with self._lock:
            return self.state == 'running'

    def add_sample(self, t, signal, value):
        with self._lock:
            if self.state in ('running', 'tail') and signal in self.samples:
                self.samples[signal].append((t - self.t0, value))

    def abort(self, reason):
        """Прервать тест (СТОП, потеря связи). Возвращает True, если он шёл."""
        with self._lock:
            if self.state not in ('running', 'tail'):
                return False
            self.state = 'aborted'
            self.reason = reason
            return True

    def tick(self, now):
        """
        Продвинуть тест по времени; вызывать по таймеру.

        Возвращает:
        'run'  - ступенька идёт, повторять команду;
        'stop' - ступенька закончилась, нужно остановить робота;
        'done' - запись завершена, результат посчитан;
        None   - тест не идёт.
        """
        with self._lock:
            if self.state == 'running':
                if now - self.t0 >= self.duration:
                    self.state = 'tail'
                    return 'stop'
                return 'run'
            if self.state == 'tail':
                if now - self.t0 >= self.duration + self._tail:
                    self.result = {
                        s: compute_metrics(self.samples[s], self.setpoints.get(s, 0.0),
                                           self.duration)
                        for s in SIGNALS}
                    self.state = 'done'
                    return 'done'
                return None
            return None

    def status(self, now):
        with self._lock:
            elapsed = now - self.t0 if self.t0 is not None else 0.0
            return {
                'state': self.state,
                'mode': self.mode,
                'a': self.a,
                'b': self.b,
                'duration': self.duration,
                'elapsed': elapsed,
                'setpoints': self.setpoints,
                'result': self.result,
                'reason': self.reason,
            }

    def recording(self):
        """Записанный отклик для графика на странице."""
        with self._lock:
            return {s: list(v) for s, v in self.samples.items()}

    def to_csv(self):
        """CSV: time_s, signal, value, setpoint (уставка 0 после ступеньки)."""
        with self._lock:
            out = io.StringIO()
            writer = csv.writer(out, lineterminator='\n')
            writer.writerow(['# mode', self.mode, 'a', self.a, 'b', self.b,
                             'duration_s', self.duration])
            writer.writerow(['time_s', 'signal', 'value', 'setpoint'])
            for signal in SIGNALS:
                sp = self.setpoints.get(signal, 0.0)
                for t, v in self.samples[signal]:
                    writer.writerow([f'{t:.4f}', signal, f'{v:.5f}',
                                     f'{sp if t <= self.duration else 0.0:.5f}'])
            return out.getvalue()
