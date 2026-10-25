# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
Протокол ESP32 jetbot_mirea: формирование команд и разбор ответов.

Без зависимостей от ROS, чтобы проверять тестами.

Команды (Jetson -> ESP32):
    $1;v;w;#                          скорость базы, м/с и рад/с
    $2;v_left;v_right;#               скорости колёс, м/с
    $3;KpL;KiL;KdL;KpR;KiR;KdR;#      ПИД только в ОЗУ
    $4;KpL;KiL;KdL;KpR;KiR;KdR;#      ПИД с записью во flash
    $5;#                              запрос текущих коэффициентов ПИД
"""

import math

PID_KEYS = ('kp_l', 'ki_l', 'kd_l', 'kp_r', 'ki_r', 'kd_r')


class ProtocolError(ValueError):
    """Неверные данные для команды или ответа."""


def to_float(value, name):
    """Число из JSON/строки; NaN и бесконечность не принимаются."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ProtocolError(f'{name}: не число ({value!r})')
    if not math.isfinite(result):
        raise ProtocolError(f'{name}: должно быть конечным числом')
    return result


def clamp(value, limit):
    """Ограничить значение диапазоном [-limit, limit]."""
    return max(-limit, min(limit, value))


def _fmt(value):
    # 5 знаков, как в ответах ESP32; без экспоненциальной записи:
    # парсер прошивки (atof по полю до 15 символов) её не ожидает.
    return f'{value:.5f}'


def cmd_base_speed(linear, angular):
    return f'$1;{_fmt(linear)};{_fmt(angular)};#'


def cmd_wheel_speeds(left, right):
    return f'$2;{_fmt(left)};{_fmt(right)};#'


def _pid_fields(pid):
    return ';'.join(_fmt(to_float(pid[k], k)) for k in PID_KEYS)


def cmd_set_pid(pid):
    return f'$3;{_pid_fields(pid)};#'


def cmd_write_pid_flash(pid):
    return f'$4;{_pid_fields(pid)};#'


def cmd_request_pid():
    return '$5;#'


def cmd_stop():
    return cmd_base_speed(0.0, 0.0)


def parse_frame(text):
    """
    Разобрать кадр ESP32 '$тип;v1;...;vN;#'.

    Возвращает (тип, [значения]). Пустые поля (после последнего ';')
    пропускаются. Бросает ProtocolError при неверном кадре.
    """
    text = text.strip()
    if len(text) < 3 or text[0] != '$' or text[-1] != '#':
        raise ProtocolError(f'не кадр $...#: {text!r}')
    fields = [f for f in text[1:-1].split(';') if f]
    if not fields:
        raise ProtocolError('пустой кадр')
    try:
        kind = int(fields[0])
        values = [float(f) for f in fields[1:]]
    except ValueError:
        raise ProtocolError(f'нечисловое поле: {text!r}')
    return kind, values


def parse_pid_coeffs(text):
    """
    Разбор /PIDcoeffs от serial_bridge_cpp: "KpL;KiL;KdL;KpR;KiR;KdR".

    Возвращает dict с ключами PID_KEYS.
    """
    parts = [p for p in text.strip().split(';') if p.strip()]
    if len(parts) != len(PID_KEYS):
        raise ProtocolError(f'ожидалось 6 коэффициентов, получено {len(parts)}')
    return {k: to_float(v, k) for k, v in zip(PID_KEYS, parts)}


def validate_pid(pid):
    """Проверить коэффициенты из запроса страницы: 6 конечных чисел >= 0."""
    if not isinstance(pid, dict):
        raise ProtocolError('коэффициенты ПИД: ожидался объект')
    result = {}
    for k in PID_KEYS:
        if k not in pid:
            raise ProtocolError(f'нет коэффициента {k}')
        value = to_float(pid[k], k)
        if value < 0.0:
            raise ProtocolError(f'{k}: коэффициент не может быть отрицательным')
        result[k] = value
    return result


def wheel_setpoints(linear, angular, wheel_base):
    """Уставки колёс по скорости базы (как в MotorController::setTargetVelocity)."""
    return (linear - angular * wheel_base / 2.0,
            linear + angular * wheel_base / 2.0)


def base_setpoints(left, right, wheel_base):
    """Скорость базы по скоростям колёс."""
    return ((left + right) / 2.0, (right - left) / wheel_base)
