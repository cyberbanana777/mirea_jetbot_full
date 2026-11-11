# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
Протокол ESP32 jetbot_mirea (v2): формирование команд и разбор ответов.

Без зависимостей от ROS, чтобы проверять тестами.

Команды (Jetson -> ESP32):
    $101;v;w;#                        скорость базы, м/с и рад/с
    $102;v_left;v_right;#             скорости колёс, м/с
    $103;#                            остановка
    $201;KpL;KiL;KdL;KpR;KiR;KdR;#    ПИД только в ОЗУ
    $202;#                            запрос текущих коэффициентов ПИД
    $204;KpL;KiL;KdL;KpR;KiR;KdR;#    ПИД с записью во flash

Ответы (ESP32 -> Jetson):
    $301;x;y;theta;v;w;v_left;v_right;#   одометрия
    $202;KpL;KiL;KdL;KpR;KiR;KdR;#         коэффициенты ПИД
    $401;t;ref_L;vel_L;duty_L;ref_R;vel_R;duty_R;#  телеметрия контура

Полный контракт - в esp_firmware/PROTOCOL.md.
"""

import math

PID_KEYS = ('kp_l', 'ki_l', 'kd_l', 'kp_r', 'ki_r', 'kd_r')

# --- идентификаторы протокола v2 -----------------------------------------
CMD_SET_TWIST = 101        # $101;linear;angular;#  скорость базы
CMD_SET_WHEEL_SPEEDS = 102  # $102;left;right;#      скорости колёс
CMD_STOP = 103             # $103;#                 остановка
CMD_SET_PID = 201          # $201;6xPID;#           ПИД только в ОЗУ
CMD_REQUEST_PID = 202      # $202;#                 запрос ПИД
CMD_SET_PID_FLASH = 204    # $204;6xPID;#           ПИД с записью во flash

MSG_ODOMETRY = 301         # $301;x;y;theta;v;w;v_left;v_right;#
MSG_PID = 202              # $202;6xPID;#  (тот же ID, что и запрос)
MSG_TELEMETRY = 401        # $401;t;ref_L;vel_L;duty_L;ref_R;vel_R;duty_R;#

# Верхняя граница коэффициента ПИД: та же, что в прошивке
# (CommandPolicy::validPid в esp_firmware/esp32_firmware/command_policy.h).
PID_MAX = 1000.0

# Ожидаемое число полей (вместе с ID) для ответов, которые разбирает узел.
ODOMETRY_VALUES = 7        # x, y, theta, v, w, v_left, v_right
PID_VALUES = 6             # KpL, KiL, KdL, KpR, KiR, KdR
TELEMETRY_VALUES = 7       # t, ref_L, vel_L, duty_L, ref_R, vel_R, duty_R


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
    return f'${CMD_SET_TWIST};{_fmt(linear)};{_fmt(angular)};#'


def cmd_wheel_speeds(left, right):
    return f'${CMD_SET_WHEEL_SPEEDS};{_fmt(left)};{_fmt(right)};#'


def _pid_fields(pid):
    return ';'.join(_fmt(to_float(pid[k], k)) for k in PID_KEYS)


def cmd_set_pid(pid):
    return f'${CMD_SET_PID};{_pid_fields(pid)};#'


def cmd_write_pid_flash(pid):
    return f'${CMD_SET_PID_FLASH};{_pid_fields(pid)};#'


def cmd_request_pid():
    return f'${CMD_REQUEST_PID};#'


def cmd_stop():
    """
    Остановка.

    В протоколе v2 это отдельная команда $103, а не нулевая скорость:
    прошивка обнуляет цель и интеграторы ПИД сразу, а не «едет к нулю».
    Команда не продлевает watchdog прошивки, поэтому робот остановится и
    при потере связи.
    """
    return f'${CMD_STOP};#'


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
    """
    Проверить коэффициенты из запроса страницы.

    Диапазон 0..1000 - тот же, что проверяет прошивка
    (CommandPolicy::validPid). Без этой проверки коэффициент больше 1000
    был бы молча отброшен прошивкой: подтверждения записи (ACK/NACK) у $201
    и $204 нет, и страница показала бы старые значения.
    """
    if not isinstance(pid, dict):
        raise ProtocolError('коэффициенты ПИД: ожидался объект')
    result = {}
    for k in PID_KEYS:
        if k not in pid:
            raise ProtocolError(f'нет коэффициента {k}')
        value = to_float(pid[k], k)
        if value < 0.0:
            raise ProtocolError(f'{k}: коэффициент не может быть отрицательным')
        if value > PID_MAX:
            raise ProtocolError(f'{k}: коэффициент больше {PID_MAX:g}')
        result[k] = value
    return result


def wheel_setpoints(linear, angular, wheel_base):
    """Уставки колёс по скорости базы (как в MotorController::setTargetVelocity)."""
    return (linear - angular * wheel_base / 2.0,
            linear + angular * wheel_base / 2.0)


def base_setpoints(left, right, wheel_base):
    """Скорость базы по скоростям колёс."""
    return ((left + right) / 2.0, (right - left) / wheel_base)
