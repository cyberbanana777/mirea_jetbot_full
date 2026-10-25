# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

from web_regulators_configurator import protocol as p


def _raises(fn, *args):
    try:
        fn(*args)
    except p.ProtocolError:
        return True
    return False


PID = {'kp_l': 0.85, 'ki_l': 1.5, 'kd_l': 0.05, 'kp_r': 0.8, 'ki_r': 1.5, 'kd_r': 0.04}


def test_commands_format():
    assert p.cmd_base_speed(0.2, -0.5) == '$1;0.20000;-0.50000;#'
    assert p.cmd_wheel_speeds(0.1, 0.15) == '$2;0.10000;0.15000;#'
    assert p.cmd_set_pid(PID) == '$3;0.85000;1.50000;0.05000;0.80000;1.50000;0.04000;#'
    assert p.cmd_write_pid_flash(PID).startswith('$4;0.85000;')
    assert p.cmd_request_pid() == '$5;#'
    assert p.cmd_stop() == '$1;0.00000;0.00000;#'


def test_no_exponent_for_small_values():
    # парсер прошивки не ожидает экспоненциальную запись
    assert 'e' not in p.cmd_base_speed(1e-7, 1e-9)


def test_parse_pid_coeffs_from_bridge():
    pid = p.parse_pid_coeffs('0.85;1.5;0.05;0.8;1.5;0.04')
    assert pid == PID
    assert _raises(p.parse_pid_coeffs, '1;2;3')
    assert _raises(p.parse_pid_coeffs, '1;2;3;4;5;x')


def test_validate_pid():
    assert p.validate_pid({k: str(v) for k, v in PID.items()}) == PID
    assert _raises(p.validate_pid, {**PID, 'kp_l': -1})
    assert _raises(p.validate_pid, {**PID, 'kd_r': float('nan')})
    bad = dict(PID)
    del bad['ki_r']
    assert _raises(p.validate_pid, bad)
    assert _raises(p.validate_pid, [1, 2, 3])


def test_kinematics_roundtrip():
    wb = 0.1775
    left, right = p.wheel_setpoints(0.2, 1.0, wb)
    assert abs(left - (0.2 - 0.08875)) < 1e-12
    v, w = p.base_setpoints(left, right, wb)
    assert abs(v - 0.2) < 1e-12 and abs(w - 1.0) < 1e-12


def test_clamp_and_to_float():
    assert p.clamp(5.0, 0.4) == 0.4 and p.clamp(-5.0, 0.4) == -0.4
    assert p.to_float('0.5', 'x') == 0.5
    assert _raises(p.to_float, 'inf', 'x')
    assert _raises(p.to_float, None, 'x')


def test_parse_frame():
    kind, vals = p.parse_frame('$1;0.1;0.2;0.3;0.4;0.5;0.6;0.7;#\r\n')
    assert kind == 1 and vals == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    kind, vals = p.parse_frame('$2;0.85;1.5;0.05;0.8;1.5;0.04;#')
    assert kind == 2 and len(vals) == 6
    assert _raises(p.parse_frame, 'INA219 reinitialized')
    assert _raises(p.parse_frame, '$1;abc;#')
    assert _raises(p.parse_frame, '$#')
