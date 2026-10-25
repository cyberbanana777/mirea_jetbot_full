# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

import math

from web_regulators_configurator.step_test import StepTest, compute_metrics


def first_order(target, tau, duration, dt=0.02):
    """Отклик апериодического звена 1-го порядка на ступеньку."""
    return [(i * dt, target * (1 - math.exp(-i * dt / tau)))
            for i in range(int(duration / dt) + 1)]


def second_order(target, zeta, wn, duration, dt=0.02):
    """Отклик колебательного звена 2-го порядка (0 < zeta < 1) на ступеньку."""
    wd = wn * math.sqrt(1 - zeta ** 2)
    k = zeta / math.sqrt(1 - zeta ** 2)
    out = []
    for i in range(int(duration / dt) + 1):
        t = i * dt
        y = 1 - math.exp(-zeta * wn * t) * (math.cos(wd * t) + k * math.sin(wd * t))
        out.append((t, target * y))
    return out


def test_first_order_metrics():
    tau = 0.2
    m = compute_metrics(first_order(0.3, tau, 3.0), 0.3, 3.0)
    # для 1-го порядка время нарастания 10-90 % = tau*ln(9)
    assert abs(m['rise_time'] - tau * math.log(9)) < 0.03
    assert m['overshoot'] == 0.0
    assert abs(m['settling_time'] - tau * math.log(20)) < 0.03   # полоса 5 %
    assert abs(m['steady_error']) < 1e-3


def test_negative_target_and_overshoot():
    zeta = 0.3
    m = compute_metrics(second_order(-0.2, zeta, 10.0, 3.0), -0.2, 3.0)
    expected = 100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))   # ~37 %
    assert abs(m['overshoot'] - expected) < 1.0
    assert m['rise_time'] is not None and m['settling_time'] is not None


def test_not_settled_and_zero_target():
    samples = [(t * 0.02, 0.1) for t in range(100)]    # стоит на 50 % уставки
    m = compute_metrics(samples, 0.2, 2.0)
    assert m['settling_time'] is None and m['rise_time'] is None
    assert abs(m['steady_error_pct'] - 50.0) < 1e-9
    assert compute_metrics(samples, 0.0, 2.0) is None


def test_state_machine():
    st = StepTest(tail_duration=0.5)
    sp = {'v': 0.2, 'w': 0.0, 'vl': 0.2, 'vr': 0.2}
    st.start('base', 0.2, 0.0, 1.0, sp, now=10.0)
    assert st.tick(10.5) == 'run'
    for t, y in first_order(0.2, 0.1, 1.4):
        st.add_sample(10.0 + t, 'vl', y)
    assert st.tick(11.0) == 'stop'
    assert st.tick(11.2) is None
    assert st.tick(11.6) == 'done'
    status = st.status(11.6)
    assert status['state'] == 'done'
    assert status['result']['vl']['rise_time'] is not None
    assert status['result']['w'] is None
    csv_text = st.to_csv()
    assert csv_text.split('\n')[1] == 'time_s,signal,value,setpoint'
    assert ',vl,' in csv_text


def test_abort():
    st = StepTest()
    st.start('wheels', 0.1, 0.1, 2.0, {}, now=0.0)
    assert st.abort('СТОП') is True
    assert st.abort('again') is False
    assert st.tick(5.0) is None
    try:
        st.start('wheels', 0.1, 0.1, 2.0, {}, now=1.0)
    except RuntimeError:
        raise AssertionError('после прерывания тест можно запустить заново')
