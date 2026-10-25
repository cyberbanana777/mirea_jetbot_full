# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

from web_regulators_configurator.telemetry import DeadMan, Telemetry


def test_since_returns_only_new_records():
    tel = Telemetry()
    tel.add_sample(1.0, 'v', 0.1)
    tel.add_log(1.0, 'tx', '$5;#')
    cur, samples, log, pid = tel.since(0)
    assert samples == [(1.0, 'v', 0.1)] and log == [(1.0, 'tx', '$5;#')] and pid is None
    tel.add_sample(2.0, 'vl', 0.2)
    tel.set_pid({'kp_l': 1.0})
    cur2, samples, log, pid = tel.since(cur)
    assert samples == [(2.0, 'vl', 0.2)] and log == [] and pid == {'kp_l': 1.0}
    cur3, samples, log, pid = tel.since(cur2)
    assert cur3 == cur2 and samples == [] and pid is None


def test_frame_rate():
    tel = Telemetry()
    for i in range(101):
        tel.mark_frame(10.0 + i * 0.02)   # 50 Гц
    assert abs(tel.frame_rate(12.0) - 50.0) < 0.5
    assert abs(tel.last_frame_age(12.5) - 0.5) < 1e-9
    assert Telemetry().frame_rate(1.0) == 0.0


def test_deadman_repeats_until_timeout():
    dm = DeadMan(timeout=1.0)
    assert dm.check(0.0) == (None, False)
    dm.set_command('base', 0.2, 0.0, now=0.0)
    assert dm.check(0.5) == (('base', 0.2, 0.0), False)
    dm.heartbeat(0.9)
    assert dm.check(1.8)[0] is not None      # heartbeat продлил
    assert dm.check(2.0) == (None, True)      # 1.1 с без heartbeat -> снята
    assert dm.check(2.1) == (None, False)


def test_deadman_clear():
    dm = DeadMan(timeout=1.0)
    dm.set_command('wheels', 0.1, 0.1, now=0.0)
    assert dm.clear() is True
    assert dm.clear() is False
    assert dm.check(0.1) == (None, False)
