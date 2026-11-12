#!/usr/bin/env python3
"""One-port, non-motion UART bench suite. Never sends nonzero motion or NVS writes."""
import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

from client import Client

# Only permitted outbound IDs: STOP and read-only PID request.
ALLOWED_TX_IDS = {103, 202, 211, 402, 405, 501}

# Expected answer to $501;# — the wire contract version of the flashed firmware.
# Must match protocol_wrapper.cpp; tools/ci/guard.py checks that both agree.
EXPECTED_PROTOCOL_VERSION = (2.0, 3.0)
MALFORMED = [
    ('unknown_id', b'$999;#'),
    ('non_numeric_id', b'$abc;#'),
    ('invalid_pid_numeric', b'$202;NaN;#'),
    ('invalid_pid_huge', b'$202;99999999999999999999999999999999999;#'),
    ('invalid_pid_empty', b'$202;;#'),
    ('invalid_pid_unterminated', b'$202;0'),
    ('resync', b'$999;#'),
]


def send(dev, msg_id):
    if msg_id not in ALLOWED_TX_IDS:
        raise ValueError('forbidden outbound ID')
    dev.send(msg_id)


def capture(dev, seconds, rows, phase):
    times = []
    before_errors = dev.errors
    before_overflow = dev.framer.overflow
    for frame in dev.receive(seconds):
        now = time.time()
        if frame.id == 301:
            times.append(time.monotonic())
            rows.append((datetime.fromtimestamp(now, timezone.utc).isoformat(), phase, frame.id, *frame.fields))
    duration = times[-1] - times[0] if len(times) > 1 else None
    rate = (len(times) - 1) / duration if duration and duration > 0 else None
    gaps = [b - a for a, b in zip(times, times[1:])]
    return {
        'frames': len(times), 'rate_hz': round(rate, 3) if rate is not None else None,
        'max_host_gap_ms': round(max(gaps) * 1000, 2) if gaps else None,
        'decode_errors': dev.errors - before_errors,
        'overflow': dev.framer.overflow - before_overflow,
    }


def pid_query(dev, timeout):
    send(dev, 202)
    for frame in dev.receive(timeout):
        if frame.id == 202 and len(frame.fields) == 6:
            return list(frame.fields)
    return None


def query_frame(dev, request_id, expected_fields, timeout=2.0):
    send(dev, request_id)
    for frame in dev.receive(timeout):
        if frame.id == request_id and len(frame.fields) == expected_fields:
            return list(frame.fields)
    return None


def transactional_checks(dev, pid_before):
    # New read is sequence-correlated; no coefficients are changed.
    seq = 314
    dev.send(211, seq)
    response = None
    for frame in dev.receive(2.0):
        if frame.id == 211 and len(frame.fields) == 7 and frame.fields[0] == seq:
            response = list(frame.fields[1:])
            break
    # Invalid values only: this command must return NACK without altering PID.
    # Literal 210 is intentionally NOT permitted through the general send() helper.
    dev.serial.write(b'$210;315;-1;5;0;1;10;0;#\n')
    dev.serial.flush()
    nack = False
    for frame in dev.receive(2.0):
        if frame.id == 503 and list(frame.fields) == [315.0, 210.0, 1.0]:
            nack = True
            break
    after = pid_query(dev, 2.0)
    return {'status': 'PASS' if response == pid_before and nack and after == pid_before else 'FAIL',
            'correlated_pid': response, 'nack_invalid_pid': nack, 'pid_unchanged': after == pid_before}


def run_suite(dev, duration, rows, malformed=True):
    results = {}
    info = query_frame(dev, 501, 2)
    results['UART-INFO-001'] = {'status': 'PASS' if info == list(EXPECTED_PROTOCOL_VERSION) else 'FAIL', 'version': info}
    snapshot_before = query_frame(dev, 402, 7)
    results['UART-DIAG-001'] = {'status': 'PASS' if snapshot_before is not None and all(math.isfinite(x) for x in snapshot_before) else 'FAIL', 'snapshot': snapshot_before, 'fields': ['uptime_ms', 'accepted', 'rejected', 'watchdog_stops', 'stopped', 'ina_ok', 'pwm_ok']}
    i2c = query_frame(dev, 405, 12)
    ok_i2c = i2c is not None and all(
        math.isfinite(x) and x.is_integer() and x >= 0 for x in i2c
    ) and all(i2c[j] <= 3 for j in (0, 4, 8)) and all(
        i2c[j] <= 4294967295 for j in range(12) if j not in (0, 4, 8)
    ) and all(i2c[j+3] <= i2c[j+2] for j in (0, 4, 8))
    results['UART-I2C-DIAG-001'] = {
        'status': 'PASS' if ok_i2c else 'FAIL', 'snapshot': i2c,
        'device_order': ['INA219', 'PCA9685', 'OLED'],
        'fields_per_device': ['state', 'probe_failures', 'recovery_attempts', 'successful_recoveries']
    }
    baseline = capture(dev, duration, rows, 'baseline')
    results['UART-ODOM-001'] = {'status': 'PASS' if baseline['frames'] >= 10 and baseline['decode_errors'] == 0 and baseline['overflow'] == 0 else 'FAIL', 'metrics': baseline}
    pid_before = pid_query(dev, 2.0)
    results['UART-CONFIG-001'] = {'status': 'PASS' if pid_before is not None and all(math.isfinite(x) for x in pid_before) else 'FAIL', 'pid': pid_before}
    if pid_before is not None:
        results['UART-TXCONFIG-001'] = transactional_checks(dev, pid_before)
    else:
        results['UART-TXCONFIG-001'] = {'status': 'SKIP', 'reason': 'no PID baseline'}
    if results['UART-ODOM-001']['status'] != 'PASS' or results['UART-CONFIG-001']['status'] != 'PASS':
        results['UART-PARSER-001'] = {'status': 'SKIP', 'reason': 'baseline failed'}
        results['MOTOR-SAFETY-001-UART'] = {'status': 'SKIP', 'reason': 'baseline failed'}
        return results
    if malformed:
        for _, raw in MALFORMED:
            dev.serial.write(raw)
            dev.serial.flush()
            time.sleep(0.08)
        recovered = capture(dev, duration, rows, 'after_malformed')
        pid_after = pid_query(dev, 2.0)
        ok = recovered['frames'] >= 10 and recovered['decode_errors'] == 0 and recovered['overflow'] == 0 and pid_after == pid_before
        results['UART-PARSER-001'] = {'status': 'PASS' if ok else 'FAIL', 'metrics': recovered, 'pid_unchanged': pid_after == pid_before}
    else:
        results['UART-PARSER-001'] = {'status': 'SKIP', 'reason': '--no-malformed'}
    send(dev, 103)
    after_stop = capture(dev, duration, rows, 'after_stop')
    send(dev, 103)
    stop_pid = pid_query(dev, 2.0)
    ok = after_stop['frames'] >= 10 and after_stop['decode_errors'] == 0 and after_stop['overflow'] == 0 and stop_pid == pid_before
    results['MOTOR-SAFETY-001-UART'] = {'status': 'PASS' if ok else 'FAIL', 'metrics': after_stop, 'pid_unchanged': stop_pid == pid_before,
                                      'note': 'Does NOT verify physical stop or PCA9685 output'}
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--duration', type=float, default=2.5, help='seconds per receive phase, minimum 1')
    p.add_argument('--output', default='reports')
    p.add_argument('--no-malformed', action='store_true')
    a = p.parse_args()
    if a.duration < 1: p.error('--duration must be >= 1')
    output = Path(a.output)
    output.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    rows = []
    print('SAFE UART suite: no motion commands, no NVS writes. STOP, read-only queries, and invalid PID NACK probe only.')
    try:
        with Client(a.port, a.baud) as dev:
            time.sleep(0.3)
            results = run_suite(dev, a.duration, rows, not a.no_malformed)
    except (KeyboardInterrupt, Exception) as exc:
        # Do not claim STOP succeeded if the port/controller failed.
        print(f'ABORTED: {type(exc).__name__}: {exc}')
        raise SystemExit(2) from exc
    report = {'timestamp_utc': timestamp, 'port': a.port, 'baud': a.baud, 'duration_per_phase_s': a.duration,
              'results': results, 'limitations': ['No physical PWM measurement', 'Host receive timestamps do not measure ESP32 scheduler jitter', 'No NVS writing or motor movement']}
    json_path = output / f'bench_{timestamp}.json'
    csv_path = output / f'odom_{timestamp}.csv'
    md_path = output / f'bench_{timestamp}.md'
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    with csv_path.open('w', newline='', encoding='utf-8') as file:
        w = csv.writer(file)
        w.writerow(['utc_time', 'phase', 'id', 'x_m', 'y_m', 'heading_rad', 'v_m_s', 'omega_rad_s', 'left_m_s', 'right_m_s'])
        w.writerows(rows)
    md_lines = [f'# UART hardware test — {timestamp}', '', f'Port: `{a.port}`; baud: {a.baud}.', '',
                '| Test | Status |', '|---|---|']
    for name, item in results.items():
        print(f'{name}: {item["status"]}', item.get('metrics', item.get('pid', '')))
        md_lines.append(f'| {name} | {item["status"]} |')
    md_lines.extend(['', 'This report tests software UART communication only; it does not verify physical motor stop, PWM output, or NVS persistence.',
                     f'', f'Raw data: `{csv_path.name}`. Machine-readable results: `{json_path.name}`.'])
    md_path.write_text('\n'.join(md_lines) + '\n', encoding='utf-8')
    print(f'Reports: {json_path}, {csv_path}, {md_path}')
    raise SystemExit(1 if any(x['status'] == 'FAIL' for x in results.values()) else 0)


if __name__ == '__main__':
    main()
