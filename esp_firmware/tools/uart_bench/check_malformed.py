#!/usr/bin/env python3
"""Non-motion UART resilience smoke test. Never sends motor/config write IDs."""
import argparse
import time
from client import Client

CASES = [
    ('unknown_id', b'$999;#'),
    ('non_numeric_id', b'$abc;#'),
    ('invalid_pid_numeric', b'$202;NaN;#'),
    ('invalid_pid_huge', b'$202;99999999999999999999999999999999999999999;#'),
    ('invalid_pid_empty', b'$202;;#'),
    ('invalid_pid_unterminated', b'$202;0'),
    ('recovery_marker', b'$999;#'),
]

def sample(dev, seconds):
    stats = {'odom': 0, 'pid': 0, 'others': 0}
    start_err = dev.errors
    for frame in dev.receive(seconds):
        if frame.id == 301: stats['odom'] += 1
        elif frame.id == 202: stats['pid'] += 1
        else: stats['others'] += 1
    stats['errors'] = dev.errors - start_err
    return stats

def pid_query(dev, timeout):
    dev.send(202)
    for f in dev.receive(timeout):
        if f.id == 202 and len(f.fields) == 6:
            return f.fields
    return None

def main():
    p=argparse.ArgumentParser(description='Read-only malformed-frame recovery test; motor power must be disconnected.')
    p.add_argument('--port', required=True)
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--duration', type=float, default=2.0, help='seconds for baseline and recovery each')
    a=p.parse_args()
    if a.duration < 1: p.error('--duration must be >= 1')
    with Client(a.port,a.baud) as dev:
        time.sleep(0.3)
        baseline=sample(dev,a.duration)
        print('Baseline:',baseline)
        baseline_pid=pid_query(dev,2.0)
        print('PID before:', baseline_pid)
        if baseline['odom'] < 10 or baseline_pid is None:
            raise SystemExit('INCONCLUSIVE: baseline odometry/PID response missing; no malformed frames sent')
        for name, raw in CASES:
            dev.serial.write(raw)
            dev.serial.flush()
            print('Injected:',name)
            time.sleep(0.1)
        # A new start marker resynchronizes any incomplete input frame.
        recovered=sample(dev,a.duration)
        after_pid=pid_query(dev,2.0)
        print('Recovery:',recovered)
        print('PID after:',after_pid)
        ok = (recovered['odom'] >= 10 and after_pid == baseline_pid
              and recovered['errors'] == 0)
        print('PASS: parser survived and read-only telemetry/config recovered' if ok else
              'FAIL: UART recovery needs investigation')
        raise SystemExit(0 if ok else 1)
if __name__=='__main__':main()
