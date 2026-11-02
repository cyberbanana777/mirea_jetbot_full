#!/usr/bin/env python3
"""Non-motion smoke test for STOP: no motor commands, does not verify physical PWM."""
import argparse
from client import Client

p=argparse.ArgumentParser(description='Send STOP and verify UART telemetry continues; disconnect motor power.')
p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
p.add_argument('--duration',type=float,default=2.0)
a=p.parse_args()
if a.duration < 1: p.error('--duration must be >= 1')
with Client(a.port,a.baud) as dev:
    before=sum(f.id==301 for f in dev.receive(a.duration))
    if before < 10: raise SystemExit('INCONCLUSIVE: insufficient odometry before STOP')
    dev.send(103)
    after=sum(f.id==301 for f in dev.receive(a.duration))
    dev.send(103)  # idempotence
    pid=None
    dev.send(202)
    for f in dev.receive(2.0):
        if f.id==202 and len(f.fields)==6: pid=f.fields;break
    print(f'Odometry before={before}, after={after}; PID response={pid}; decode errors={dev.errors}')
    if after>=10 and pid is not None and dev.errors==0:
        print('PASS: STOP sent twice; UART/telemetry recovered. PWM zero NOT proven by this test.')
    else:raise SystemExit('FAIL: STOP UART smoke test')
