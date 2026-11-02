import argparse
from client import Client
p=argparse.ArgumentParser(description='Read-only PID configuration request')
p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
p.add_argument('--timeout',type=float,default=3)
a=p.parse_args()
with Client(a.port,a.baud) as dev:
    dev.send(202)
    for f in dev.receive(a.timeout):
        if f.id==202:
            print('PID: KpL,KiL,KdL,KpR,KiR,KdR =',f.fields);break
    else:raise SystemExit('FAIL: no PID reply (ID 202)')
