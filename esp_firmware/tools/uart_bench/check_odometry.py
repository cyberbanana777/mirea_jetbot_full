import argparse,time,statistics
from client import Client
p=argparse.ArgumentParser(description='Passive odometry frequency and finite-value check')
p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
p.add_argument('--duration',type=float,default=10)
a=p.parse_args(); times=[];samples=[]
with Client(a.port,a.baud) as dev:
    for f in dev.receive(a.duration):
        if f.id==301:times.append(time.monotonic());samples.append(f.fields)
    errors=dev.errors;overflow=dev.framer.overflow
if len(times)<2:raise SystemExit('FAIL: fewer than 2 odometry frames (check port, baud, telemetry mode)')
intervals=[b-a for a,b in zip(times,times[1:])]
print(f'Odometry frames={len(times)}, mean rate={1/statistics.mean(intervals):.2f} Hz, '+
      f'max gap={max(intervals)*1000:.1f} ms, decode errors={errors}, overflow={overflow}')
print('Latest [x,y,heading,v,w,left,right]:',samples[-1])
