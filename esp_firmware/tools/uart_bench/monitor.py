import argparse,time
from client import Client
from protocol import IDS
p=argparse.ArgumentParser(description='Passive UART monitor; does not command motors')
p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
p.add_argument('--duration',type=float,default=10);p.add_argument('--csv')
a=p.parse_args()
import csv
with Client(a.port,a.baud) as dev:
    file=open(a.csv,'w',newline='') if a.csv else None
    try:
        writer=csv.writer(file) if file else None
        if writer:writer.writerow(['host_time_iso','monotonic_s','id','name','fields','raw'])
        counts={};start=time.monotonic()
        for f in dev.receive(a.duration):
            counts[f.id]=counts.get(f.id,0)+1
            print(f'{time.monotonic()-start:8.3f}s {f.id:>3} {IDS.get(f.id,"unknown"):>20} {f.fields}')
            if writer:writer.writerow([time.strftime('%Y-%m-%dT%H:%M:%S'),time.monotonic(),f.id,IDS.get(f.id,'unknown'),' '.join(map(str,f.fields)),f.raw])
        print('Frames:',counts,'decode errors:',dev.errors,'overflow:',dev.framer.overflow)
    finally:
        if file:file.close()
