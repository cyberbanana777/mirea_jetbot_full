import argparse
from client import Client
p=argparse.ArgumentParser(description='Send software stop ID 103; NOT a hardware E-stop')
p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
a=p.parse_args()
with Client(a.port,a.baud) as dev:dev.send(103)
print('Software STOP frame sent. No ACK exists; visually verify wheels stopped.')
