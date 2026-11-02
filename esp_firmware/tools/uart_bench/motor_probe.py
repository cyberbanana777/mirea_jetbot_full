import argparse,time
from client import Client
p=argparse.ArgumentParser(description='SHORT MOTOR TEST. Wheels MUST be suspended.')
p.add_argument('--port',required=True);p.add_argument('--baud',type=int,default=115200)
p.add_argument('--speed',type=float,default=0.03);p.add_argument('--seconds',type=float,default=1.0)
p.add_argument('--confirm-wheels-lifted',action='store_true')
a=p.parse_args()
if not a.confirm_wheels_lifted: p.error('requires --confirm-wheels-lifted; no hardware motor kill switch')
if not (0<a.speed<=0.05) or not (0<a.seconds<=2):p.error('speed must be (0,0.05] m/s, seconds (0,2]')
print('WARNING: software stop cannot guarantee motor shutdown on ESP32/I2C failure.')
with Client(a.port,a.baud) as dev:
    try:
        until=time.monotonic()+a.seconds
        while time.monotonic()<until:
            dev.send(101,a.speed,0.0)
            time.sleep(0.08)
    finally:
        for _ in range(3):dev.send(103);time.sleep(0.05)
print('STOP frames sent. Disconnect motor power if wheels do not stop.')
