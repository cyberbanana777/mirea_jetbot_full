import time
from protocol import Framer,decode,encode
class Client:
    def __init__(self,port,baud=115200):
        try:import serial
        except ImportError as e:raise RuntimeError('Install pyserial: python -m pip install pyserial') from e
        self.serial=serial.Serial(port,baudrate=baud,timeout=0.05)
        self.framer=Framer()
        self.errors=0
    def close(self):self.serial.close()
    def send(self,msg_id,*fields):self.serial.write(encode(msg_id,*fields));self.serial.flush()
    def receive(self,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            for raw in self.framer.feed(self.serial.read(256)):
                try:yield decode(raw)
                except (ValueError,UnicodeError):self.errors+=1
    def __enter__(self):return self
    def __exit__(self,*_):self.close()
