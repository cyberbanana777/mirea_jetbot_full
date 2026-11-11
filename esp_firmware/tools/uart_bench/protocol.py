"""Protocol v2 ASCII codec. Works without serial hardware."""
import math
from dataclasses import dataclass

# Предел приёмного буфера прошивки: SerialReceiver::BUFFER_SIZE - 1 = 127 байт.
# Кадр длиннее прошивка молча отбросит (serial_receiver.cpp: переполнение ->
# сброс приёма), поэтому кодировщик не должен собирать такие кадры.
MAX_COMMAND_BYTES = 127
# Предел для кадров, которые шлёт прошивка: самый длинный — $405
# (до 140 байт с CRLF при насыщенных 32-битных счётчиках).
MAX_FRAME_BYTES = 192
IDS = {101:'twist',102:'wheel_targets',103:'stop',104:'open_loop',
       201:'set_pid_ram',202:'pid',210:'set_pid_transaction',211:'pid_transaction',203:'adrc',204:'set_pid_nvs',
       205:'feedforward',206:'query_config',207:'reference_filter',
       208:'control_mode',209:'query_adrc',301:'odometry',
       401:'diagnostics',402:'diagnostic_snapshot',405:'i2c_diagnostics',501:'device_info',502:'ack',503:'nack',403:'diagnostics_enable',404:'speed_window'}
EXPECTED_RX_FIELDS = {211:7,502:2,503:3,301:7,202:6,203:6,205:4,207:1,401:7,402:7,405:12,501:2,404:2}

@dataclass(frozen=True)
class Frame:
    id: int
    fields: tuple[float,...]
    raw: str

def encode(msg_id:int,*fields:float)->bytes:
    if msg_id not in IDS: raise ValueError(f'unknown ID: {msg_id}')
    tokens=[]
    for x in fields:
        v=float(x)
        if not math.isfinite(v): raise ValueError('non-finite field')
        tokens.append(format(v,'.9g'))
    msg='$'+str(msg_id)+';' + ''.join(s+';' for s in tokens)+'#'
    b=msg.encode('ascii')
    if len(b)>MAX_COMMAND_BYTES:raise ValueError('frame too long for the firmware buffer')
    return b

def decode(raw:bytes|str)->Frame:
    if isinstance(raw,bytes):raw=raw.decode('ascii','strict')
    raw=raw.strip('\r\n')
    if len(raw.encode('ascii'))>MAX_FRAME_BYTES or not raw.startswith('$') or not raw.endswith('#'):
        raise ValueError('invalid frame boundaries/length')
    body=raw[1:-1]
    if not body.endswith(';'):raise ValueError('missing terminal separator')
    items=body[:-1].split(';')
    if not items[0].isdigit():raise ValueError('invalid ID')
    msg_id=int(items[0]); values=[]
    for t in items[1:]:
        if not t:raise ValueError('empty numeric field')
        v=float(t)
        if not math.isfinite(v):raise ValueError('non-finite number')
        values.append(v)
    if msg_id in EXPECTED_RX_FIELDS and len(values)!=EXPECTED_RX_FIELDS[msg_id]:
        raise ValueError('incorrect field count')
    return Frame(msg_id,tuple(values),raw)

class Framer:
    def __init__(self):self.buf=bytearray();self.collecting=False;self.overflow=0
    def feed(self,data:bytes):
        ready=[]
        for b in data:
            if b==36:
                self.buf=bytearray(b'$');self.collecting=True
            elif self.collecting:
                self.buf.append(b)
                if len(self.buf)>MAX_FRAME_BYTES:
                    self.overflow+=1;self.buf.clear();self.collecting=False
                elif b==35:
                    ready.append(bytes(self.buf));self.buf.clear();self.collecting=False
        return ready
