import unittest
from unittest.mock import patch
from hardware_suite import run_suite, send, ALLOWED_TX_IDS
from protocol import Frame

class FakeSerial:
    def __init__(self): self.sent = []
    def write(self, b): self.sent.append(b)
    def flush(self): pass

class FakeFramer:
    overflow = 0

class FakeClient:
    def __init__(self):
        self.errors = 0
        self.framer = FakeFramer()
        self.serial = FakeSerial()
        self.sent = []
    def send(self, msg_id, *fields): self.sent.append(msg_id)
    def receive(self, seconds):
        if self.sent and self.sent[-1] in (202, 402, 501, 211):
            msg_id = self.sent.pop()
            payload = {202:(0.951,5.475,0.,1.426,10.009,0.),
                       402:(1200.,3.,0.,1.,1.,1.,1.),501:(2.,2.),211:(314.,0.951,5.475,0.,1.426,10.009,0.)}[msg_id]
            yield Frame(msg_id, payload, f'${msg_id};...;#')
        elif self.serial.sent and self.serial.sent[-1].startswith(b'$210;315;'):
            self.serial.sent.clear()
            yield Frame(503, (315.,210.,1.), '$503;315;210;1;#')
        else:
            for i in range(12):
                yield Frame(301, (0.,)*7, '$301;...;#')

class Tests(unittest.TestCase):
    def test_only_read_and_stop(self):
        dev = FakeClient()
        rows=[]
        with patch('hardware_suite.time.sleep'):
            results=run_suite(dev, 1.0, rows, malformed=True)
        self.assertTrue(all(x['status']=='PASS' for x in results.values()))
        self.assertEqual(dev.serial.sent, [raw for _, raw in __import__('hardware_suite').MALFORMED])
        self.assertIn(103, dev.sent)
        self.assertTrue(set(dev.sent) <= ALLOWED_TX_IDS)
    def test_reject_other_tx(self):
        with self.assertRaises(ValueError): send(FakeClient(), 101)

if __name__ == '__main__': unittest.main()
