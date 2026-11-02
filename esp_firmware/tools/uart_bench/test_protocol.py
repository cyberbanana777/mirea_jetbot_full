import unittest
from protocol import encode,decode,Framer
class ProtocolTests(unittest.TestCase):
    def test_twist(self):self.assertEqual(encode(101,0.1,-0.2),b'$101;0.1;-0.2;#')
    def test_odometry(self):self.assertEqual(decode('$301;0;0;0;0;0;0;0;#').fields,(0.,)*7)
    def test_pid(self):self.assertEqual(len(decode('$202;1;2;3;4;5;6;#').fields),6)
    def test_invalid(self):
        for s in ('$301;1;#','$x;#','$301;nan;0;0;0;0;0;0;#','$301;0;0;0;0;0;0;0#'):
            with self.subTest(s=s),self.assertRaises(ValueError):decode(s)
    def test_recovery_and_multiple(self):
        f=Framer();self.assertEqual(len(f.feed(b'junk$101;0;0;#$202;#')),2)
        self.assertEqual(f.feed(b'$301;0'),[])
        self.assertEqual(f.feed(b';0;0;0;0;0;0;#'),[b'$301;0;0;0;0;0;0;0;#'])
    def test_oversize(self):
        f=Framer();self.assertEqual(f.feed(b'$'+b'1'*300+b'#$202;#'),[b'$202;#']);self.assertEqual(f.overflow,1)
if __name__=='__main__':unittest.main()


class TransactionalProtocolTests(unittest.TestCase):
    def test_transactional_pid_reply(self):
        f=decode('$211;314;0.951;5.475;0;1.426;10.009;0;#')
        self.assertEqual(f.id, 211)
        self.assertEqual(len(f.fields), 7)
    def test_ack_and_nack(self):
        self.assertEqual(decode('$502;42;210;#').fields, (42.,210.))
        self.assertEqual(decode('$503;42;210;1;#').fields, (42.,210.,1.))
