"""Source and protocol tests only; firmware is compiled manually."""
import unittest
from pathlib import Path
from protocol import encode, decode
ROOT = Path(__file__).resolve().parents[2] / 'esp32_firmware'
class TestI2CUART(unittest.TestCase):
    def test_decoder_and_length(self):
        sample = '$405;1;0;0;0;2;3;4;2;3;5;8;6;#'
        f = decode(sample)
        self.assertEqual((f.id, len(f.fields)), (405, 12))
        with self.assertRaises(ValueError): decode('$405;1;#')
        self.assertEqual(encode(405), b'$405;#')
    def test_read_only_dispatch(self):
        parser=(ROOT/'command_parser.cpp').read_text()
        policy=(ROOT/'command_policy.h').read_text()
        diag=(ROOT/'diagnostics_service.cpp').read_text()
        publisher=(ROOT/'protocol_wrapper.cpp').read_text()
        self.assertIn('case 405: return Command::REQUEST_I2C_DIAGNOSTICS;', parser)
        self.assertIn('case Command::REQUEST_INFO: case Command::REQUEST_I2C_DIAGNOSTICS: return c.count == 0;',policy)
        self.assertIn('i2cDiagnosticsPublish(ctx.inaDiagnostics, ctx.pwmDiagnostics, ctx.oledDiagnostics)',diag)
        self.assertIn('Serial.availableForWrite() >= n',publisher)
if __name__ == '__main__': unittest.main()
