"""Source-level regression: command dispatch remains exhaustive."""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / 'esp32_firmware'

class DispatcherCoverageTests(unittest.TestCase):
    def test_all_enum_values_have_dispatch_cases(self):
        header = (ROOT / 'command_parser.h').read_text(encoding='utf-8')
        dispatcher = (ROOT / 'command_dispatcher.cpp').read_text(encoding='utf-8')
        enum = re.search(r'enum class Command\s*:\s*uint8_t\s*\{([^}]*)\}', header, re.S)
        self.assertIsNotNone(enum)
        names = set(re.findall(r'^\s*([A-Z][A-Z0-9_]*)\s*=', enum.group(1), re.M))
        cases = set(re.findall(r'case\s+Command::([A-Z][A-Z0-9_]*)\s*:', dispatcher))
        self.assertEqual(names, cases)

    def test_main_loop_has_one_dispatch_entry(self):
        ino = (ROOT / 'esp32_firmware.ino').read_text(encoding='utf-8')
        self.assertEqual(ino.count('dispatchCommand(cmd, commandContext);'), 1)
        for name in ('REQUEST_PID_TX', 'SET_PID_TX', 'REQUEST_INFO', 'REQUEST_DIAGNOSTICS'):
            self.assertNotIn('cmd.type == Command::' + name, ino)

if __name__ == '__main__':
    unittest.main()
