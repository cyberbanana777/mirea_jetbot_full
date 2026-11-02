"""Source-only checks for config global ownership (not an ESP32 compiler test)."""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "esp32_firmware"
VARIABLES = ("KP_R", "KI_R", "KD_R", "KP_L", "KI_L", "KD_L", "DT",
             "FF_DEADZONE_L", "FF_DEADZONE_R", "FF_GAIN_L", "FF_GAIN_R",
             "REF_TAU_S", "CTRL_MODE", "ADRC_WC", "ADRC_WO",
             "ADRC_B0_L", "ADRC_B0_R", "ADRC_FF")

class ConfigLinkageTest(unittest.TestCase):
    def test_mutable_globals_declared_and_defined_once(self):
        header = (ROOT / "config.h").read_text()
        impl = (ROOT / "config.cpp").read_text()
        self.assertIn('#include "config.h"', impl)
        for name in VARIABLES:
            with self.subTest(name=name):
                self.assertRegex(header, rf"extern (?:float|int) {name};")
                self.assertNotRegex(header, rf"(?m)^(?:float|int)\s+{name}\s*=")
                self.assertEqual(len(re.findall(rf"(?m)^(?:float|int)\s+{name}\s*=", impl)), 1)

if __name__ == "__main__":
    unittest.main()
