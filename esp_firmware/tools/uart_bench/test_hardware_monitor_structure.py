import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2] / "esp32_firmware"

class HardwareMonitorStructure(unittest.TestCase):
    def test_owning_module_and_calls(self):
        main = (ROOT / "esp32_firmware.ino").read_text()
        impl = (ROOT / "hardware_monitor.cpp").read_text()
        self.assertIn("hardwareMonitor.samplePower();", main)
        self.assertIn("hardwareMonitor.checkI2CDevices();", main)
        self.assertNotIn("void checkI2CDevices()", main)
        self.assertIn("void HardwareMonitor::checkI2CDevices()", impl)
        self.assertIn("ctx_.probe(0x41)", impl)
        self.assertIn("ctx_.probe(0x60)", impl)
        self.assertIn("ctx_.probe(0x3C)", impl)

if __name__ == "__main__":
    unittest.main()
