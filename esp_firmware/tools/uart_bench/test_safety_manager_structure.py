import unittest
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[2] / "esp32_firmware"

class SafetyManagerBoundaryTests(unittest.TestCase):
    def test_watchdog_extracted_once(self):
        ino = (FIRMWARE / "esp32_firmware.ino").read_text()
        cpp = (FIRMWARE / "safety_manager.cpp").read_text()
        self.assertIn("safetyManager.check();", ino)
        self.assertNotIn("const bool host_silent =", ino)
        self.assertIn("const bool host_silent =", cpp)
        self.assertIn("SafetyPolicy::mustStop", cpp)
        self.assertIn("ctx_.controller.emergencyStop();", cpp)
        self.assertIn("ctx_.telemetryEnabled = false;", cpp)
    def test_shared_watchdog_and_order(self):
        ino = (FIRMWARE / "esp32_firmware.ino").read_text()
        self.assertLess(ino.index("serialService.poll();"), ino.index("safetyManager.check();"))
        self.assertLess(ino.index("safetyManager.check();"), ino.index("controlLoop.controlStep();"))
        self.assertIn("controller, emergency_timer, emergency_timer_timeout", ino)
        self.assertIn("watchdog_stop_count, watchdog_stop_latched, telemetry_on", ino)
        self.assertIn("emergency_timer, watchdog_stop_latched", ino)

if __name__ == "__main__":
    unittest.main()
