import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2] / "esp32_firmware"

class StartupStructureTests(unittest.TestCase):
    def test_service_exists_and_setup_is_delegated(self):
        sketch = (ROOT / "esp32_firmware.ino").read_text()
        service = (ROOT / "startup_service.cpp").read_text()
        self.assertIn("startupService.begin();", sketch)
        self.assertIn("void StartupService::begin()", service)
        self.assertNotIn("Wire.begin()", sketch.split("void setup()", 1)[1].split("void loop()", 1)[0])

    def test_startup_sequence_and_safe_stop(self):
        code = (ROOT / "startup_service.cpp").read_text()
        stages = ["ctx_.error_manager.begin(", "Serial.begin(", "Serial1.begin(",
                  "Wire.begin()", "ctx_.encoders.begin(", "ctx_.controller.begin()",
                  "ctx_.INA.begin()", "ctx_.display.begin(", "ctx_.flashStorage.begin(",
                  "ctx_.flashStorage.loadPID(", "ctx_.controller.setPID(",
                  "ctx_.flashStorage.loadFF(", "ctx_.flashStorage.loadRefTau(",
                  "ctx_.flashStorage.loadAdrc(", "ctx_.flashStorage.loadCtrlMode(",
                  "ctx_.controller.emergencyStop()", "ctx_.encoders.reset(",
                  "ctx_.speedL.reset(", "ctx_.speedR.reset(", "ctx_.odom_tmr = micros()",
                  "ctx_.control_next_us = micros() + CONTROL_PERIOD_US"]
        offsets = [code.index(t) for t in stages]
        self.assertEqual(offsets, sorted(offsets))

if __name__ == "__main__":
    unittest.main()
