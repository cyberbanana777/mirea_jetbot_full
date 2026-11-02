"""Source-only regression checks; no ESP32 build or I2C writes."""
from pathlib import Path
import unittest
F = Path(__file__).resolve().parents[2] / "esp32_firmware"
class Diagnostics232(unittest.TestCase):
    def test_state_and_counters(self):
        h = (F / "hardware_monitor.h").read_text()
        for token in ("kNotDetected", "kHealthy", "kFault", "kRecovered", "probeFailures", "recoveryAttempts", "successfulRecoveries"):
            self.assertIn(token, h)
    def test_startup_capture(self):
        ino=(F / "esp32_firmware.ino").read_text()
        self.assertLess(ino.index("startupService.begin();"), ino.index("hardwareMonitor.observeStartup();"))
        code=(F / "hardware_monitor.cpp").read_text()
        for field in ("inaOk", "pwmOk", "oledOk"):
            self.assertIn("ctx_."+field+" ? I2CDeviceState::kHealthy", code)
    def test_attempt_result_recorded_each_device(self):
        code=(F / "hardware_monitor.cpp").read_text()
        for dev in ("ina", "pwm", "oled"):
            self.assertIn(f"recordProbe(ctx_.{dev}Diagnostics,",code)
            self.assertIn(f"recordRecoveryAttempt(ctx_.{dev}Diagnostics);",code)
            self.assertIn(f"recordRecoveryResult(ctx_.{dev}Diagnostics, true);",code)
            self.assertIn(f"recordRecoveryResult(ctx_.{dev}Diagnostics, false);",code)
    def test_unchanged_uart_and_scheduling(self):
        ino=(F / "esp32_firmware.ino").read_text()
        self.assertIn("serialService.processCommands();",ino)
        self.assertIn("ControlScheduler::controlDue(",ino)
        self.assertIn("ControlScheduler::odometryDue(",ino)
if __name__ == "__main__": unittest.main()
