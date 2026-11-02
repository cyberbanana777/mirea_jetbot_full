"""Source-level regression guards; no MCU compilation or hardware actions."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2] / 'esp32_firmware'


class HardwareSafety231Tests(unittest.TestCase):
    def test_startup_flags_begin_unconfirmed(self):
        sketch = (ROOT / 'esp32_firmware.ino').read_text()
        for name in ('inaOk', 'pwmOk', 'oledOk'):
            self.assertIn(f'bool {name} = false;', sketch)
            self.assertIn(f'{name}, pwmOk' if name == 'inaOk' else name, sketch)

    def test_startup_flags_depend_on_probe_and_init(self):
        code = (ROOT / 'startup_service.cpp').read_text()
        for device, prefix in [('ina', 'ina'), ('pwm', 'pwm'), ('oled', 'oled')]:
            self.assertIn(f'ctx_.{device}Ok = {prefix}Detected && {prefix}Initialized;', code)
        self.assertIn('const bool pwmInitialized = ctx_.controller.begin();', code)
        self.assertIn('const bool inaInitialized = ctx_.INA.begin();', code)
        self.assertIn('const bool oledInitialized = ctx_.display.begin(', code)

    def test_pwm_reinit_requires_real_i2c_ack(self):
        code = (ROOT / 'motor_controller.cpp').read_text()
        section = code.split('bool MotorController::reinitializePWM() {', 1)[1].split('void MotorController::emergencyStop()', 1)[0]
        self.assertIn('if (!_pwm.begin())', section)
        self.assertIn('emergencyStop();', section)
        self.assertIn('return false;', section)
        self.assertLess(section.index('if (!_pwm.begin())'), section.index('_pwm.setPWMFreq(1000)'))
        monitor = (ROOT / 'hardware_monitor.cpp').read_text()
        self.assertIn('controller.reinitializePWM() && ctx_.probe(0x60)', monitor)


if __name__ == '__main__':
    unittest.main()
