"""Source-level checks; these do not compile firmware."""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / 'esp32_firmware'

class ServiceBoundariesTests(unittest.TestCase):
    def test_uart_and_scheduler_delegate_from_loop(self):
        ino = (ROOT / 'esp32_firmware.ino').read_text()
        service = (ROOT / 'serial_service.cpp').read_text()
        scheduler = (ROOT / 'control_scheduler.h').read_text()
        for statement in ('serialService.poll();', 'serialService.processCommands();',
                          'serialService.processSecondary();', 'ControlScheduler::controlDue(',
                          'ControlScheduler::odometryDue('):
            self.assertIn(statement, ino)
        self.assertIn('dispatchCommand(cmd, ctx_.commands);', service)
        self.assertIn('ctx_.mainReceiver.update();', service)
        self.assertIn('next_us = now_us + period_us;', scheduler)

    def test_no_motor_command_created_in_uart_service(self):
        src = (ROOT / 'serial_service.cpp').read_text()
        self.assertNotIn('controller.update(', src)
        self.assertNotIn('controller.setPID(', src)
        self.assertNotIn('odomPublish(', src)

if __name__ == '__main__':
    unittest.main()
