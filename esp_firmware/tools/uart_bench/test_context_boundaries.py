import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2] / 'esp32_firmware'

class ContextBoundariesTests(unittest.TestCase):
    def test_services_have_separate_contexts(self):
        config = (ROOT / 'config_service.cpp').read_text()
        diagnostics = (ROOT / 'diagnostics_service.cpp').read_text()
        dispatcher = (ROOT / 'command_dispatcher.cpp').read_text()
        self.assertIn('ConfigContext& ctx', config)
        self.assertIn('DiagnosticsContext& ctx', diagnostics)
        self.assertIn('handleConfigCommand(cmd, ctx.config)', dispatcher)
        self.assertIn('handleDiagnosticsCommand(cmd, ctx.diagnostics)', dispatcher)

    def test_diagnostics_has_no_flash_access(self):
        hdr = (ROOT / 'command_dispatcher.h').read_text()
        section = hdr.split('struct DiagnosticsContext {')[1].split('};')[0]
        self.assertNotIn('FlashStorageManager', section)
        self.assertNotIn('MTSpeedEstimator', section)

if __name__ == '__main__':
    unittest.main()
