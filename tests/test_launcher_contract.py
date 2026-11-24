import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class LauncherContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launcher = (ROOT / "apex").read_text(encoding="utf-8")

    def test_camera_supervisor_confirms_before_restart(self):
        self.assertIn('http://127.0.0.1:8888/health', self.launcher)
        camera_ready = self.launcher.split('camera_ready() {', 1)[1].split('}', 1)[0]
        self.assertNotIn('/snapshot', camera_ready)
        self.assertIn('local timeout_s="${1:-1.5}"', self.launcher)
        self.assertIn('if camera_ready 4.0; then', self.launcher)
        confirmation = self.launcher.index('if camera_ready 4.0; then')
        restart = self.launcher.index('Kamera yayını sağlıksız; otomatik toparlama başlıyor.')
        self.assertLess(confirmation, restart)

    def test_hailo_camera_defaults_to_measured_hd_profile(self):
        self.assertIn('HUD_WIDTH="${APEX_HUD_WIDTH:-1280}"', self.launcher)
        self.assertIn('HUD_HEIGHT="${APEX_HUD_HEIGHT:-720}"', self.launcher)
        self.assertIn('CAMERA_START_ATTEMPTS="${APEX_CAMERA_START_ATTEMPTS:-180}"', self.launcher)


if __name__ == "__main__":
    unittest.main()
