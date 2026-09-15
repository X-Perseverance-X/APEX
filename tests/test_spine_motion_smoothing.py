import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "v57_spine_safe_boot" / "v57_spine_safe_boot.ino"


class SpineMotionSmoothingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = FIRMWARE.read_text(encoding="utf-8")

    def test_safe_boot_and_new_version_remain_explicit(self):
        self.assertIn('APEX_FIRMWARE_VERSION "v57.9-mounted-vector-gait"', self.source)
        setup = self.source.split("void setup()", 1)[1].split("void loop()", 1)[0]
        self.assertIn("disableLegPwm(-1, -1);", setup)
        self.assertIn("spineState       = SPINE_DISARMED;", setup)

    def test_calibration_offset_is_applied_once_at_pwm_conversion(self):
        output = self.source.split("void send_to_PCA9685", 1)[1].split(
            "void writeLegAngleImmediate", 1
        )[0]
        self.assertIn("safe_angle = constrain(target_angle, minPhysical, maxPhysical)", output)
        self.assertIn("currentPhysicalAngle[leg][joint] + offset[leg][joint]", output)

    def test_walk_has_time_based_slew_and_smooth_contact_boundaries(self):
        self.assertIn("LEG_TRACK_MAX_SPEED_DEG_S_BY_JOINT[joint] * controlFrameDtS", self.source)
        self.assertNotIn("else                             maxStep = 30.0f", self.source)
        gait = self.source.split("else if (currentMode == BEZIER_JOY)", 1)[1].split(
            "else if (currentMode == HOME_RISE)", 1
        )[0]
        self.assertIn("bt*bt*bt*(10.0f + bt*(-15.0f + 6.0f*bt))", gait)
        self.assertIn("powf(liftSin, 4.0f)", gait)
        self.assertIn("liftSin * liftSin", gait)
        self.assertIn("st*st*st*(10.0f + st*(-15.0f + 6.0f*st))", gait)
        self.assertIn("GAIT_CONTACT_SLOPE*h10", gait)
        self.assertIn("forwardToLocalX", gait)
        self.assertIn("forwardToLocalYOut", gait)
        self.assertIn("preservePureTurnGait", gait)
        self.assertIn("gaitPhaseRateScale", gait)
        self.assertIn("pureTurnGaitActive ? max(s_joyMag, 0.1f)", gait)
        self.assertIn("maxStep = 120.0f * controlFrameDtS", self.source)

    def test_each_new_walk_starts_from_double_support(self):
        self.assertIn("gaitCommandWasActive", self.source)
        self.assertIn("currentPhase = 0.0f;", self.source)


if __name__ == "__main__":
    unittest.main()
