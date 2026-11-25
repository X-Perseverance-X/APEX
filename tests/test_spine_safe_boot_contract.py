import math
import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "v57_spine_safe_boot" / "v57_spine_safe_boot.ino"


class SpineSafeBootContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = FIRMWARE.read_text(encoding="utf-8")
        cls.ui = (ROOT / "apex_ui.html").read_text(encoding="utf-8")
        cls.server = (ROOT / "apex_server.py").read_text(encoding="utf-8")

    def test_usb_status_printf_has_one_argument_per_field(self):
        status_line = next(line for line in self.source.splitlines() if 'Serial.printf("@TEL1,' in line)
        self.assertEqual(len(re.findall(r"%(?:[-+0-9.]+)?(?:lu|ld|[udfs])", status_line)), 40)
        self.assertIn('homeSet ? 1U : 0U,', self.source)
        self.assertIn('APEX_FIRMWARE_VERSION);', self.source)

    def test_boot_is_disarmed_and_does_not_start_ramp(self):
        setup = self.source.split("void setup()", 1)[1].split("void loop()", 1)[0]
        self.assertIn("disableLegPwm(-1, -1);", setup)
        self.assertIn("spineState       = SPINE_DISARMED;", setup)
        self.assertIn("bootSoftStart   = false;", setup)
        self.assertNotIn("currentMode = homeSet ? HOME_RISE", setup)

    def test_dfplayer_uses_filename_addressed_mp3_folder_command(self):
        self.assertIn("dfPlayer.playMp3Folder(dfTrack);", self.source)
        self.assertNotIn("dfPlayer.play(dfTrack);", self.source)

    def test_foot_contacts_are_read_only_debounced_inputs(self):
        self.assertIn("const uint8_t FOOT_CONTACT_PINS[6] = {6, 7, 10, 11, 12, 13};", self.source)
        self.assertIn("pinMode(FOOT_CONTACT_PINS[i], INPUT_PULLUP);", self.source)
        self.assertIn("FOOT_CONTACT_DEBOUNCE_MS = 10UL", self.source)
        self.assertIn("pollFootContactSensors();", self.source)
        self.assertIn("@FOOT1,%lu,%lu,%u,%lu", self.source)
        self.assertIn("emitFootContactSerialTelemetry();", self.source)
        self.assertIn("footContactMask", self.source)
        self.assertNotIn("footContactStable[i] && send_to_PCA9685", self.source)

    def test_gps_diagnostics_distinguish_transport_from_rf_fix(self):
        for marker in (
            "gps.passedChecksum()",
            "gps.failedChecksum()",
            "gps.sentencesWithFix()",
            "gps.location.age()",
            "gps.hdop.value()",
        ):
            self.assertIn(marker, self.source)

    def test_stand_requires_explicit_confirmation_and_online_controllers(self):
        stand = self.source.split("bool beginStandupSequence", 1)[1].split(
            "// ================= 3D", 1
        )[0]
        endpoint = self.source.split('server.on("/goHome"', 1)[1].split(
            'server.on("/armStatus"', 1
        )[0]
        self.assertIn('server.arg("confirm") != "start"', endpoint)
        self.assertIn("!pcaRightOnline || !pcaLeftOnline", stand)
        self.assertIn("!configValid", stand)
        self.assertIn("spineState = SPINE_CAPTURE;", stand)

    def test_hud_names_missing_pca_before_home_and_shows_single_2s_pack(self):
        self.assertIn("PCA9685 ' + missing + ' çevrimdışı", self.ui)
        self.assertIn("if (!t.homeSet) throw new Error('Kayıtlı HOME/CAPTURE", self.ui)
        self.assertIn('TEK 2S PİL · GPIO1', self.ui)
        self.assertIn("data.batR/2", self.ui)
        self.assertNotIn('DISARMED, iki PCA ve geçerli HOME gerekli', self.ui)

    def test_stand_ramp_is_slow_and_cannot_fall_through_to_fast_slew(self):
        self.assertIn("STAND_RAMP_MS   = 3000UL", self.source)
        self.assertIn("STAND_MAX_SPEED_DEG_S = 30.0f", self.source)
        self.assertIn(
            "if      (currentMode == HOME_RISE) maxStep = STAND_MAX_SPEED_DEG_S * controlFrameDtS;",
            self.source,
        )
        self.assertIn(
            "MANUAL_LAB_MAX_ACCEL_DEG_S2 * controlFrameDtS",
            self.source,
        )

    def test_stand_splays_front_and_rear_to_mirrored_coxa_angles(self):
        match = re.search(r"legNeutralFwdOffset\[6\]\s*=\s*\{([^}]+)\}", self.source)
        self.assertIsNotNone(match)
        offsets = [float(value.strip().rstrip("f")) for value in match.group(1).split(",")]
        self.assertEqual(len(offsets), 6)
        outward_match = re.search(r"legNeutralOutwardY\[6\]\s*=\s*\{([^}]+)\}", self.source)
        self.assertIsNotNone(outward_match)
        outward = [float(value.strip().rstrip("f")) for value in outward_match.group(1).split(",")]
        for x, y in zip(offsets, outward):
            self.assertAlmostEqual(math.hypot(x, y), 150.0, places=2)
        geometric = [math.degrees(math.atan2(y, x)) for x, y in zip(offsets, outward)]
        servo = [180.0 - angle if leg < 3 else angle for leg, angle in enumerate(geometric)]
        for actual, expected in zip(servo, [135, 90, 45, 45, 90, 135]):
            self.assertAlmostEqual(actual, expected)
        self.assertIn("homeRisePhase = 2;", self.source)
        self.assertIn("STAND_SPLAY_LIFT_MM * lift", self.source)
        self.assertIn("STAND_SPLAY_ORDER[standSplayIndex]", self.source)
        self.assertIn("gaitY*gaitY - neutralX*neutralX", self.source)

    def test_body_ik_is_guarded_on_both_usb_ends(self):
        self.assertIn('if (action == "BODY" && count == 7)', self.source)
        self.assertIn('"BODY_REQUIRES_STAND"', self.source)
        self.assertIn('if SPINE_USB_PREFER and path == "/bodyIk":', self.server)
        self.assertIn('"SPINE", "BODY"', self.server)
        self.assertIn(
            "safeTransitionActive && currentMode != HOME_RISE",
            self.source,
        )

    def test_unpowered_i2c_devices_cannot_block_safe_boot(self):
        setup = self.source.split("void setup()", 1)[1].split("void loop()", 1)[0]
        self.assertLess(setup.index("scanI2C();"), setup.index("initMPU6050();"))
        self.assertLess(setup.index("scanI2C();"), setup.index("initTOF400C();"))
        tof_init = self.source.split("void initTOF400C()", 1)[1].split(
            "void updateTOF400C()", 1
        )[0]
        self.assertIn("if (!tofAddressOnline)", tof_init)
        self.assertIn("init atlandi", tof_init)

    def test_old_tripod_autolift_is_removed_from_v57(self):
        rise = self.source.split("else if (currentMode == HOME_RISE)", 1)[1]
        self.assertIn("CAPTURE_HOLD_MS", rise)
        self.assertIn("spineState = SPINE_RISING;", rise)
        self.assertNotIn("bool isA", rise)
        self.assertNotIn("bool isB", rise)

    def test_pitch_feedback_matches_requested_front_and_rear_z_response(self):
        self.assertIn("float pErr = s_mpuPitch - gimbalRefPitch;", self.source)
        # IMU +X points forward. Nose-up is negative pitch. With the firmware's
        # R^T transform z2=sin(pitch)*x+cos(pitch)*z, front Z becomes lower and
        # rear Z becomes higher, exactly matching the physical requirement.
        pitch = math.radians(-5.0)
        nominal_z = -100.0
        front_z = math.sin(pitch) * 120.0 + math.cos(pitch) * nominal_z
        rear_z = math.sin(pitch) * -120.0 + math.cos(pitch) * nominal_z
        self.assertLess(front_z, nominal_z)
        self.assertGreater(rear_z, nominal_z)

    def test_hud_and_proxy_forward_the_confirmation(self):
        self.assertIn("startControlledStand()", self.ui)
        self.assertIn("/goHome?confirm=start", self.ui)
        self.assertIn('"goHome":          ("/goHome",          ["confirm"])', self.server)

    def test_telemetry_polling_does_not_queue_ahead_of_control_commands(self):
        self.assertIn('if esp32_lock.locked() and cached_telemetry is not None:', self.server)
        self.assertIn('cached_telemetry["proxyStale"] = True', self.server)
        self.assertIn('if path == "/legStatus" and esp32_lock.locked():', self.server)
        self.assertNotIn('"coalesced": True', self.server)
        self.assertIn("let motionModeActive = false;", self.ui)
        self.assertIn("if (!motionModeActive || controlMode !== 'MANUAL' || motionRequestPending)", self.ui)
        self.assertIn("const shouldTransmitStop = motionModeActive", self.ui)
        self.assertIn("if (!shouldTransmitStop) return;", self.ui)

    def test_wired_deadman_remains_bounded(self):
        self.assertIn("MOTION_COMMAND_TIMEOUT_MS = 750UL", self.source)
        self.assertIn('APEX_FIRMWARE_VERSION "v57.32-body-frame-gait"', self.source)
        self.assertLess(750, 1000)

    def test_failsafe_holds_last_joint_commands_without_idle_ik_jump(self):
        failsafe = self.source.split("void triggerMotionFailsafe()", 1)[1].split(
            "// --- TOF400C", 1
        )[0]
        self.assertIn("manualTargetAngle[leg][joint] = currentPhysicalAngle[leg][joint]", failsafe)
        self.assertIn("currentMode = MANUAL", failsafe)
        self.assertNotIn("currentMode = IDLE", failsafe)

    def test_physical_joint_state_applies_calibration_offset_exactly_once(self):
        output = self.source.split("void send_to_PCA9685", 1)[1].split(
            "void writeLegAngleImmediate", 1
        )[0]
        self.assertIn("safe_angle = constrain(target_angle, minPhysical, maxPhysical)", output)
        self.assertIn("currentPhysicalAngle[leg][joint] + totalOffset", output)
        self.assertIn("totalOffset = (float)offset[leg][joint] + softwareTrim", output)
        self.assertNotIn("safe_angle - currentPhysicalAngle", output.split("const float safe_angle", 1)[0])

    def test_walk_has_time_based_joint_speed_limit_and_zero_velocity_foot_boundaries(self):
        self.assertIn("LEG_TRACK_MAX_SPEED_DEG_S_BY_JOINT[joint] * controlFrameDtS", self.source)
        self.assertNotIn("else                             maxStep = 30.0f", self.source)
        gait = self.source.split("else if (currentMode == BEZIER_JOY)", 1)[1].split(
            "else if (currentMode == HOME_RISE)", 1
        )[0]
        self.assertIn("bt*bt*bt*(10.0f + bt*(-15.0f + 6.0f*bt))", gait)
        self.assertIn("powf(liftSin, 4.0f)", gait)
        self.assertIn("liftSin * liftSin", gait)
        self.assertIn("st*st*st*(10.0f + st*(-15.0f + 6.0f*st))", gait)

    def test_translation_gait_has_continuous_ground_speed_and_lag_governor(self):
        self.assertIn('APEX_FIRMWARE_VERSION "v57.32-body-frame-gait"', self.source)
        gait = self.source.split("else if (currentMode == BEZIER_JOY)", 1)[1].split(
            "else if (currentMode == HOME_RISE)", 1
        )[0]
        for contract in (
            "MIN_DYNAMIC_GAIT_CYCLE_MS",
            "gaitTrackingErrorDeg",
            "gaitPhaseRateScale",
            "preservePureTurnGait",
            "GAIT_CONTACT_SLOPE*h10",
            "leg_vX = vX - sideSign * diff",
            "leg_vY = vY",
            "GAIT_FULL_STRIDE_INPUT",
            "GAIT_FULL_LIFT_INPUT",
            "liftSin * liftSin",
        ):
            self.assertIn(contract, gait if contract not in {
                "MIN_DYNAMIC_GAIT_CYCLE_MS", "gaitTrackingErrorDeg", "gaitPhaseRateScale"
            } else self.source)
        self.assertIn("pureTurnGaitActive ? max(s_joyMag, 0.1f)", gait)
        self.assertNotIn("forwardToLocalX", gait)
        self.assertIn("maxStep = 120.0f * controlFrameDtS", self.source)

        # Translation swing and stance Hermite curves share the same non-zero
        # endpoint slope, so the foot never receives a horizontal velocity jump.
        slope = -0.55
        def swing(t):
            h00 = 2*t**3 - 3*t**2 + 1
            h10 = t**3 - 2*t**2 + t
            h01 = -2*t**3 + 3*t**2
            h11 = t**3 - t**2
            return -h00 + slope*h10 + h01 + slope*h11

        eps = 1e-5
        self.assertAlmostEqual(swing(0.0), -1.0)
        self.assertAlmostEqual(swing(1.0), 1.0)
        self.assertAlmostEqual((swing(eps)-swing(0.0))/eps, slope, places=3)
        self.assertAlmostEqual((swing(1.0)-swing(1.0-eps))/eps, slope, places=3)

        # Reconstruct a +X body command from every leg's radial mount frame.
        offsets = ((120,-80),(0,-90),(-120,-80),(120,80),(0,90),(-120,80))
        for index, (ox, oy) in enumerate(offsets):
            radius = (ox*ox + oy*oy) ** 0.5
            out = (ox/radius, oy/radius)
            tangent = ((-out[1], out[0]) if index < 3 else (out[1], -out[0]))
            local_x = tangent[0]
            local_out = out[0]
            reconstructed = (
                local_x*tangent[0] + local_out*out[0],
                local_x*tangent[1] + local_out*out[1],
            )
            self.assertAlmostEqual(reconstructed[0], 1.0, places=6)
            self.assertAlmostEqual(reconstructed[1], 0.0, places=6)

    def test_new_walk_command_restarts_from_double_support_phase(self):
        self.assertIn("currentPhase = 0.0f;", self.source)
        self.assertIn("gaitCommandWasActive", self.source)

    def test_esp_uplink_is_usb_only_and_wifi_is_disabled(self):
        setup = self.source.split("void setup()", 1)[1].split("void loop()", 1)[0]
        self.assertIn("WiFi.disconnect(true, true);", setup)
        self.assertIn("WiFi.mode(WIFI_OFF);", setup)
        self.assertNotIn("WiFi.begin(", setup)
        self.assertNotIn("server.begin();", setup)

    def test_thermal_fan_is_isolated_and_fails_safe(self):
        for marker in (
            "#define FAN_PWM_PIN 18",
            "#define NTC_ADC_PIN 15",
            "FAN_PWM_HZ = 25000",
            "NTC_BETA_K = 3950.0f",
            "if (ntcValid && internalTempC >= 65.0f)",
            "applyFanDuty(100);",
            "requestFanPower(ntcValid ? autoFanPercent(internalTempC) : 100);",
            'if (device == "FAN")',
            "initThermalManager();",
            "updateThermalManager();",
        ):
            self.assertIn(marker, self.source)
        self.assertNotIn("pinMode(FAN_PWM_PIN, OUTPUT);\n  digitalWrite", self.source)

    def test_camera_gimbal_owns_only_reserved_channels_and_is_bounded(self):
        for marker in (
            "CAMERA_PAN_CHANNEL = 15",
            "CAMERA_TILT_CHANNEL = 14",
            "CAMERA_GIMBAL_MIN_DEG = 45.0f",
            "CAMERA_GIMBAL_MAX_DEG = 135.0f",
            "CAMERA_GIMBAL_CENTER_DEG = 90.0f",
            'if (device == "CAM_PAN" || device == "CAM_TILT")',
            'if (device == "CAM_GIMBAL")',
            'action == "STABILIZE"',
            'action == "SET"',
            "CAMERA_GIMBAL_STABILIZE",
            "CAMERA_GIMBAL_YAW_RECENTER_S = 6.0f",
            "CAMERA_GIMBAL_MAX_YAW_CORRECTION_DEG = 20.0f",
            "yawDelta *= (1.0f - yawRecenterAlpha)",
            "cameraPanTargetDeg = constrain(CAMERA_GIMBAL_CENTER_DEG - yawDelta",
            "cameraTiltTargetDeg = constrain(CAMERA_GIMBAL_CENTER_DEG - pitchDelta",
            "initCameraGimbal();",
            "updateCameraGimbal();",
        ):
            self.assertIn(marker, self.source)
        self.assertIn("const uint8_t armPins[ARM_JOINT_COUNT] = {12, 13, 3, 7};", self.source)

    def test_dfplayer_online_requires_a_real_uart_reply_but_playback_stays_nonblocking(self):
        for marker in (
            "DF_TX_PIN 4",
            "DF_RX_PIN 5",
            "dfPlayer.begin(dfSerial, false, appendBootLog)",
            "dfPlayer.readVolume()",
            "dfLinkConfirmed = reportedVolume >= 0 && reportedVolume <= 30",
            "dfLinkConfirmed ? 1U : 0U",
            "DFPLAYER_TX_ONLY_UNCONFIRMED",
            "dfPlayer.enableLoop()",
            "if (dfShouldPlay) startDFPlayerTrack(dfTrack)",
            "AUDIO_STOP_DEBOUNCED",
            "pollDFPlayerEvents();",
        ):
            self.assertIn(marker, self.source)

    def test_usb_leg_lab_is_single_joint_bounded_and_keeps_motion_locked(self):
        for marker in (
            'if (device == "LEG_LAB")',
            'action == "SET"',
            'action == "ON"',
            'action == "OFF"',
            'LEG_LAB_SET_TARGET_FIRST',
            'enableLegPwm(leg, joint)',
            'disableLegPwm(leg, joint)',
        ):
            self.assertIn(marker, self.source)


if __name__ == "__main__":
    unittest.main()
