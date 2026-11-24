import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class UiContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main_ui = (ROOT / "apex_ui.html").read_text(encoding="utf-8")
        cls.camera_ui = (ROOT / "camera_console.html").read_text(encoding="utf-8")
        cls.navigation_ui = (ROOT / "navigation_console.html").read_text(encoding="utf-8")
        cls.server = (ROOT / "apex_server.py").read_text(encoding="utf-8")

    def test_main_hud_does_not_embed_or_start_camera_stream(self):
        self.assertNotIn('<img id="cameraFeed"', self.main_ui)
        self.assertNotIn("safeStep('Kamera', connectCameraFeed)", self.main_ui)
        self.assertIn('id="LidarMapCanvas"', self.main_ui)
        self.assertIn('onclick="openCameraConsole()"', self.main_ui)

    def test_audio_panel_directly_selects_all_installed_tracks(self):
        self.assertIn('id="audioTrackSelect"', self.main_ui)
        self.assertIn('onchange="selectAudioTrack(this.value)"', self.main_ui)
        for track in range(1, 10):
            self.assertIn(f'<option value="{track}">{track:04d}</option>', self.main_ui)
        self.assertIn("Math.min(9, currentTrack + 1)", self.main_ui)
        self.assertIn("OYNAT (DÖNGÜ)", self.main_ui)
        self.assertIn("audioCommandBusyUntil = now + 500", self.main_ui)

    def test_usb_leg_lab_sets_target_before_arming_only_the_selected_joint(self):
        self.assertIn("/legServo?leg=${leg}&j=${joint}&angle=${a}", self.main_ui)
        self.assertIn("/legOn?leg=${leg}&j=${joint}&confirm=lab", self.main_ui)
        self.assertIn('if path == "/legServo":', self.server)
        self.assertIn('if path == "/legOn":', self.server)

    def test_main_hud_renders_live_foot_contact_telemetry(self):
        for index, name in enumerate(("RF", "RM", "RR", "LF", "LM", "LR")):
            self.assertIn(f'id="footContact{index}">{name} BOŞ', self.main_ui)
        self.assertIn("latestFootContacts = data.footContacts.slice(0, 6).map(Boolean)", self.main_ui)
        self.assertIn("el.classList.toggle('pressed', pressed)", self.main_ui)
        self.assertIn("setInterval(fetchFootContacts, 50)", self.main_ui)
        self.assertIn("fetch('/api/foot-contacts'", self.main_ui)
        self.assertIn('@app.get("/api/foot-contacts")', self.server)
        self.assertIn('"footContacts": list(spine.get("foot_contacts", [False] * 6))[:6]', self.server)

    def test_usb_bridge_accepts_extended_contact_packet(self):
        bridge = (ROOT / "tools" / "apex_spine_usb_bridge.py").read_text(encoding="utf-8")
        self.assertIn("(20, 24, 26, 31, 35, 38, 39, 41)", bridge)
        self.assertIn('"foot_contact_mask": contact_mask', bridge)
        self.assertIn('"foot_contacts": [bool(contact_mask & (1 << i)) for i in range(6)]', bridge)
        self.assertIn('"gps_passed_checksum": int(f[26])', bridge)
        self.assertIn('"gps_failed_checksum": int(f[27])', bridge)
        self.assertIn('"gps_sentences_with_fix": int(f[28])', bridge)
        self.assertIn('elif raw.startswith("@FOOT1,")', bridge)
        self.assertIn('"foot_fps": state.foot_frames / uptime', bridge)
        for marker in ("gpsPassedChecksum", "gpsFailedChecksum", "gpsFixSentences", "gpsLocationAgeMs", "gpsHdop"):
            self.assertIn(f'"{marker}"', self.server)

    def test_thermal_fan_has_auto_manual_ui_and_usb_allowlist(self):
        bridge = (ROOT / "tools" / "apex_spine_usb_bridge.py").read_text(encoding="utf-8")
        for marker in (
            'id="fanPowerSlider"',
            'onclick="setFanAuto()"',
            "function updateFanTelemetry(data)",
            "updateFanTelemetry(data);",
            "'/fan?power='",
            "'/fan?mode=auto'",
        ):
            self.assertIn(marker, self.main_ui)
        self.assertIn('"FAN": {"AUTO", "OFF", "SET"}', bridge)
        self.assertIn('if path == "/fan":', self.server)
        for marker in ('"internalTempC"', '"ntcValid"', '"fanPowerPct"', '"fanAuto"'):
            self.assertIn(marker, self.server)

    def test_camera_pan_tilt_is_centered_bounded_and_usb_wired(self):
        bridge = (ROOT / "tools" / "apex_spine_usb_bridge.py").read_text(encoding="utf-8")
        for marker in (
            'id="cameraPanSlider"',
            'id="cameraTiltSlider"',
            'min="45" max="135"',
            'onclick="centerCameraGimbal()"',
            "function updateCameraGimbalTelemetry(data)",
            "updateCameraGimbalTelemetry(data);",
            "'/cameraGimbal?axis='",
            "'/cameraGimbal?action=center'",
        ):
            self.assertIn(marker, self.main_ui)
        for marker in (
            '"CAM_PAN": {"SET"}',
            '"CAM_TILT": {"SET"}',
            '"CAM_GIMBAL": {"CENTER", "ON", "OFF", "SET", "STABILIZE"}',
            'allowed_value_chars = "-+.0123456789,"',
            'device == "CAM_GIMBAL" and action == "SET" and value.count(",") == 1',
        ):
            self.assertIn(marker, bridge)
        self.assertIn('if path == "/cameraGimbal":', self.server)
        self.assertIn('action == "stabilize"', self.server)
        self.assertIn('"pan" in query and "tilt" in query', self.server)
        for marker in ('"cameraPanDeg"', '"cameraTiltDeg"', '"cameraGimbalEnabled"'):
            self.assertIn(marker, self.server)

    def test_camera_console_uses_direct_stable_stream_without_watchdog_reset(self):
        self.assertIn(":8888/stream", self.camera_ui)
        self.assertIn("object-fit:contain", self.camera_ui.replace(" ", ""))
        status_block = self.camera_ui.split("async function updateStatus()", 1)[1].split(
            "function openRawStream", 1
        )[0]
        self.assertNotIn("reconnectStream(", status_block)

    def test_camera_console_preserves_person_tracking_controls(self):
        for endpoint in (
            "/select_target?x=",
            "/select_target?clear=1",
            "/ai_tracking?state=",
        ):
            self.assertIn(endpoint, self.camera_ui)
        for removed in ("pickRedCube", "/arm/pick_cube", "KIRMIZI XYZ", "cubeVisible"):
            self.assertNotIn(removed, self.camera_ui)

    def test_server_exposes_camera_console_and_read_only_vision_state(self):
        self.assertIn('@app.get("/camera-console")', self.server)
        self.assertIn('@app.get("/api/vision/status")', self.server)
        self.assertIn('@app.get("/api/vision/control")', self.server)
        self.assertNotIn('@app.get("/arm/pick_cube")', self.server)
        self.assertIn('CAMERA_UI_FILE', self.server)

    def test_lidar_supports_local_2d_and_3d_views(self):
        for marker in (
            'id="Lidar2DView"',
            'id="Lidar3DView"',
            'id="PointCloudMap"',
            'setLidarViewMode',
            'updateLidar3D(msg)',
            'src="/assets/three.min.js"',
        ):
            self.assertIn(marker, self.main_ui)
        self.assertIn('@app.get("/assets/three.min.js")', self.server)
        self.assertTrue((ROOT / "apex_kinematics_lab" / "static" / "three.min.js").is_file())

    def test_3d_lidar_does_not_invent_height_for_planar_a1m8(self):
        block = self.main_ui.split("function updateLidar3D(msg)", 1)[1].split(
            "// KAMERA", 1
        )[0]
        self.assertIn("lidarPositions[count*3+1] = 0", block)

    def test_unstable_slam_is_not_silently_enabled(self):
        self.assertNotIn("SLAM_MAPPING_ENABLED = True", self.server)

    def test_gps_map_uses_live_filtered_telemetry_not_fixed_marker(self):
        self.assertIn('id="gpsLivePanel"', self.main_ui)
        self.assertIn("function updateGpsTelemetry(data)", self.main_ui)
        self.assertIn("updateGpsTelemetry(data);", self.main_ui)
        self.assertIn("apexMarker.setLatLng(gpsFiltered)", self.main_ui)
        self.assertIn("gpsTrail.setLatLngs(gpsTrailPoints)", self.main_ui)
        self.assertIn("data.fix === true && sats >= 3 && hasCoordinates", self.main_ui)
        self.assertIn("FIX KAYIP", self.main_ui)
        self.assertIn("İZ DONDURULDU", self.main_ui)
        self.assertNotIn("L.marker([41.068716, 28.896486]", self.main_ui)

    def test_lidar_motor_uses_live_websocket_with_http_fallback(self):
        self.assertIn("lidar_motor_ack", self.main_ui)
        self.assertIn("{cmd:'motor', state: enabled ? 1 : 0}", self.main_ui)
        self.assertIn("await _set_lidar_motor(bool(command[\"state\"]))", self.server)
        self.assertIn('/lidar/motor?state=', self.main_ui)

    def test_lidar_is_off_at_boot_and_autonomy_owns_its_motor(self):
        self.assertIn("_lidar_enabled = False", self.server)
        self.assertIn('log.info("[LIDAR] Hazır, motor başlangıçta KAPALI")', self.server)
        nav = self.server.split('async def lidar_nav(state: str = "1")', 1)[1].split(
            '# ─── ROBOT KOL', 1
        )[0]
        self.assertIn("motor = await _set_lidar_motor(True)", nav)
        self.assertIn("await _set_lidar_motor(False)", nav)

    def test_launcher_rejects_reused_stale_process_ids(self):
        launcher = (ROOT / "apex").read_text(encoding="utf-8")
        alive = launcher.split("is_alive()", 1)[1].split("camera_ready()", 1)[0]
        self.assertIn('/proc/$pid/cmdline', alive)
        self.assertIn('uvicorn apex_server:app', alive)
        self.assertIn('--port 8080', alive)

    def test_navigation_console_is_dry_run_and_does_not_replace_lidar_views(self):
        self.assertIn('onclick="openNavigationConsole()"', self.main_ui)
        self.assertIn('@app.get("/navigation-console")', self.server)
        self.assertIn('@app.websocket("/ws/navigation")', self.server)
        self.assertIn('LIDAR GÜVENLİKLİ · DÜŞÜK HIZLI ROTA YÜRÜTME', self.navigation_ui)
        self.assertIn('/api/navigation/goal', self.navigation_ui)
        self.assertIn('/api/navigation/approve', self.navigation_ui)
        self.assertIn('id="LidarMapCanvas"', self.main_ui)
        self.assertIn('id="PointCloudMap"', self.main_ui)
        for forbidden in ('/joy', '/manual', '/bodyIk', '/goHome', '/arm/'):
            self.assertNotIn(forbidden, self.navigation_ui)

    def test_navigation_preview_is_integrated_as_an_independent_main_hud_layer(self):
        for marker in (
            'id="NavigationView"',
            'id="NavigationCanvas"',
            'id="btnLidarNav"',
            "setLidarViewMode('nav')",
            "new WebSocket(`${proto}://${location.host}/ws/navigation`)",
            "postNavigationJson('/api/navigation/goal'",
            "postNavigationJson('/api/navigation/mode'",
            "postNavigationJson('/api/navigation/approve'",
            "postNavigationJson('/api/navigation/clear'",
            'LIDAR GÜVENLİKLİ · DÜŞÜK HIZLI ROTA YÜRÜTME',
        ):
            self.assertIn(marker, self.main_ui)
        self.assertIn('id="LidarMapCanvas"', self.main_ui)
        self.assertIn('id="PointCloudMap"', self.main_ui)
        navigation_block = self.main_ui.split('// ANA HUD İÇİ ROTA ÖNİZLEMESİ', 1)[1].split(
            'function lidarDepthColor', 1
        )[0]
        for forbidden in ('/joy', '/manual', '/bodyIk', '/goHome', '/arm/'):
            self.assertNotIn(forbidden, navigation_block)

    def test_navigation_has_google_maps_style_2_5d_without_fake_height(self):
        for marker in (
            'id="Navigation3DMap"',
            'id="navigation2DBtn"',
            'id="navigation3DBtn"',
            "setNavigationDimension('3d')",
            'function initNavigation3DMap()',
            'function selectNavigation3DGoal(event)',
            'function updateNavigation3DAttitude()',
            'kolonlar gerçek yükseklik değildir',
        ):
            self.assertIn(marker, self.main_ui)
        navigation_block = self.main_ui.split('// ANA HUD İÇİ ROTA ÖNİZLEMESİ', 1)[1].split(
            'function lidarDepthColor', 1
        )[0]
        self.assertIn("matrix.scale.set(resolution * .88, .22, resolution * .88)", navigation_block)
        self.assertNotIn('Math.random', navigation_block)
        for forbidden in ('/joy', '/manual', '/bodyIk', '/goHome', '/arm/'):
            self.assertNotIn(forbidden, navigation_block)

    def test_navigation_rgb_cloud_uses_camera_pixels_not_false_depth_palette(self):
        self.assertIn('/api/navigation/dense-cloud', self.navigation_ui)
        self.assertIn('TRUE RGB-D', self.navigation_ui)
        self.assertIn('RENKLİ NOKTA', self.navigation_ui)
        self.assertIn('rgb:true', self.navigation_ui)
        self.assertNotIn('function depthColor(', self.navigation_ui)

    def test_navigation_robot_pitch_matches_meta1_imu_mount_direction(self):
        self.assertIn(
            'THREE.MathUtils.degToRad(-latestMpuPitch)',
            self.main_ui,
        )
        self.assertNotIn(
            'THREE.MathUtils.degToRad(latestMpuPitch)',
            self.main_ui,
        )

    def test_navigation_websocket_is_revision_driven_and_rate_bounded(self):
        self.assertIn('NAVIGATION_BROADCAST_MAX_HZ = 10', self.server)
        self.assertIn('MAPPING_SENSOR_POLL_HZ = 20', self.server)
        self.assertIn('revision = _mapping_engine.revision()', self.server)
        self.assertIn('if revision != last_revision or now - last_sent >= 1.0:', self.server)

    def test_navigation_server_has_no_execution_endpoint(self):
        self.assertIn('MappingEngine() if MAPPING_V2_AVAILABLE else None', self.server)
        self.assertIn('@app.post("/api/navigation/goal")', self.server)
        self.assertIn('@app.post("/api/navigation/mode")', self.server)
        self.assertIn('@app.post("/api/navigation/approve")', self.server)
        self.assertNotIn('@app.post("/api/navigation/execute")', self.server)

    def test_platform_hil_heartbeat_is_not_dropped_by_generic_proxy(self):
        runner = self.server.split("async def _run_navigation_platform_test", 1)[1].split(
            "async def _begin_navigation_execution", 1
        )[0]
        self.assertIn("NAV_ROUTE_COMMAND_HZ = 10.0", self.server)
        self.assertIn("NAV_PLATFORM_TEST_REQUEST_BUDGET_S = 0.30", self.server)
        self.assertIn("_navigation_direct_request", runner)
        self.assertIn("_navigation_send_heartbeat", runner)
        self.assertIn('headers={"Connection": "close"}', runner)
        self.assertIn("max_keepalive_connections=0", runner)
        self.assertIn("timeout=1.0", runner)
        self.assertNotIn('_proxy_to_esp32("/joy"', runner)
        self.assertNotIn("_navigation_read_telemetry()", runner)
        self.assertIn("type(exc).__name__", runner)
        self.assertIn('cached_telemetry["platformTestActive"] = True', self.server)

    def test_servo_status_does_not_shift_controls_or_flash_stale_age(self):
        self.assertIn('#armStatus { height:28px; min-height:28px;', self.main_ui)
        self.assertNotIn("' · SON VERİ ' + Math.floor", self.main_ui)

    def test_telemetry_poll_is_single_flight_and_failure_debounced(self):
        self.assertIn('let telemetryRequestPending = false', self.main_ui)
        self.assertIn('if (telemetryRequestPending) return', self.main_ui)
        self.assertIn('if (telemetryFailures < 3) return', self.main_ui)
        self.assertIn('nextLogs !== lastTelemetryLogs', self.main_ui)
        self.assertIn('data.bootDiag !== lastBootDiag', self.main_ui)
        self.assertIn('TELEMETRY_CACHE_TTL_S = 0.45', self.server)

    def test_camera_control_http_fallback_retries_transient_transport_failure(self):
        self.assertIn("return fetchJson(fallbackUrl, {}, 1)", self.camera_ui)
        self.assertIn("e.name === 'AbortError'", self.camera_ui)

    def test_camera_controls_use_dedicated_websocket_with_http_fallback(self):
        self.assertIn('@app.websocket("/ws/vision")', self.server)
        self.assertIn("new WebSocket(`${proto}://${location.host}/ws/vision`)", self.camera_ui)
        for command in ("'select', {x, y}", "'clear', {}", "'ai', {state:next}"):
            self.assertIn(command, self.camera_ui)
        for fallback in ("/select_target?x=", "/select_target?clear=1", "/ai_tracking?state="):
            self.assertIn(fallback, self.camera_ui)

    def test_camera_console_visually_distinguishes_locked_and_active_tracking(self):
        self.assertIn("let targetState = 'none'", self.camera_ui)
        self.assertIn("HEDEF SEÇİLDİ · TAKİBİ BAŞLAT", self.camera_ui)
        self.assertIn("HEDEF KİLİTLİ · TAKİP HAZIR", self.camera_ui)
        self.assertIn("btn.classList.toggle('target-ready'", self.camera_ui)
        self.assertIn(".orange.active", self.camera_ui)
        self.assertIn("previousTargetState !== 'cleared'", self.camera_ui)
        self.assertNotIn('id="hudAiState"', self.camera_ui)
        self.assertNotIn("function positionLiveHudState()", self.camera_ui)
        self.assertNotIn('<div class="crosshair"></div>', self.camera_ui)

    def test_camera_supervisor_recovers_hailo_after_plain_fallback(self):
        launcher = (ROOT / "apex").read_text(encoding="utf-8")
        self.assertIn("APEX_CAMERA_HUD_RETRY_SECONDS", launcher)
        self.assertIn("Plain fallback aktif; Hailo HUD kontrollü yeniden deneniyor", launcher)
        supervisor = launcher.split("camera_supervisor_loop()", 1)[1].split("start_camera_supervisor()", 1)[0]
        self.assertIn('[[ "$(camera_mode)" == "plain" ]]', supervisor)
        self.assertIn('start_camera || true', supervisor)


if __name__ == "__main__":
    unittest.main()
